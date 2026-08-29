#!/usr/bin/env python

import xml.etree.ElementTree as ET
from xml.dom import minidom
import subprocess
import yaml
import rospkg
import os
import sys
import rospy
import shutil
from convert import convert_dae_to_stl

rotor_list = []
joint_list = []
rospack = rospkg.RosPack()


def run_subprocess(cmd):
    if sys.version.split(".")[0] == "2":
        subprocess.call(cmd, shell=True)
    if sys.version.split(".")[0] == "3":
        subprocess.run(cmd, shell=True)


def get_filename(filepath):
    return filepath.rsplit("/", 1)[1]


def get_directory(filepath):
    return filepath.rsplit("/", 1)[0]


def remove_extension(filename):
    before_ext, ext = os.path.splitext(filename)
    return before_ext


def run_xacro(input_path, output_path):
    cmd = "rosrun xacro xacro {} > {}".format(input_path, output_path)
    run_subprocess(cmd)


def process_urdf(package, urdf_path, workdir_path):
    global rotor_list
    global joint_list
    rotor_list = []
    joint_list = []
    urdf_tree = ET.parse(urdf_path)
    urdf_root = urdf_tree.getroot()

    # add mujoco config
    ## mujoco tag under root
    mujoco = urdf_root.find("mujoco")
    if mujoco is None:
        mujoco = ET.Element("mujoco")
        urdf_root.append(mujoco)

    ## compiler tag under mujoco
    compiler = mujoco.find("compiler")
    if compiler is None:
        compiler = ET.SubElement(mujoco, "compiler")
    compiler.set("balanceinertia", "true")
    compiler.set("fusestatic", "false")

    # fix mesh path in visual tag
    for link in urdf_root.findall("link"):
        for link_visual in link.findall("visual"):
            for link_visual_geometry in link_visual.findall("geometry"):
                for link_visual_geometry_mesh in link_visual_geometry.findall("mesh"):
                    original_filepath = link_visual_geometry_mesh.attrib["filename"]
                    search_string = "package://"
                    index = original_filepath.find(search_string)
                    filepath_with_pkg = original_filepath[index + len(search_string):]
                    filepath_from_pkg = filepath_with_pkg[filepath_with_pkg.find("/"):]
                    filepath = rospack.get_path(package) + filepath_from_pkg
                    mujoco_mesh_path = ""

                    # generate stl in mujoco workdir
                    _, ex = os.path.splitext(filepath)
                    if(ex == ".stl" or ex == ".STL"): # used mesh is originally stl
                        shutil.copy(filepath, workdir_path)
                        mujoco_mesh_path = os.path.join(workdir_path, get_filename(filepath))

                    elif(ex == ".dae" or ex == ".DAE"): # used mesh is originally dae, need to copy stl if exist, otherwise convert to stl
                        dae_path = filepath
                        stl_path = remove_extension(filepath) + ".stl"
                        STL_path = remove_extension(filepath) + ".STL"
                        if os.path.isfile(stl_path): # stl exist, copy stl
                            shutil.copy(stl_path, workdir_path)
                            print("{} already exist, copy to {}".format(stl_path, workdir_path))
                            mujoco_mesh_path = stl_path
                        elif os.path.isfile(STL_path): # STL exist, copy STL
                            shutil.copy(STL_path, workdir_path)
                            print("{} already exist, copy to {}".format(STL_path, workdir_path))
                            mujoco_mesh_path = STL_path
                        else:
                            print("{} not exist, convert {} to stl".format(stl_path, dae_path))
                            convert_dae_to_stl(dae_path, os.path.join(workdir_path, get_filename(stl_path)))
                            mujoco_mesh_path = stl_path

                    filename = get_filename(mujoco_mesh_path)

                    # add geometry in visual tag
                    geometry_elem = ET.Element('geometry')
                    mesh_elem = ET.Element("mesh")
                    mesh_elem.set("filename", filename)
                    geometry_elem.append(mesh_elem)
                    link_visual.remove(link_visual_geometry)
                    link_visual.append(geometry_elem)

    # replace collision tag by mesh
    ## remove initial collision
    for link in urdf_root.findall("link"):
        for link_collision in link.findall("collision"):
            link.remove(link_collision)

    ## copy from visual
    for link in urdf_root.findall("link"):
        collision_tag = ET.Element("collision")
        link_name = link.attrib["name"]
        collision_tag.set("name", link_name)
        visual_exist = False
        for link_visual in link.findall("visual"):
            visual_exist = True
            for link_visual_elem in link_visual:
                collision_tag.append(link_visual_elem)
        if visual_exist:
            link.append(collision_tag)

    # get actuator list
    for transmission in urdf_root.findall("transmission"):
        for transmission_joint in transmission.findall("joint"):
            name = transmission_joint.attrib["name"]
            for transmission_joint_hardwareinterface in transmission_joint.findall("hardwareInterface"):
                if transmission_joint_hardwareinterface.text == "RotorInterface":
                    rotor_list.append(name)
                if transmission_joint_hardwareinterface.text == "hardware_interface/EffortJointInterface":
                    joint_list.append(name)
        urdf_root.remove(transmission)

    # output modified urdf
    xmlstr = minidom.parseString(ET.tostring(urdf_root)).toprettyxml(indent="  ")
    with open(urdf_path, 'w') as f:
        f.write(xmlstr)

    # remove blank lines in urdf
    cmd = "sed -i '/^[[:space:]]*$/d' {}".format(urdf_path)
    run_subprocess(cmd)


def generate_xml(urdf_path, mujoco_path):
    cmd = "rosrun mujoco compile {} {}".format(urdf_path, mujoco_path)
    run_subprocess(cmd)

def process_xml(urdf_path, mujoco_path):
    mujoco_tree = ET.parse(mujoco_path)
    mujoco_root = mujoco_tree.getroot()
    urdf_tree = ET.parse(urdf_path)
    urdf_root = urdf_tree.getroot()

    # set contents of mujoco tag in urdf to mujoco model
    urdf_mujoco_tag = urdf_root.find("mujoco")
    if urdf_mujoco_tag is not None:
        for urdf_mujoco_elem in urdf_mujoco_tag:
            mujoco_root.append(urdf_mujoco_elem)

    # map of (child, parent)
    mujoco_parent_map = dict((c, p) for p in mujoco_tree.iter() for c in p)

    # get m_f_rate from urdf
    m_f_rate = 0.0
    for m_f_rate_elem in urdf_root.iter("m_f_rate"):
        m_f_rate = m_f_rate_elem.attrib["value"]

    # process joints
    thrusts = ""
    rotor_axis_dict = {}
    joint_effort_limit_dict = {}
    for joint in mujoco_root.iter("joint"):
        if joint.get("name") is None:
            continue
        ## for rotor
        if "rotor" in joint.attrib["name"]:
            ### get control range
            thrusts = joint.attrib["range"]

            ### get rotor axis for counter torque
            axis = joint.attrib["axis"].split()[2]
            rotor_axis_dict[joint.attrib["name"]] = axis

            ### add site at rotor
            parent_body = mujoco_parent_map[joint]
            site_elem = ET.Element("site")
            site_elem.set("name", joint.attrib["name"])
            site_elem.set("pos", "0 0 0")
            parent_body.remove(joint)
            parent_body.append(site_elem)
        ## for joint
        else:
            joint_effort_limit_dict[joint.attrib["name"]] = joint.attrib["actuatorfrcrange"]

    # add free joint to root link
    ## find root link: the link which never appears as a child of any joint
    child_link_names = set()
    for urdf_joint in urdf_root.findall("joint"):
        for urdf_joint_child in urdf_joint.findall("child"):
            child_link_names.add(urdf_joint_child.attrib["link"])
    root_link_name = ""
    for link in urdf_root.findall("link"):
        if link.attrib["name"] not in child_link_names:
            root_link_name = link.attrib["name"]
            break
    if root_link_name == "":
        print("Error! root link is not found in urdf")
        sys.exit()

    ## with fusestatic="false" the urdf tree is kept
    ## only set the initial pose and add a free joint.
    is_freejoint_added = False
    for worldbody in mujoco_root.iter("worldbody"):
        for body in worldbody.findall("body"):
            if body.attrib["name"] != root_link_name:
                continue
            body.set("pos", "0 0 0.2")
            freejoint_elem = ET.Element("freejoint")
            freejoint_elem.set("name", "root_joint")
            body.insert(0, freejoint_elem)
            is_freejoint_added = True
    if not is_freejoint_added:
        print("Error! root link '{}' is not a top level body in mujoco model".format(root_link_name))
        sys.exit()

    # add site at fc
    fc_name = ""
    ## find fc name
    for baselink in urdf_root.iter("baselink"):
        fc_name = baselink.attrib["name"]
    if fc_name == "":
        print("Error! baselink is not found in urdf, please add '<baselink name=\"fc\" />' to your urdf file")
        sys.exit()

    ## with fusestatic="false" the fc link itself remains as a body, so put the site
    ## at the origin of that body. the orientation of fc is reflected as well.
    is_fc_site_added = False
    for mujoco_body in mujoco_root.iter("body"):
        if mujoco_body.attrib["name"] != fc_name:
            continue
        site_elem = ET.Element("site")
        site_elem.set("name", fc_name)
        site_elem.set("pos", "0 0 0")
        mujoco_body.append(site_elem)
        is_fc_site_added = True
    if not is_fc_site_added:
        print("Error! baselink '{}' is not found as a body in mujoco model".format(fc_name))
        sys.exit()

    # process geoms
    for geom in mujoco_root.iter("geom"):
        geom.set("contype", "1")
        geom.set("conaffinity", "0")

    # actuators
    global rotor_list
    global joint_list
    actuator_elem = ET.Element("actuator")
    ## rotors
    for rotor in rotor_list:
        rotor_elem = ET.Element("motor")
        rotor_elem.set("name", rotor)
        rotor_elem.set("ctrllimited", "true")
        rotor_elem.set("ctrlrange", thrusts)
        rotor_elem.set("gear", "0 0 1 0 0 " + str(float(m_f_rate) * float(rotor_axis_dict[rotor])))
        rotor_elem.set("site", rotor)
        actuator_elem.append(rotor_elem)
    ## joints
    for joint in joint_list:
        joint_elem = ET.Element("motor")
        joint_elem.set("name", joint)
        joint_elem.set("ctrllimited", "true")
        joint_elem.set("ctrlrange", joint_effort_limit_dict[joint])
        joint_elem.set("gear", "1")
        joint_elem.set("joint", joint)
        actuator_elem.append(joint_elem)
    mujoco_root.append(actuator_elem)

    # sensors
    sensor_elem = ET.Element("sensor")
    acc = ET.Element("accelerometer")
    acc.set("name", "acc")
    acc.set("site", "fc")
    gyro = ET.Element("gyro")
    gyro.set("name", "gyro")
    gyro.set("site", "fc")
    mag = ET.Element("magnetometer")
    mag.set("name", "mag")
    mag.set("site", "fc")
    sensor_elem.append(acc)
    sensor_elem.append(gyro)
    sensor_elem.append(mag)
    mujoco_root.append(sensor_elem)

    # include world
    mujoco_ros_utils = rospack.get_path("mujoco_ros_utils")
    world_path = os.path.join(mujoco_ros_utils, "config/world.xml")
    rel_path = os.path.relpath(world_path, get_directory(mujoco_path))
    include_elem = ET.Element("include")
    include_elem.set("file", rel_path)
    mujoco_root.append(include_elem)

    # output modified mujoco model
    xmlstr = minidom.parseString(ET.tostring(mujoco_root)).toprettyxml(indent="  ")
    with open(mujoco_path, "w") as f:
        f.write(xmlstr)

    # remove brank line in xml
    cmd = "sed -i '/^[[:space:]]*$/d' {}".format(mujoco_path)
    run_subprocess(cmd)

    # remove intermediate urdf file
    # os.remove(urdf_path)


config_path = ""
if(len(sys.argv) == 2):
    config_path = sys.argv[1]
else:
    print("Variable error! Please run following command.\nrosrun mujoco_ros_utils mujoco_model_generator.py absolute_path_to_config_file")
    sys.exit()

with open(config_path) as file:
    obj = yaml.safe_load(file)
    for package in obj["package"]:
        print(package)
        pkg_path = rospack.get_path(package)
        for (input_path, filename) in zip(obj[package]["input"], obj[package]["filename"]):
            input_xacro_path = os.path.join(pkg_path, input_path)
            workdir_path = os.path.join(pkg_path, "mujoco", filename)
            output_urdf_path = os.path.join(workdir_path, "robot.urdf")

            os.makedirs(workdir_path)

            run_xacro(input_xacro_path, output_urdf_path)

            process_urdf(package, output_urdf_path, workdir_path)

            mujoco_path = os.path.join(workdir_path, "robot.xml")

            generate_xml(output_urdf_path, mujoco_path)

            process_xml(output_urdf_path, mujoco_path)
