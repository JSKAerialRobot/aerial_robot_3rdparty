#include <mujoco_ros_control/mujoco_default_robot_hw_sim.h>

namespace mujoco_ros_control
{

  bool DefaultRobotHWSim::init(const std::string& robot_namespace,
                              ros::NodeHandle model_nh,
                              mjModel* mujoco_model,
                              mjData* mujoco_data
                              )
  {
    mujoco_model_ = mujoco_model;
    mujoco_data_ = mujoco_data;

    control_input_.resize(mujoco_model_->nu);
    joint_list_.resize(0);

    // get joint names from mujoco model
    for(int i = 0; i < mujoco_model_->njnt; i++)
      {
        if(mujoco_model_->jnt_type[i] > 1)
          {
            joint_list_.push_back(mj_id2name(mujoco_model_, mjtObj_::mjOBJ_JOINT, i));
          }
      }

    joint_state_pub_ = model_nh.advertise<sensor_msgs::JointState>("joint_states", 1);
    control_input_sub_ = model_nh.subscribe("mujoco/ctrl_input", 1, &DefaultRobotHWSim::controlInputCallback, this);

    dierct_joint_position_sub_ = model_nh.subscribe("mujoco/direct_joint_position", 1, &DefaultRobotHWSim::jointPositionCallback, this);
    dierct_root_pose_sub_ = model_nh.subscribe("mujoco/direct_root_pose", 1, &DefaultRobotHWSim::rootPoseCallback, this);
    direct_root_pose_flag_ = false;

    return true;
  }

  void DefaultRobotHWSim::read(const ros::Time& time, const ros::Duration& period)
  {
    if((time - last_joint_state_time_).toSec() >= joint_state_pub_rate_)
      {
        sensor_msgs::JointState joint_state_msg;
        joint_state_msg.header.stamp = time;
        joint_state_msg.name = joint_list_;

        mjtNum* qpos = mujoco_data_->qpos;
        int* jnt_qposadr = mujoco_model_->jnt_qposadr;
        mjtNum* qvel = mujoco_data_->qvel;
        int* jnt_dofadr = mujoco_model_->jnt_dofadr;
        for(int i = 0; i < mujoco_model_->njnt; i++)
          {
            if(mujoco_model_->jnt_type[i] > 1)
              {
                joint_state_msg.position.push_back(qpos[jnt_qposadr[i]]);
                joint_state_msg.velocity.push_back(qvel[jnt_dofadr[i]]);
              }
          }
        joint_state_pub_.publish(joint_state_msg);
        last_joint_state_time_ = time;
      }
  }

  void DefaultRobotHWSim::write(const ros::Time& time, const ros::Duration& period)
  {
    for(int i = 0; i < control_input_.size(); i++)
      {
        mujoco_data_->ctrl[i] = control_input_.at(i);
      }

    // set the root pose directly if necessary
    if (direct_root_pose_flag_) {
      mjtNum* qpos = mujoco_data_->qpos;
      // check the type of the first joint
      switch (mujoco_model_->jnt_type[0]) {
      case mjtJoint_::mjJNT_FREE:
        {
          qpos[0] = direct_root_pose_.position.x;
          qpos[1] = direct_root_pose_.position.y;
          qpos[2] = direct_root_pose_.position.z;
          qpos[3] = direct_root_pose_.orientation.w;
          qpos[4] = direct_root_pose_.orientation.x;
          qpos[5] = direct_root_pose_.orientation.y;
          qpos[6] = direct_root_pose_.orientation.z;
          break;
        }
      case mjtJoint_::mjJNT_BALL:
        {
          qpos[0] = direct_root_pose_.orientation.w;
          qpos[1] = direct_root_pose_.orientation.x;
          qpos[2] = direct_root_pose_.orientation.y;
          qpos[3] = direct_root_pose_.orientation.z;
          break;
        }
      default:
        {
          break;
        }
      }
      direct_root_pose_flag_ = false;
    }

    // set the joint position directly if necessary
    const auto names = direct_joint_position_.name;
    const auto target_positions = direct_joint_position_.position;
    if (names.size() > 0)
      {
        for(int i = 0; i < names.size(); i++)
          {
            int id = mj_name2id(mujoco_model_, mjtObj_::mjOBJ_JOINT, names.at(i).c_str());
            if(id == -1)
              {
                ROS_WARN_STREAM("mujoco: joint name " <<  names.at(i) << " does not exist");
                continue;
              }

            mjtNum* qpos = mujoco_data_->qpos;
            int* jnt_qposadr = mujoco_model_->jnt_qposadr;
            qpos[jnt_qposadr[id]] = target_positions.at(i);


          }
        direct_joint_position_.name.resize(0);
        direct_joint_position_.position.resize(0);
      }
  }

  void DefaultRobotHWSim::controlInputCallback(const sensor_msgs::JointState & msg)
  {
    if(msg.name.size() != msg.position.size())
      {
        ROS_WARN("mujoco: size of actuator names and size of input is not same.");
      }
    for(int i = 0; i < msg.name.size(); i++)
      {
        int actuator_id = mj_name2id(mujoco_model_, mjtObj_::mjOBJ_ACTUATOR, msg.name.at(i).c_str());
        if(actuator_id == -1)
          {
            ROS_WARN_STREAM("mujoco: joint name " <<  msg.name.at(i) << " does not exist");
          }
        else
          {
            control_input_.at(actuator_id) = msg.position.at(i);
          }
      }
  }

  void DefaultRobotHWSim::jointPositionCallback(const sensor_msgs::JointState & msg)
  {
    if(msg.name.size() != msg.position.size())
      {
        ROS_WARN("mujoco: size of actuator names and size of input is not same.");
        return;
      }

    direct_joint_position_ = msg;
  }

  void DefaultRobotHWSim::rootPoseCallback(const geometry_msgs::Pose & msg)
  {
    // check the validity of quaternion

    direct_root_pose_ = msg;
    direct_root_pose_flag_ = true;
  }

}

PLUGINLIB_EXPORT_CLASS(mujoco_ros_control::DefaultRobotHWSim, mujoco_ros_control::RobotHWSim)
