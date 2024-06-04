#pragma once

#include <urdf/model.h>
#include <pluginlib/class_list_macros.h>
#include <mujoco_ros_control/mujoco_robot_hw_sim.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>

namespace mujoco_ros_control
{
  class DefaultRobotHWSim : public mujoco_ros_control::RobotHWSim
  {
  public:
    DefaultRobotHWSim() {};
    ~DefaultRobotHWSim() {}

    bool virtual init(const std::string& robot_namespace,
              ros::NodeHandle model_nh,
              mjModel* mujoco_model,
              mjData* mujoco_data
              );

    void virtual read(const ros::Time& time, const ros::Duration& period);

    void virtual write(const ros::Time& time, const ros::Duration& period);

    void controlInputCallback(const sensor_msgs::JointState & msg);

    void jointPositionCallback(const sensor_msgs::JointState & msg);
    void rootPoseCallback(const geometry_msgs::Pose & msg);

  protected:
    mjModel* mujoco_model_;
    mjData* mujoco_data_;

    std::vector<std::string> joint_list_;
    ros::Publisher joint_state_pub_;
    ros::Subscriber control_input_sub_;
    ros::Subscriber dierct_joint_position_sub_;
    ros::Subscriber dierct_root_pose_sub_;
    double joint_state_pub_rate_ = 0.02;
    std::vector<double> control_input_;

    ros::Time last_joint_state_time_;

    sensor_msgs::JointState direct_joint_position_;
    geometry_msgs::Pose direct_root_pose_;
    bool direct_root_pose_flag_;

  };
}
