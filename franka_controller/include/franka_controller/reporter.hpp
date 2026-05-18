#pragma once

#include <string>
#include <vector>

#include <eigen3/Eigen/Dense>
#include <franka/robot_state.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace franka_controller
{

// Both reporters are called from the FrankaInterface reporter thread (see
// FrankaInterface::registerReporter / startReporters). They each own their
// own publisher and do their own state extraction.

class JointStateReporter
{
  public:
    JointStateReporter(
        rclcpp::Node::SharedPtr node,
        const std::string& topic,
        const std::vector<std::string>& joint_names);

    void report(const franka::RobotState& robot_state);

  private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
    std::vector<std::string> joint_names_;
};

class TipPoseReporter
{
  public:
    TipPoseReporter(
        rclcpp::Node::SharedPtr node,
        const std::string& topic,
        const std::string& base_frame_id,
        const Eigen::Affine3d& F_T_tip);

    void report(const franka::RobotState& robot_state);

  private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
    std::string base_frame_id_;
    Eigen::Affine3d F_T_tip_;
};

inline const std::vector<std::string>& defaultPandaJointNames()
{
    static const std::vector<std::string> names = {
        "panda_joint1",
        "panda_joint2",
        "panda_joint3",
        "panda_joint4",
        "panda_joint5",
        "panda_joint6",
        "panda_joint7"};
    return names;
}

} // namespace franka_controller
