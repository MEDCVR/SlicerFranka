#include "franka_controller/reporter.hpp"

#include "franka_controller/tip_pose.hpp"

namespace franka_controller
{

JointStateReporter::JointStateReporter(
    rclcpp::Node::SharedPtr node,
    const std::string& topic,
    const std::vector<std::string>& joint_names)
    : node_(node), joint_names_(joint_names)
{
    pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(topic, 10);
}

void JointStateReporter::report(const franka::RobotState& robot_state)
{
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = node_->now();
    msg.name = joint_names_;
    msg.position.reserve(robot_state.q.size());
    for (const double q : robot_state.q)
    {
        msg.position.push_back(q);
    }
    pub_->publish(msg);
}

TipPoseReporter::TipPoseReporter(
    rclcpp::Node::SharedPtr node,
    const std::string& topic,
    const std::string& base_frame_id,
    const Eigen::Affine3d& F_T_tip)
    : node_(node), base_frame_id_(base_frame_id), F_T_tip_(F_T_tip)
{
    pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(topic, 10);
}

void TipPoseReporter::report(const franka::RobotState& robot_state)
{
    const TipPose tip = tipPoseFromRobotState(robot_state, F_T_tip_);
    pub_->publish(tipPoseToMsg(tip, base_frame_id_, node_->now()));
}

} // namespace franka_controller
