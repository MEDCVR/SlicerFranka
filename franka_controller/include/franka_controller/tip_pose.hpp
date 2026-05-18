#pragma once

#include <array>
#include <cmath>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <franka/robot_state.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/time.hpp>

namespace franka_controller
{

struct TipPose
{
    Eigen::Vector3d position{0.0, 0.0, 0.0};
    Eigen::Quaterniond orientation{1.0, 0.0, 0.0, 0.0};
};

inline Eigen::Affine3d makeAffine(
    const std::array<double, 3>& translation_m,
    const std::array<double, 4>& quaternion_xyzw)
{
    Eigen::Affine3d t = Eigen::Affine3d::Identity();
    t.translation() = Eigen::Vector3d(
        translation_m[0], translation_m[1], translation_m[2]);
    Eigen::Quaterniond q(
        quaternion_xyzw[3],
        quaternion_xyzw[0],
        quaternion_xyzw[1],
        quaternion_xyzw[2]);
    q.normalize();
    t.linear() = q.toRotationMatrix();
    return t;
}

inline double quaternionAngle(
    const Eigen::Quaterniond& a, const Eigen::Quaterniond& b)
{
    const Eigen::Quaterniond aq = a.normalized();
    const Eigen::Quaterniond bq = b.normalized();
    double dot = std::abs(aq.coeffs().dot(bq.coeffs()));
    dot = std::min(1.0, std::max(-1.0, dot));
    return 2.0 * std::acos(dot);
}

inline TipPose tipPoseFromMsg(const geometry_msgs::msg::PoseStamped& msg)
{
    TipPose out;
    out.position = Eigen::Vector3d(
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
    out.orientation = Eigen::Quaterniond(
        msg.pose.orientation.w,
        msg.pose.orientation.x,
        msg.pose.orientation.y,
        msg.pose.orientation.z);
    out.orientation.normalize();
    return out;
}

inline geometry_msgs::msg::PoseStamped tipPoseToMsg(
    const TipPose& tip,
    const std::string& frame_id,
    const rclcpp::Time& stamp)
{
    geometry_msgs::msg::PoseStamped ps;
    ps.header.stamp = stamp;
    ps.header.frame_id = frame_id;
    ps.pose.position.x = tip.position.x();
    ps.pose.position.y = tip.position.y();
    ps.pose.position.z = tip.position.z();
    ps.pose.orientation.x = tip.orientation.x();
    ps.pose.orientation.y = tip.orientation.y();
    ps.pose.orientation.z = tip.orientation.z();
    ps.pose.orientation.w = tip.orientation.w();
    return ps;
}

// Given the flange pose in base (O_T_F, from robot_state.O_T_EE) and the
// flange-to-tip transform, return the tip pose in base.
inline TipPose tipPoseFromRobotState(
    const franka::RobotState& robot_state,
    const Eigen::Affine3d& F_T_tip)
{
    Eigen::Affine3d O_T_F(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
    Eigen::Affine3d O_T_tip = O_T_F * F_T_tip;

    TipPose tip;
    tip.position = O_T_tip.translation();
    tip.orientation = Eigen::Quaterniond(O_T_tip.linear());
    tip.orientation.normalize();
    return tip;
}

// Convert a tip-space target to the corresponding flange-space target,
// given tip_T_F (inverse of F_T_tip).
inline Eigen::Affine3d tipPoseToFlangeGoal(
    const TipPose& tip, const Eigen::Affine3d& tip_T_F)
{
    Eigen::Affine3d O_T_tip = Eigen::Affine3d::Identity();
    O_T_tip.translation() = tip.position;
    O_T_tip.linear() = tip.orientation.toRotationMatrix();
    return O_T_tip * tip_T_F;
}

} // namespace franka_controller
