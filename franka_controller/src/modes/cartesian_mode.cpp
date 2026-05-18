#include "franka_controller/modes/cartesian_mode.hpp"

#include <algorithm>
#include <cmath>

namespace franka_controller
{

namespace
{
constexpr int kMaxAdaptiveAttempts = 16;
constexpr double kPositionReachedTolerance = 1e-5;
constexpr double kRotationReachedTolerance = 1e-4;
} // namespace

CartesianMode::CartesianMode(ModeServices services) : ControlMode(services)
{
    sub_ = s_.node->create_subscription<geometry_msgs::msg::PoseStamped>(
        s_.config->cartesian_tip_command_topic,
        rclcpp::SystemDefaultsQoS(),
        std::bind(
            &CartesianMode::onTipPoseCommand, this, std::placeholders::_1));
}

void CartesianMode::onEnter(const franka::RobotState& current)
{
    std::lock_guard<std::mutex> lock(mtx_);
    current_command_q_ = current.q;
    current_command_tip_ = tipPoseFromRobotState(current, s_.F_T_tip);
    goal_active_ = false;
    s_.jic->syncToRobotState(current);
}

void CartesianMode::onTipPoseCommand(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    const TipPose goal = tipPoseFromMsg(*msg);
    std::lock_guard<std::mutex> lock(mtx_);
    goal_tip_ = goal;
    goal_active_ = true;
}

void CartesianMode::step()
{
    std::lock_guard<std::mutex> lock(mtx_);
    if (!goal_active_)
        return;

    const TipPose current = current_command_tip_;
    const TipPose goal = goal_tip_;

    const double max_pos_step =
        s_.config->max_cartesian_linear_velocity_m_per_s /
        static_cast<double>(s_.config->execution_rate_hz);
    const double max_rot_step =
        s_.config->max_cartesian_angular_velocity_rad_per_s /
        static_cast<double>(s_.config->execution_rate_hz);

    const Eigen::Vector3d delta_p = goal.position - current.position;
    const double dist = delta_p.norm();
    const double pos_scale =
        (dist > 1e-9) ? std::min(1.0, max_pos_step / dist) : 1.0;

    const Eigen::Quaterniond q0 = current.orientation.normalized();
    const Eigen::Quaterniond q1 = goal.orientation.normalized();
    const double angle = quaternionAngle(q0, q1);
    const double rot_scale =
        (angle > 1e-9) ? std::min(1.0, max_rot_step / angle) : 1.0;

    const double joint_step_limit =
        0.95 * JointImpedance::commandJointPositionToleranceRad();

    for (int attempt = 0; attempt < kMaxAdaptiveAttempts; ++attempt)
    {
        const double adaptive_scale = std::ldexp(1.0, -attempt);

        TipPose next = current;
        if (dist > 1e-9)
        {
            next.position =
                current.position + adaptive_scale * pos_scale * delta_p;
        }
        if (angle > 1e-9)
        {
            next.orientation = q0.slerp(adaptive_scale * rot_scale, q1);
            next.orientation.normalize();
        }

        const Eigen::Affine3d O_T_F_goal = tipPoseToFlangeGoal(next, s_.tip_T_F);
        std::array<double, 7> q_command = current_command_q_;

        const bool ik_ok = s_.ik->computeIk(
            O_T_F_goal.translation(),
            Eigen::Quaterniond(O_T_F_goal.linear()),
            q_command);
        if (!ik_ok)
            continue;

        bool within_joint_step = true;
        for (size_t i = 0; i < 7; ++i)
        {
            if (std::abs(q_command[i] - current_command_q_[i]) >
                joint_step_limit)
            {
                within_joint_step = false;
                break;
            }
        }
        if (!within_joint_step)
            continue;

        if (!s_.jic->setCommandJointPosition(q_command))
            continue;

        current_command_q_ = q_command;
        current_command_tip_ = next;

        const bool pos_reached =
            (goal.position - next.position).norm() < kPositionReachedTolerance;
        const bool rot_reached =
            quaternionAngle(goal.orientation, next.orientation) <
            kRotationReachedTolerance;
        if (pos_reached && rot_reached)
        {
            goal_active_ = false;
        }
        return;
    }

    if (s_.config->stop_on_ik_failure)
    {
        RCLCPP_ERROR(
            s_.node->get_logger(),
            "cartesian step: no joint config within tolerance after %d "
            "attempts",
            kMaxAdaptiveAttempts);
        goal_active_ = false;
    }
}

} // namespace franka_controller
