#include "franka_controller/modes/joint_mode.hpp"

#include <algorithm>
#include <cmath>

namespace franka_controller
{

JointMode::JointMode(ModeServices services) : ControlMode(services)
{
    sub_ = s_.node->create_subscription<sensor_msgs::msg::JointState>(
        s_.config->joint_command_topic,
        rclcpp::SystemDefaultsQoS(),
        std::bind(&JointMode::onJointCommand, this, std::placeholders::_1));
}

void JointMode::onEnter(const franka::RobotState& current)
{
    std::lock_guard<std::mutex> lock(mtx_);
    current_command_q_ = current.q;
    goal_active_ = false;
    s_.jic->syncToRobotState(current);
}

void JointMode::onJointCommand(
    const sensor_msgs::msg::JointState::SharedPtr msg)
{
    if (msg->position.size() < 7)
    {
        RCLCPP_ERROR(
            s_.node->get_logger(),
            "JointState command has %zu positions, expected at least 7",
            msg->position.size());
        return;
    }
    std::lock_guard<std::mutex> lock(mtx_);
    for (size_t i = 0; i < 7; ++i)
    {
        goal_q_[i] = msg->position[i];
    }
    goal_active_ = true;
}

void JointMode::step()
{
    std::lock_guard<std::mutex> lock(mtx_);
    if (!goal_active_)
        return;

    const double max_step = s_.config->max_joint_velocity_rad_per_s /
                            static_cast<double>(s_.config->execution_rate_hz);

    std::array<double, 7> next = current_command_q_;
    bool reached = true;
    for (size_t i = 0; i < 7; ++i)
    {
        const double diff = goal_q_[i] - next[i];
        if (std::abs(diff) > max_step)
        {
            next[i] += (diff > 0.0 ? 1.0 : -1.0) * max_step;
            reached = false;
        }
        else
        {
            next[i] = goal_q_[i];
        }
    }

    if (!s_.jic->setCommandJointPosition(next))
    {
        RCLCPP_ERROR(s_.node->get_logger(), "joint setpoint rejected by JIC");
        goal_active_ = false;
        return;
    }

    current_command_q_ = next;
    if (reached)
    {
        goal_active_ = false;
    }
}

} // namespace franka_controller
