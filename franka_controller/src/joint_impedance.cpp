#include "franka_controller/joint_impedance.hpp"

#include <algorithm>
#include <cmath>

#include <franka/rate_limiting.h>
#include <rclcpp/rclcpp.hpp>

namespace franka_controller
{

JointImpedance::JointImpedance(
    std::shared_ptr<franka::Model> model,
    const std::array<double, 7>& initial_q,
    const Config& config)
    : model_(std::move(model)),
      k_gains_(config.k_gains),
      d_gains_(config.d_gains),
      dq_filter_size_(config.dq_filter_size)
{
    command_q_ = initial_q;
    dq_buffer_ = std::make_unique<double[]>(dq_filter_size_ * 7);
    std::fill(&dq_buffer_[0], &dq_buffer_[dq_filter_size_ * 7], 0.0);
}

bool JointImpedance::setCommandJointPosition(
    const std::array<double, 7>& joint_positions)
{
    return setCommandJointPositionImpl(joint_positions);
}

bool JointImpedance::setCommandJointPosition(
    const std::vector<double>& joint_positions)
{
    return setCommandJointPositionImpl(joint_positions);
}

template <typename T>
bool JointImpedance::setCommandJointPositionImpl(const T& joint_positions)
{
    const double tolerance = commandJointPositionToleranceRad();
    std::lock_guard<std::mutex> lock(mtx_);
    if (joint_positions.size() > command_q_.size())
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("joint_impedance"),
            "command has %zu joints, expected %zu",
            joint_positions.size(),
            command_q_.size());
        return false;
    }
    for (size_t i = 0; i < joint_positions.size(); ++i)
    {
        if (std::abs(joint_positions[i] - command_q_[i]) > tolerance)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("joint_impedance"),
                "joint %zu: command %f is more than %f rad from current %f",
                i,
                joint_positions[i],
                tolerance,
                command_q_[i]);
            return false;
        }
    }
    for (size_t i = 0; i < joint_positions.size(); ++i)
    {
        command_q_[i] = joint_positions[i];
    }
    return true;
}

void JointImpedance::syncToRobotState(const franka::RobotState& robot_state)
{
    std::lock_guard<std::mutex> lock(mtx_);
    for (size_t i = 0; i < 7; ++i)
    {
        command_q_[i] = robot_state.q[i];
    }
    for (size_t i = 0; i < dq_filter_size_; ++i)
    {
        for (size_t j = 0; j < 7; ++j)
        {
            dq_buffer_[i * 7 + j] = robot_state.dq[j];
        }
    }
    dq_filter_pos_ = 0;
}

void JointImpedance::updateDqFilter(const franka::RobotState& state)
{
    for (size_t i = 0; i < 7; ++i)
    {
        dq_buffer_[dq_filter_pos_ * 7 + i] = state.dq[i];
    }
    dq_filter_pos_ = (dq_filter_pos_ + 1) % dq_filter_size_;
}

double JointImpedance::filteredDq(size_t index) const
{
    double sum = 0.0;
    for (size_t i = index; i < 7 * dq_filter_size_; i += 7)
    {
        sum += dq_buffer_[i];
    }
    return sum / static_cast<double>(dq_filter_size_);
}

franka::Torques JointImpedance::impedanceControl(
    const franka::RobotState& robot_state, franka::Duration /*period*/)
{
    updateDqFilter(robot_state);
    std::array<double, 7> tau_d{};
    {
        std::lock_guard<std::mutex> lock(mtx_);
        for (size_t i = 0; i < 7; ++i)
        {
            tau_d[i] = k_gains_[i] * (command_q_[i] - robot_state.q[i])
                     - d_gains_[i] * filteredDq(i);
        }
    }
    return franka::limitRate(
        franka::kMaxTorqueRate, tau_d, robot_state.tau_J_d);
}

} // namespace franka_controller
