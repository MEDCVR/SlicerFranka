#pragma once

#include <array>
#include <mutex>

#include <sensor_msgs/msg/joint_state.hpp>

#include "franka_controller/modes/mode.hpp"

namespace franka_controller
{

// Tracks an external joint-position goal. On each executor tick, advances
// current_command_q_ toward goal_q_ by at most
// max_joint_velocity_rad_per_s / execution_rate_hz per joint and hands the
// new setpoint to the joint impedance controller.
class JointMode : public ControlMode
{
  public:
    explicit JointMode(ModeServices services);

    void onEnter(const franka::RobotState& current) override;
    void step() override;

  private:
    void onJointCommand(const sensor_msgs::msg::JointState::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;

    std::mutex mtx_;
    std::array<double, 7> current_command_q_{};
    std::array<double, 7> goal_q_{};
    bool goal_active_ = false;
};

} // namespace franka_controller
