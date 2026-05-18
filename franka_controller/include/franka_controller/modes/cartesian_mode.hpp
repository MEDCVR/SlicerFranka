#pragma once

#include <array>
#include <mutex>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include "franka_controller/modes/mode.hpp"
#include "franka_controller/tip_pose.hpp"

namespace franka_controller
{

// Tracks an external tip-pose goal. On each executor tick:
//   1. Compute the maximum cartesian step (linear + angular) for this tick.
//   2. Try IK at progressively smaller fractions of that step until a
//      configuration is found that lies within JIC's joint-step tolerance.
//   3. Commit the new joint and tip command.
class CartesianMode : public ControlMode
{
  public:
    explicit CartesianMode(ModeServices services);

    void onEnter(const franka::RobotState& current) override;
    void step() override;

  private:
    void onTipPoseCommand(
        const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;

    std::mutex mtx_;
    std::array<double, 7> current_command_q_{};
    TipPose current_command_tip_{};
    TipPose goal_tip_{};
    bool goal_active_ = false;
};

} // namespace franka_controller
