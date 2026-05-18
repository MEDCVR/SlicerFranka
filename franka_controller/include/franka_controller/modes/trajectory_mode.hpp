#pragma once

#include <array>
#include <mutex>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>

#include "franka_controller/modes/mode.hpp"
#include "franka_controller/status_publisher.hpp"
#include "franka_controller/tip_pose.hpp"
#include "franka_controller/trajectory_planner.hpp"

namespace franka_controller
{

// Buffers tip-pose waypoints, then on `start` executes them as a single
// joint-space trajectory. Execution runs in five phases:
//
//   ENTRY_PLAN    : plan a half-speed easing path from the current
//                   commanded pose to the first user waypoint
//   ENTRY_EXECUTE : stream the planned entry samples to JIC
//   ENTRY_DWELL   : hold for entry_dwell_seconds to let transients settle
//   MAIN_PLAN     : plan the full-speed trajectory across the user waypoints
//   MAIN_EXECUTE  : stream the planned main samples
//
// Status transitions: idle → buffering → executing → done / canceled /
// error.
class TrajectoryMode : public ControlMode
{
  public:
    TrajectoryMode(
        ModeServices services,
        StatusPublisher* status,
        const TrajectoryPlanner* planner);

    void onEnter(const franka::RobotState& current) override;
    void onExit() override;
    void step() override;

  private:
    enum class Phase
    {
        IDLE,
        ENTRY_PLAN,
        ENTRY_EXECUTE,
        ENTRY_DWELL,
        MAIN_PLAN,
        MAIN_EXECUTE
    };

    void onWaypoint(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void onCommand(const std_msgs::msg::String::SharedPtr msg);

    void handleStart();
    void handleClear();
    void handleCancel();

    // Advances along trajectory_joint_samples_ by one step. Returns true if
    // the trajectory was advanced this tick (caller checks sample_index_
    // for completion). Returns false on JIC rejection (caller handles the
    // error path).
    bool advanceSamples();

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
        waypoint_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;

    StatusPublisher* status_;
    const TrajectoryPlanner* planner_;

    std::mutex mtx_;
    std::array<double, 7> current_command_q_{};
    TipPose current_command_tip_{};
    std::vector<TipPose> waypoints_;
    std::vector<TipPose> entry_waypoints_;
    std::vector<std::array<double, 7>> samples_;
    size_t sample_index_ = 0;
    Phase phase_ = Phase::IDLE;
    rclcpp::Time entry_dwell_start_;
};

} // namespace franka_controller
