#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <franka/model.h>
#include <franka/robot_state.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "franka_controller/config.hpp"
#include "franka_controller/franka_interface.hpp"
#include "franka_controller/modes/mode.hpp"
#include "franka_controller/status_publisher.hpp"
#include "franka_controller/trajectory_planner.hpp"

namespace franka_controller
{

// The orchestrator. Owns the current mode, the executor thread, and the
// /franka/control_mode subscriber that drives mode swaps.
//
// Threading:
//   * The libfranka RT thread calls control() at ~1 kHz.
//   * An owned executor thread calls current_mode_->step() at
//     config.execution_rate_hz.
//   * The ROS executor thread runs the mode-change subscriber callback.
//
// All three access current_mode_; the mutex is held only across pointer
// swaps and brief copies of the shared_ptr, never across mode construction
// or step() work.
class Controller
{
  public:
    Controller(
        rclcpp::Node::SharedPtr node,
        std::shared_ptr<FrankaInterface> interface,
        ModeServices services,
        std::shared_ptr<franka::Model> model,
        std::shared_ptr<StatusPublisher> status,
        std::shared_ptr<TrajectoryPlanner> planner,
        const franka::RobotState& initial_state);

    ~Controller();

    Controller(const Controller&) = delete;
    Controller& operator=(const Controller&) = delete;

    // Bound and handed to FrankaInterface::start() as the control callback.
    franka::Torques control(
        const franka::RobotState& state, franka::Duration period);

  private:
    void onModeMsg(const std_msgs::msg::String::SharedPtr msg);
    void changeMode(const std::string& name);
    std::shared_ptr<ControlMode> createMode(const std::string& name);
    void executorLoop();

    std::shared_ptr<ControlMode> currentMode() const;

    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<FrankaInterface> interface_;
    ModeServices services_;
    std::shared_ptr<franka::Model> model_;
    std::shared_ptr<StatusPublisher> status_;
    std::shared_ptr<TrajectoryPlanner> planner_;
    const Config& config_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;

    mutable std::mutex mode_mtx_;
    std::shared_ptr<ControlMode> current_mode_;

    std::thread executor_thread_;
    std::atomic<bool> stop_requested_{false};
};

} // namespace franka_controller
