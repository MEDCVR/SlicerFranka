#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <franka/model.h>
#include <franka/robot.h>
#include <franka/robot_state.h>

namespace franka_controller
{

// Owns the libfranka Robot + Model, the real-time control thread, and the
// state-reporter thread. Reporters and the controller receive RobotState
// snapshots directly — there is no singleton lookup.
class FrankaInterface
{
  public:
    using ControlCallback = std::function<franka::Torques(
        const franka::RobotState& robot_state, franka::Duration period)>;

    using ReporterCallback =
        std::function<void(const franka::RobotState& robot_state)>;

    explicit FrankaInterface(const std::string& ip_address);
    ~FrankaInterface();

    FrankaInterface(const FrankaInterface&) = delete;
    FrankaInterface& operator=(const FrankaInterface&) = delete;

    std::shared_ptr<franka::Model> model();

    // Apply impedance / collision behavior to the robot. Must be called
    // before start() — libfranka rejects these after the control loop begins.
    void applyBehavior(std::function<void(franka::Robot&)> fn);

    // Synchronous, single-shot read of the current robot state. Only legal
    // before the control loop is running.
    franka::RobotState readOnce();

    // Copy of the most recent cached RobotState. Safe to call at any time
    // once start() has been called and at least one tick has elapsed.
    franka::RobotState currentStateCopy() const;

    // Register a callback invoked by the reporter thread on each tick. The
    // callback receives a snapshot of the current robot state.
    void registerReporter(ReporterCallback callback);

    // Spawn the libfranka real-time control thread. The control callback is
    // invoked at the libfranka control rate (~1 kHz). Returns immediately.
    void start(ControlCallback control_callback);

    // Spawn the reporter thread that calls all registered reporter callbacks
    // at the given rate. Must be called after start().
    void startReporters(int rate_hz);

    // Request the control loop to stop and join all threads.
    void stop();
    void join();

  private:
    bool saveRobotState(const franka::RobotState& robot_state);

    std::shared_ptr<franka::Robot> robot_;
    std::shared_ptr<franka::Model> model_;

    mutable std::mutex current_state_mtx_;
    franka::RobotState current_state_{};
    bool current_state_valid_ = false;

    ControlCallback control_callback_;
    std::thread control_thread_;
    std::atomic<bool> control_running_{false};
    std::atomic<bool> should_stop_{false};

    std::vector<ReporterCallback> reporters_;
    std::thread reporter_thread_;
    std::atomic<bool> reporter_running_{false};
};

} // namespace franka_controller
