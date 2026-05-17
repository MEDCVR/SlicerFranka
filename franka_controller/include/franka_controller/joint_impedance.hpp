#pragma once

#include <array>
#include <memory>
#include <mutex>
#include <vector>

#include <franka/model.h>
#include <franka/robot_state.h>

namespace franka_controller
{

// Joint-space PD impedance controller. Tracks a commanded joint position
// internally; modes update the command via setCommandJointPosition() and the
// RT loop calls impedanceControl() at ~1 kHz to compute torques.
//
// External setpoint changes are rate-limited per tick: any single call may
// not move the commanded position more than commandJointPositionToleranceRad
// (0.08 rad) away from the current commanded position on any joint. Internal
// resyncs (after a mode switch) use syncToRobotState() to bypass this check.
class JointImpedance
{
  public:
    struct Config
    {
        std::array<double, 7> k_gains = {
            400.0, 400.0, 400.0, 400.0, 200.0, 100.0, 50.0};
        std::array<double, 7> d_gains = {
            50.0, 50.0, 50.0, 50.0, 30.0, 25.0, 15.0};
        size_t dq_filter_size = 5;
    };

    JointImpedance(
        std::shared_ptr<franka::Model> model,
        const std::array<double, 7>& initial_q,
        const Config& config);

    static constexpr double commandJointPositionToleranceRad()
    {
        return 0.08;
    }

    // Force-sync the commanded position and the dq filter to the measured
    // robot state. Bypasses the tolerance check — intended for use right
    // after a mode switch or initial bring-up.
    void syncToRobotState(const franka::RobotState& robot_state);

    // The RT-loop hook. Computes tau = K*(q_cmd - q) - D*filtered(dq) and
    // rate-limits to libfranka's torque-rate cap.
    franka::Torques impedanceControl(
        const franka::RobotState& robot_state, franka::Duration period);

    // Update the commanded joint position. Returns false (and logs) if the
    // requested setpoint is more than the tolerance away on any joint.
    bool setCommandJointPosition(const std::array<double, 7>& joint_positions);
    bool setCommandJointPosition(const std::vector<double>& joint_positions);

  private:
    template <typename T>
    bool setCommandJointPositionImpl(const T& joint_positions);

    void updateDqFilter(const franka::RobotState& state);
    double filteredDq(size_t index) const;

    std::shared_ptr<franka::Model> model_;
    const std::array<double, 7> k_gains_;
    const std::array<double, 7> d_gains_;

    mutable std::mutex mtx_;
    std::array<double, 7> command_q_{};

    size_t dq_filter_size_;
    size_t dq_filter_pos_ = 0;
    std::unique_ptr<double[]> dq_buffer_;
};

} // namespace franka_controller
