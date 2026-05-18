#pragma once

#include <eigen3/Eigen/Dense>
#include <franka/robot_state.h>
#include <rclcpp/rclcpp.hpp>

#include "franka_controller/config.hpp"
#include "franka_controller/joint_impedance.hpp"
#include "franka_controller/kinematics.hpp"

namespace franka_controller
{

// Shared building blocks injected into every mode. The Controller owns the
// underlying objects; modes hold a copy of this struct (pointers + small
// values, cheap to copy).
struct ModeServices
{
    rclcpp::Node* node = nullptr;
    const Config* config = nullptr;
    JointImpedance* jic = nullptr;
    Kinematics* ik = nullptr;
    Eigen::Affine3d F_T_tip = Eigen::Affine3d::Identity();
    Eigen::Affine3d tip_T_F = Eigen::Affine3d::Identity();
};

// One mode is active at a time. The Controller swaps modes on
// /franka/control_mode; the previous mode is destroyed (taking its ROS
// subscribers with it) before the new mode is constructed.
class ControlMode
{
  public:
    explicit ControlMode(ModeServices services) : s_(services) {}
    virtual ~ControlMode() = default;

    // Called once when this mode becomes active, with a snapshot of the
    // current robot state. Modes use this to seed their commanded state and
    // to re-sync the joint impedance controller.
    virtual void onEnter(const franka::RobotState& current) = 0;

    // Called once when this mode is about to be replaced. Modes that
    // maintain external state visible to Slicer (e.g. the trajectory
    // status topic) use this hook to publish a terminal state before they
    // disappear. Default no-op.
    virtual void onExit() {}

    // Called from the executor thread at config.execution_rate_hz. Default
    // is a no-op (suitable for modes whose work happens entirely in
    // control()). JointMode / CartesianMode / TrajectoryMode override.
    virtual void step() {}

    // Called from the libfranka RT thread at ~1 kHz. Default delegates to
    // the joint impedance controller — the right behavior for joint /
    // cartesian / trajectory modes. CompliantMode overrides to return
    // gravity-compensation torques instead.
    virtual franka::Torques control(
        const franka::RobotState& state, franka::Duration period)
    {
        return s_.jic->impedanceControl(state, period);
    }

  protected:
    ModeServices s_;
};

} // namespace franka_controller
