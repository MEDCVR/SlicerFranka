#include "franka_controller/modes/compliant_mode.hpp"

#include <franka/rate_limiting.h>

namespace franka_controller
{

CompliantMode::CompliantMode(
    ModeServices services, std::shared_ptr<franka::Model> model)
    : ControlMode(services), model_(std::move(model))
{
}

void CompliantMode::onEnter(const franka::RobotState& current)
{
    // Re-sync the joint impedance setpoint so that whichever mode comes
    // after compliant starts from the robot's actual hand-guided position
    // instead of the last commanded setpoint.
    s_.jic->syncToRobotState(current);
}

franka::Torques CompliantMode::control(
    const franka::RobotState& state, franka::Duration /*period*/)
{
    const std::array<double, 7> coriolis = model_->coriolis(state);
    return franka::limitRate(
        franka::kMaxTorqueRate, coriolis, state.tau_J_d);
}

} // namespace franka_controller
