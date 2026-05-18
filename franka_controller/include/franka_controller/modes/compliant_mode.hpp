#pragma once

#include <memory>

#include <franka/model.h>

#include "franka_controller/modes/mode.hpp"

namespace franka_controller
{

// Gravity- and coriolis-compensation mode. The robot is freely backdrivable;
// the user hand-guides the tip to fiducials for registration.
class CompliantMode : public ControlMode
{
  public:
    CompliantMode(ModeServices services, std::shared_ptr<franka::Model> model);

    void onEnter(const franka::RobotState& current) override;

    franka::Torques control(
        const franka::RobotState& state, franka::Duration period) override;

  private:
    std::shared_ptr<franka::Model> model_;
};

} // namespace franka_controller
