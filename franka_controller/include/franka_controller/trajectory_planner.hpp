#pragma once

#include <array>
#include <vector>

#include <eigen3/Eigen/Dense>

#include "franka_controller/kinematics.hpp"
#include "franka_controller/tip_pose.hpp"

namespace franka_controller
{

// Pure trajectory planning. No threading, no ROS, no global state. Takes
// kinematics + tip transform + execution rate at construction; each call
// produces either entry waypoints or a sampled joint trajectory.
class TrajectoryPlanner
{
  public:
    TrajectoryPlanner(
        const Kinematics* ik,
        const Eigen::Affine3d& tip_T_F,
        int execution_rate_hz);

    // Builds 6 intermediate tip poses that ease the robot from `current` to
    // `start` (the first user-supplied waypoint). The blend interpolates
    // position linearly with a half-step at the midpoint and SLERPs the
    // orientation across the same span.
    std::vector<TipPose> buildEntryWaypoints(
        const TipPose& current, const TipPose& start) const;

    // Runs IK on each waypoint (using `seed` for the first, then the
    // previous solution as the seed for the next), fits a cubic spline in
    // joint space, parameterizes via TOPP-RA under the given velocity /
    // acceleration limits, and samples the result at execution_rate_hz.
    // Returns true on success and fills `samples`.
    bool planJointTrajectory(
        const std::vector<TipPose>& waypoints,
        const std::array<double, 7>& seed,
        double max_joint_velocity_rad_per_s,
        double max_joint_acceleration_rad_per_s2,
        std::vector<std::array<double, 7>>& samples) const;

  private:
    const Kinematics* ik_;
    Eigen::Affine3d tip_T_F_;
    int execution_rate_hz_;
};

} // namespace franka_controller
