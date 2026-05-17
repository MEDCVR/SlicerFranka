#pragma once

#include <array>
#include <cmath>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include "franka_analytical_ik/franka_ik_He.hpp"

namespace franka_controller
{

// Thin wrapper around the vendored analytical IK. We assume a flange target
// (no Franka Hand on the EE chain) — the pen/probe offset is applied by the
// caller via tipPoseToFlangeGoal before calling computeIk.
class Kinematics
{
  public:
    // Solves IK for the given flange pose. The current value of q[6] is used
    // as the redundancy parameter (with the +pi/4 flange offset applied), and
    // all of q is used as the case-consistency seed. On success q is
    // overwritten with the new joint configuration and true is returned.
    bool computeIk(
        const Eigen::Vector3d& position,
        const Eigen::Quaterniond& rotation,
        std::array<double, 7>& q) const
    {
        const double q7 = q[6] + M_PI_4;

        const std::array<double, 7> q0 = {
            q[0], q[1], q[2], q[3], q[4], q[5], q7};

        const Eigen::Matrix3d rot_mat = rotation.normalized().toRotationMatrix();

        std::array<double, 16> TF = {
            rot_mat(0, 0), rot_mat(1, 0), rot_mat(2, 0), 0.0,
            rot_mat(0, 1), rot_mat(1, 1), rot_mat(2, 1), 0.0,
            rot_mat(0, 2), rot_mat(1, 2), rot_mat(2, 2), 0.0,
            position(0),   position(1),   position(2),   1.0};

        const std::array<double, 7> qq =
            franka_ik::IK_EE_CC(TF, q7, q0, /*limit=*/true, /*flange=*/true);

        for (size_t i = 0; i < qq.size(); ++i)
        {
            if (std::isnan(qq[i]))
            {
                return false;
            }
        }

        for (size_t i = 0; i < qq.size(); ++i)
        {
            q[i] = qq[i];
        }
        q[6] -= M_PI_4;
        return true;
    }
};

} // namespace franka_controller
