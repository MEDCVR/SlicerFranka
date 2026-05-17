#include "franka_controller/trajectory_planner.hpp"

#include <algorithm>
#include <cmath>

#include <rclcpp/rclcpp.hpp>

#include <toppra/algorithm/toppra.hpp>
#include <toppra/constraint/linear_joint_acceleration.hpp>
#include <toppra/constraint/linear_joint_velocity.hpp>
#include <toppra/geometric_path/piecewise_poly_path.hpp>
#include <toppra/parametrizer/const_accel.hpp>
#include <toppra/toppra.hpp>

namespace franka_controller
{

namespace
{
std::array<double, 7> vectorToArray(const Eigen::VectorXd& v)
{
    std::array<double, 7> out{};
    for (size_t i = 0; i < out.size(); ++i)
    {
        out[i] = v(static_cast<Eigen::Index>(i));
    }
    return out;
}
} // namespace

TrajectoryPlanner::TrajectoryPlanner(
    const Kinematics* ik,
    const Eigen::Affine3d& tip_T_F,
    int execution_rate_hz)
    : ik_(ik), tip_T_F_(tip_T_F), execution_rate_hz_(execution_rate_hz)
{
}

std::vector<TipPose> TrajectoryPlanner::buildEntryWaypoints(
    const TipPose& current, const TipPose& start) const
{
    TipPose w0 = current;
    TipPose w1 = start;
    w1.position = 0.5 * (current.position + start.position);

    TipPose w2 = current;
    w2.position = 0.5 * (w0.position + w1.position);
    {
        const Eigen::Quaterniond q0 = w0.orientation.normalized();
        const Eigen::Quaterniond q1 = w1.orientation.normalized();
        w2.orientation = q0.slerp(0.5, q1);
        w2.orientation.normalize();
    }

    TipPose w3 = start;
    TipPose w4 = w1;
    TipPose w5 = w1;
    const Eigen::Vector3d delta = w3.position - w1.position;
    w4.position = w1.position + (1.0 / 3.0) * delta;
    w5.position = w1.position + (2.0 / 3.0) * delta;

    return {w0, w2, w1, w4, w5, w3};
}

bool TrajectoryPlanner::planJointTrajectory(
    const std::vector<TipPose>& waypoints,
    const std::array<double, 7>& seed,
    double max_joint_velocity,
    double max_joint_acceleration,
    std::vector<std::array<double, 7>>& samples) const
{
    samples.clear();
    if (waypoints.empty())
        return false;
    if (max_joint_velocity <= 0.0 || max_joint_acceleration <= 0.0)
        return false;

    std::array<double, 7> q_seed = seed;
    std::vector<std::array<double, 7>> joint_waypoints;
    joint_waypoints.reserve(waypoints.size());

    for (size_t i = 0; i < waypoints.size(); ++i)
    {
        const Eigen::Affine3d O_T_F_goal =
            tipPoseToFlangeGoal(waypoints[i], tip_T_F_);
        const bool ik_ok = ik_->computeIk(
            O_T_F_goal.translation(),
            Eigen::Quaterniond(O_T_F_goal.linear()),
            q_seed);
        if (!ik_ok)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("trajectory_planner"),
                "IK failed at waypoint %zu",
                i);
            return false;
        }
        joint_waypoints.push_back(q_seed);
    }

    if (joint_waypoints.size() == 1)
    {
        samples.push_back(joint_waypoints.front());
        return true;
    }

    toppra::Vectors positions;
    positions.reserve(joint_waypoints.size());
    toppra::Vector times(static_cast<Eigen::Index>(joint_waypoints.size()));
    times(0) = 0.0;

    for (size_t i = 0; i < joint_waypoints.size(); ++i)
    {
        toppra::Vector q(7);
        for (size_t j = 0; j < 7; ++j)
        {
            q(static_cast<Eigen::Index>(j)) = joint_waypoints[i][j];
        }
        positions.push_back(q);

        if (i > 0)
        {
            double step = 0.0;
            for (size_t j = 0; j < 7; ++j)
            {
                const double diff =
                    joint_waypoints[i][j] - joint_waypoints[i - 1][j];
                step += diff * diff;
            }
            step = std::sqrt(step);
            if (step < 1e-6)
                step = 1e-6;
            times(static_cast<Eigen::Index>(i)) =
                times(static_cast<Eigen::Index>(i - 1)) + step;
        }
    }

    toppra::BoundaryCondFull bc{
        toppra::BoundaryCond("natural"),
        toppra::BoundaryCond("natural")};
    auto path = std::make_shared<toppra::PiecewisePolyPath>(
        toppra::PiecewisePolyPath::CubicSpline(positions, times, bc));

    const toppra::Vector v_lower =
        toppra::Vector::Constant(7, -max_joint_velocity);
    const toppra::Vector v_upper =
        toppra::Vector::Constant(7, max_joint_velocity);
    const toppra::Vector a_lower =
        toppra::Vector::Constant(7, -max_joint_acceleration);
    const toppra::Vector a_upper =
        toppra::Vector::Constant(7, max_joint_acceleration);

    auto vel_constraint =
        std::make_shared<toppra::constraint::LinearJointVelocity>(
            v_lower, v_upper);
    auto acc_constraint =
        std::make_shared<toppra::constraint::LinearJointAcceleration>(
            a_lower, a_upper);

    toppra::algorithm::TOPPRA algo({vel_constraint, acc_constraint}, path);
    const int gridpoints =
        std::max(100, static_cast<int>(joint_waypoints.size()) * 10);
    algo.setN(gridpoints);

    if (algo.computePathParametrization(0.0, 0.0) != toppra::ReturnCode::OK)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("trajectory_planner"),
            "TOPPRA failed: %s",
            algo.getErrorMessage().c_str());
        return false;
    }

    const auto& param = algo.getParameterizationData();
    auto traj = std::make_shared<toppra::parametrizer::ConstAccel>(
        path, param.gridpoints, param.parametrization);
    if (!traj->validate())
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("trajectory_planner"),
            "TOPPRA trajectory validation failed");
        return false;
    }

    const double dt = 1.0 / static_cast<double>(execution_rate_hz_);
    const auto interval = traj->pathInterval();
    const double duration = interval(1);
    if (duration <= 0.0 || dt <= 0.0)
        return false;

    for (double t = 0.0; t < duration; t += dt)
    {
        samples.push_back(vectorToArray(traj->eval_single(t, 0)));
    }
    samples.push_back(vectorToArray(traj->eval_single(duration, 0)));

    return !samples.empty();
}

} // namespace franka_controller
