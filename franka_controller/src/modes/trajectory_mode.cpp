#include "franka_controller/modes/trajectory_mode.hpp"

namespace franka_controller
{

TrajectoryMode::TrajectoryMode(
    ModeServices services,
    StatusPublisher* status,
    const TrajectoryPlanner* planner)
    : ControlMode(services), status_(status), planner_(planner)
{
    waypoint_sub_ =
        s_.node->create_subscription<geometry_msgs::msg::PoseStamped>(
            s_.config->trajectory_waypoint_topic,
            rclcpp::SystemDefaultsQoS(),
            std::bind(
                &TrajectoryMode::onWaypoint, this, std::placeholders::_1));
    command_sub_ = s_.node->create_subscription<std_msgs::msg::String>(
        s_.config->trajectory_command_topic,
        rclcpp::SystemDefaultsQoS(),
        std::bind(&TrajectoryMode::onCommand, this, std::placeholders::_1));
}

void TrajectoryMode::onEnter(const franka::RobotState& current)
{
    std::lock_guard<std::mutex> lock(mtx_);
    current_command_q_ = current.q;
    current_command_tip_ = tipPoseFromRobotState(current, s_.F_T_tip);
    phase_ = Phase::IDLE;
    samples_.clear();
    sample_index_ = 0;
    entry_waypoints_.clear();
    s_.jic->syncToRobotState(current);
    if (waypoints_.empty())
        status_->setStatus("idle");
    else
        status_->setStatus("buffering");
}

void TrajectoryMode::onExit()
{
    // If we're being swapped out mid-execution, surface that to Slicer
    // as a cancel rather than leaving the last status (e.g. "executing")
    // stuck on the topic.
    std::lock_guard<std::mutex> lock(mtx_);
    if (phase_ != Phase::IDLE)
    {
        status_->setStatus("canceled");
    }
}

void TrajectoryMode::onWaypoint(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    const TipPose wp = tipPoseFromMsg(*msg);
    std::lock_guard<std::mutex> lock(mtx_);
    if (phase_ != Phase::IDLE)
    {
        RCLCPP_ERROR(
            s_.node->get_logger(),
            "ignoring waypoint received while executing");
        return;
    }
    waypoints_.push_back(wp);
    status_->setStatus("buffering");
}

void TrajectoryMode::onCommand(const std_msgs::msg::String::SharedPtr msg)
{
    const std::string& cmd = msg->data;
    if (cmd == "clear")
    {
        handleClear();
        return;
    }
    if (cmd == "cancel")
    {
        handleCancel();
        return;
    }
    if (cmd == "start")
    {
        handleStart();
        return;
    }
    RCLCPP_WARN(
        s_.node->get_logger(),
        "unknown trajectory command: [%s]",
        cmd.c_str());
}

void TrajectoryMode::handleClear()
{
    std::lock_guard<std::mutex> lock(mtx_);
    waypoints_.clear();
    entry_waypoints_.clear();
    samples_.clear();
    sample_index_ = 0;
    phase_ = Phase::IDLE;
    status_->setStatus("idle");
}

void TrajectoryMode::handleCancel()
{
    std::lock_guard<std::mutex> lock(mtx_);
    waypoints_.clear();
    entry_waypoints_.clear();
    samples_.clear();
    sample_index_ = 0;
    phase_ = Phase::IDLE;
    status_->setStatus("canceled");
}

void TrajectoryMode::handleStart()
{
    std::lock_guard<std::mutex> lock(mtx_);
    if (waypoints_.empty())
    {
        RCLCPP_ERROR(
            s_.node->get_logger(),
            "cannot start trajectory: no waypoints buffered");
        status_->setStatus("error");
        return;
    }
    entry_waypoints_ =
        planner_->buildEntryWaypoints(current_command_tip_, waypoints_.front());
    samples_.clear();
    sample_index_ = 0;
    phase_ = Phase::ENTRY_PLAN;
    status_->setStatus("executing");
}

bool TrajectoryMode::advanceSamples()
{
    if (sample_index_ >= samples_.size())
        return true;
    const auto& next = samples_[sample_index_];
    if (!s_.jic->setCommandJointPosition(next))
    {
        return false;
    }
    current_command_q_ = next;
    sample_index_++;
    return true;
}

void TrajectoryMode::step()
{
    bool plan_requested = false;
    std::vector<TipPose> plan_waypoints;
    std::array<double, 7> plan_seed{};
    double plan_max_vel = 0.0;
    double plan_max_acc = 0.0;
    Phase plan_source_phase = Phase::IDLE;
    Phase plan_execute_phase = Phase::IDLE;

    {
        std::lock_guard<std::mutex> lock(mtx_);
        switch (phase_)
        {
        case Phase::IDLE:
            return;

        case Phase::ENTRY_PLAN:
            plan_waypoints = entry_waypoints_;
            plan_seed = current_command_q_;
            plan_max_vel =
                0.5 * s_.config->toppra_max_joint_velocity_rad_per_s;
            plan_max_acc =
                0.5 * s_.config->toppra_max_joint_acceleration_rad_per_s2;
            plan_requested = true;
            plan_source_phase = Phase::ENTRY_PLAN;
            plan_execute_phase = Phase::ENTRY_EXECUTE;
            break;

        case Phase::ENTRY_EXECUTE:
            if (!advanceSamples())
            {
                status_->setStatus("error");
                samples_.clear();
                phase_ = Phase::IDLE;
                return;
            }
            if (sample_index_ >= samples_.size())
            {
                entry_dwell_start_ = s_.node->now();
                phase_ = Phase::ENTRY_DWELL;
            }
            break;

        case Phase::ENTRY_DWELL:
        {
            const double elapsed =
                (s_.node->now() - entry_dwell_start_).seconds();
            if (elapsed >= s_.config->entry_dwell_seconds)
            {
                phase_ = Phase::MAIN_PLAN;
            }
            break;
        }

        case Phase::MAIN_PLAN:
            plan_waypoints = waypoints_;
            plan_seed = current_command_q_;
            plan_max_vel = s_.config->toppra_max_joint_velocity_rad_per_s;
            plan_max_acc = s_.config->toppra_max_joint_acceleration_rad_per_s2;
            plan_requested = true;
            plan_source_phase = Phase::MAIN_PLAN;
            plan_execute_phase = Phase::MAIN_EXECUTE;
            break;

        case Phase::MAIN_EXECUTE:
            if (!advanceSamples())
            {
                status_->setStatus("error");
                samples_.clear();
                phase_ = Phase::IDLE;
                return;
            }
            if (sample_index_ >= samples_.size())
            {
                status_->setStatus("done");
                phase_ = Phase::IDLE;
                waypoints_.clear();
                samples_.clear();
            }
            break;
        }
    }

    if (!plan_requested)
        return;

    // Run TOPP-RA outside the lock — it can take ~10-100 ms.
    std::vector<std::array<double, 7>> planned;
    const bool ok = planner_->planJointTrajectory(
        plan_waypoints, plan_seed, plan_max_vel, plan_max_acc, planned);

    std::lock_guard<std::mutex> lock(mtx_);
    if (phase_ != plan_source_phase)
    {
        // Canceled / cleared while we were planning; drop the result.
        return;
    }
    if (!ok || planned.empty())
    {
        RCLCPP_ERROR(s_.node->get_logger(), "trajectory planning failed");
        status_->setStatus("error");
        samples_.clear();
        phase_ = Phase::IDLE;
        return;
    }
    samples_ = std::move(planned);
    sample_index_ = 0;
    phase_ = plan_execute_phase;
}

} // namespace franka_controller
