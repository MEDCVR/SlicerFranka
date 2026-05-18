#include "franka_controller/franka_interface.hpp"

#include <stdexcept>

#include <franka/exception.h>
#include <rclcpp/rclcpp.hpp>

namespace franka_controller
{

FrankaInterface::FrankaInterface(const std::string& ip_address)
    : robot_(std::make_shared<franka::Robot>(ip_address))
{
}

FrankaInterface::~FrankaInterface()
{
    stop();
    join();
}

std::shared_ptr<franka::Model> FrankaInterface::model()
{
    if (!model_)
        model_ = std::make_shared<franka::Model>(robot_->loadModel());
    return model_;
}

void FrankaInterface::applyBehavior(std::function<void(franka::Robot&)> fn)
{
    if (control_running_.load())
    {
        throw std::runtime_error(
            "FrankaInterface::applyBehavior() must be called before start()");
    }
    fn(*robot_);
}

franka::RobotState FrankaInterface::readOnce()
{
    if (control_running_.load())
    {
        RCLCPP_WARN(
            rclcpp::get_logger("franka_controller"),
            "readOnce() called while control loop is running; returning cached state");
        return currentStateCopy();
    }
    franka::RobotState s = robot_->readOnce();
    {
        std::lock_guard<std::mutex> lock(current_state_mtx_);
        current_state_ = s;
        current_state_valid_ = true;
    }
    return s;
}

franka::RobotState FrankaInterface::currentStateCopy() const
{
    std::lock_guard<std::mutex> lock(current_state_mtx_);
    return current_state_;
}

void FrankaInterface::registerReporter(ReporterCallback callback)
{
    if (reporter_running_.load())
    {
        throw std::runtime_error(
            "FrankaInterface::registerReporter() must be called before "
            "startReporters()");
    }
    reporters_.push_back(std::move(callback));
}

void FrankaInterface::start(ControlCallback control_callback)
{
    if (control_running_.exchange(true))
    {
        throw std::runtime_error(
            "FrankaInterface::start() called twice");
    }
    control_callback_ = std::move(control_callback);

    control_thread_ = std::thread([this]() {
        auto loop = [this](
                        const franka::RobotState& state,
                        franka::Duration period) -> franka::Torques {
            if (should_stop_.load())
            {
                franka::Torques zero_torques{
                    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
                return franka::MotionFinished(zero_torques);
            }
            saveRobotState(state);
            return control_callback_(state, period);
        };
        try
        {
            robot_->control(loop);
        }
        catch (const franka::Exception& e)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("franka_controller"),
                "libfranka exception in control loop: %s; requesting shutdown",
                e.what());
            // Stop the reporter thread (so we don't keep publishing stale
            // cached state) and break out of executor.spin() in main.
            should_stop_ = true;
            if (rclcpp::ok())
                rclcpp::shutdown();
        }
        control_running_ = false;
    });
}

void FrankaInterface::startReporters(int rate_hz)
{
    if (reporters_.empty())
    {
        RCLCPP_WARN(
            rclcpp::get_logger("franka_controller"),
            "startReporters() called with no registered reporters");
        return;
    }
    if (reporter_running_.exchange(true))
    {
        throw std::runtime_error(
            "FrankaInterface::startReporters() called twice");
    }

    reporter_thread_ = std::thread([this, rate_hz]() {
        rclcpp::Rate rate(rate_hz);
        while (rclcpp::ok() && !should_stop_.load())
        {
            franka::RobotState snapshot;
            bool valid = false;
            {
                std::lock_guard<std::mutex> lock(current_state_mtx_);
                snapshot = current_state_;
                valid = current_state_valid_;
            }
            if (valid)
            {
                for (auto& f : reporters_)
                    f(snapshot);
            }
            rate.sleep();
        }
        reporter_running_ = false;
    });
}

void FrankaInterface::stop()
{
    should_stop_ = true;
}

void FrankaInterface::join()
{
    if (control_thread_.joinable())
        control_thread_.join();
    if (reporter_thread_.joinable())
        reporter_thread_.join();
}

bool FrankaInterface::saveRobotState(const franka::RobotState& robot_state)
{
    std::lock_guard<std::mutex> lock(current_state_mtx_);
    current_state_ = robot_state;
    current_state_valid_ = true;
    return !should_stop_.load();
}

} // namespace franka_controller
