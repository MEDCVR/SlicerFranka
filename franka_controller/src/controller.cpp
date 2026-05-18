#include "franka_controller/controller.hpp"

#include "franka_controller/modes/cartesian_mode.hpp"
#include "franka_controller/modes/compliant_mode.hpp"
#include "franka_controller/modes/joint_mode.hpp"
#include "franka_controller/modes/trajectory_mode.hpp"

namespace franka_controller
{

Controller::Controller(
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<FrankaInterface> interface,
    ModeServices services,
    std::shared_ptr<franka::Model> model,
    std::shared_ptr<StatusPublisher> status,
    std::shared_ptr<TrajectoryPlanner> planner,
    const franka::RobotState& initial_state)
    : node_(node),
      interface_(std::move(interface)),
      services_(services),
      model_(std::move(model)),
      status_(std::move(status)),
      planner_(std::move(planner)),
      config_(*services.config)
{
    current_mode_ = createMode(config_.start_mode);
    if (!current_mode_)
    {
        RCLCPP_ERROR(
            node_->get_logger(),
            "start_mode [%s] unknown; falling back to 'joint'",
            config_.start_mode.c_str());
        current_mode_ = createMode("joint");
    }
    current_mode_->onEnter(initial_state);

    mode_sub_ = node_->create_subscription<std_msgs::msg::String>(
        config_.mode_topic,
        rclcpp::SystemDefaultsQoS(),
        std::bind(&Controller::onModeMsg, this, std::placeholders::_1));

    executor_thread_ = std::thread(&Controller::executorLoop, this);
}

Controller::~Controller()
{
    stop_requested_ = true;
    if (executor_thread_.joinable())
        executor_thread_.join();
}

franka::Torques Controller::control(
    const franka::RobotState& state, franka::Duration period)
{
    const auto mode = currentMode();
    return mode->control(state, period);
}

std::shared_ptr<ControlMode> Controller::currentMode() const
{
    std::lock_guard<std::mutex> lock(mode_mtx_);
    return current_mode_;
}

void Controller::onModeMsg(const std_msgs::msg::String::SharedPtr msg)
{
    changeMode(msg->data);
}

void Controller::changeMode(const std::string& name)
{
    auto new_mode = createMode(name);
    if (!new_mode)
    {
        RCLCPP_WARN(
            node_->get_logger(),
            "ignoring unknown mode request: [%s]",
            name.c_str());
        return;
    }

    const franka::RobotState snapshot = interface_->currentStateCopy();
    new_mode->onEnter(snapshot);

    {
        std::lock_guard<std::mutex> lock(mode_mtx_);
        current_mode_.swap(new_mode);
    }
    // After the swap `new_mode` holds the previous mode. Give it a chance
    // to publish any terminal state before it gets destroyed.
    if (new_mode)
        new_mode->onExit();

    RCLCPP_INFO(node_->get_logger(), "mode set to [%s]", name.c_str());
}

std::shared_ptr<ControlMode> Controller::createMode(const std::string& name)
{
    if (name == "joint")
    {
        return std::make_shared<JointMode>(services_);
    }
    if (name == "cartesian")
    {
        return std::make_shared<CartesianMode>(services_);
    }
    if (name == "compliant")
    {
        return std::make_shared<CompliantMode>(services_, model_);
    }
    if (name == "trajectory")
    {
        return std::make_shared<TrajectoryMode>(
            services_, status_.get(), planner_.get());
    }
    return nullptr;
}

void Controller::executorLoop()
{
    rclcpp::Rate rate(config_.execution_rate_hz);
    while (rclcpp::ok() && !stop_requested_.load())
    {
        const auto mode = currentMode();
        if (mode)
        {
            mode->step();
        }
        rate.sleep();
    }
}

} // namespace franka_controller
