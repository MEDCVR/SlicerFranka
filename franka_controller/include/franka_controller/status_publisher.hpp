#pragma once

#include <algorithm>
#include <chrono>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace franka_controller
{

// Owns the /franka/trajectory/status publisher and a wall timer that
// re-publishes the current status at status_publish_rate_hz. On
// setStatus(...) with a changed value, the new status is published
// immediately so subscribers see transitions without timer lag.
class StatusPublisher
{
  public:
    StatusPublisher(
        rclcpp::Node::SharedPtr node,
        const std::string& topic,
        int publish_rate_hz)
        : node_(node)
    {
        pub_ = node_->create_publisher<std_msgs::msg::String>(topic, 10);
        const int rate = std::max(1, publish_rate_hz);
        timer_ = node_->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / rate)),
            [this]() { publish(); });
    }

    void setStatus(const std::string& s)
    {
        {
            std::lock_guard<std::mutex> lock(mtx_);
            if (status_ == s)
                return;
            status_ = s;
        }
        publish();
    }

    std::string status() const
    {
        std::lock_guard<std::mutex> lock(mtx_);
        return status_;
    }

  private:
    void publish()
    {
        std_msgs::msg::String msg;
        {
            std::lock_guard<std::mutex> lock(mtx_);
            msg.data = status_;
        }
        pub_->publish(msg);
    }

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    mutable std::mutex mtx_;
    std::string status_ = "idle";
};

} // namespace franka_controller
