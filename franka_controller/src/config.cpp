#include "franka_controller/config.hpp"

#include <stdexcept>

#include <rclcpp/rclcpp.hpp>

namespace franka_controller
{

namespace
{
template <typename T>
void readOrKeep(const YAML::Node& node, const std::string& key, T& out)
{
    if (const YAML::Node& v = node[key])
    {
        out = v.as<T>();
    }
}

template <typename T, size_t N>
void readArrayOrKeep(
    const YAML::Node& node, const std::string& key, std::array<T, N>& out)
{
    const YAML::Node v = node[key];
    if (!v)
        return;
    if (v.size() != N)
    {
        throw std::runtime_error(
            "config: key '" + key + "' must have " + std::to_string(N) +
            " elements, got " + std::to_string(v.size()));
    }
    for (size_t i = 0; i < N; ++i)
    {
        out[i] = v[i].as<T>();
    }
}
} // namespace

Config loadConfigFromYaml(const std::string& path)
{
    Config c;
    const YAML::Node y = YAML::LoadFile(path);

    if (const YAML::Node& v = y["ip_address"])
    {
        c.ip_address = v.as<std::string>();
    }
    else
    {
        throw std::runtime_error("config: 'ip_address' is required");
    }

    readOrKeep(y, "reporting_rate_hz", c.reporting_rate_hz);

    readOrKeep(y, "start_mode", c.start_mode);

    readOrKeep(y, "mode_topic", c.mode_topic);
    readOrKeep(y, "joint_command_topic", c.joint_command_topic);
    readOrKeep(y, "cartesian_tip_command_topic", c.cartesian_tip_command_topic);
    readOrKeep(y, "trajectory_waypoint_topic", c.trajectory_waypoint_topic);
    readOrKeep(y, "trajectory_command_topic", c.trajectory_command_topic);
    readOrKeep(y, "trajectory_status_topic", c.trajectory_status_topic);
    readOrKeep(y, "current_joint_state_topic", c.current_joint_state_topic);
    readOrKeep(y, "current_tip_pose_topic", c.current_tip_pose_topic);

    readOrKeep(y, "base_frame_id", c.base_frame_id);
    readOrKeep(y, "tip_frame_id", c.tip_frame_id);

    readArrayOrKeep(y, "flange_to_tip_translation_m",
                    c.flange_to_tip_translation_m);
    readArrayOrKeep(y, "flange_to_tip_quaternion_xyzw",
                    c.flange_to_tip_quaternion_xyzw);

    readOrKeep(y, "execution_rate_hz", c.execution_rate_hz);
    readOrKeep(y, "status_publish_rate_hz", c.status_publish_rate_hz);

    readOrKeep(y, "max_joint_velocity_rad_per_s",
               c.max_joint_velocity_rad_per_s);
    readOrKeep(y, "max_cartesian_linear_velocity_m_per_s",
               c.max_cartesian_linear_velocity_m_per_s);
    readOrKeep(y, "max_cartesian_angular_velocity_rad_per_s",
               c.max_cartesian_angular_velocity_rad_per_s);
    readOrKeep(y, "toppra_max_joint_velocity_rad_per_s",
               c.toppra_max_joint_velocity_rad_per_s);
    readOrKeep(y, "toppra_max_joint_acceleration_rad_per_s2",
               c.toppra_max_joint_acceleration_rad_per_s2);
    readOrKeep(y, "entry_dwell_seconds", c.entry_dwell_seconds);

    readOrKeep(y, "stop_on_ik_failure", c.stop_on_ik_failure);

    readArrayOrKeep(y, "k_gains", c.k_gains);
    readArrayOrKeep(y, "d_gains", c.d_gains);

    return c;
}

} // namespace franka_controller
