#pragma once

#include <array>
#include <string>

#include <yaml-cpp/yaml.h>

namespace franka_controller
{

struct Config
{
    std::string ip_address;
    int reporting_rate_hz = 50;

    std::string start_mode = "joint";

    std::string mode_topic = "/franka/control_mode";
    std::string joint_command_topic = "/franka/command/joint_state";
    std::string cartesian_tip_command_topic = "/franka/command/tip_pose";
    std::string trajectory_waypoint_topic = "/franka/trajectory/waypoint";
    std::string trajectory_command_topic = "/franka/trajectory/command";
    std::string trajectory_status_topic = "/franka/trajectory/status";

    std::string current_joint_state_topic = "/franka/current/joint_state";
    std::string current_tip_pose_topic = "/franka/current/tip_pose";

    std::string base_frame_id = "panda_link0";
    std::string tip_frame_id = "pointer_tip";

    std::array<double, 3> flange_to_tip_translation_m = {0.0, 0.0, 0.14};
    std::array<double, 4> flange_to_tip_quaternion_xyzw = {0.0, 0.0, 0.0, 1.0};

    int execution_rate_hz = 250;
    int status_publish_rate_hz = 5;

    double max_joint_velocity_rad_per_s = 0.1;
    double max_cartesian_linear_velocity_m_per_s = 0.02;
    double max_cartesian_angular_velocity_rad_per_s = 0.10;
    double toppra_max_joint_velocity_rad_per_s = 1.0;
    double toppra_max_joint_acceleration_rad_per_s2 = 0.5;
    double entry_dwell_seconds = 0.25;

    bool stop_on_ik_failure = true;

    std::array<double, 7> k_gains = {
        600.0, 900.0, 600.0, 300.0, 300.0, 200.0, 100.0};
    std::array<double, 7> d_gains = {
        50.0, 70.0, 70.0, 60.0, 30.0, 15.0, 10.0};
};

Config loadConfigFromYaml(const std::string& path);

} // namespace franka_controller
