#include <memory>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "franka_controller/config.hpp"
#include "franka_controller/controller.hpp"
#include "franka_controller/franka_interface.hpp"
#include "franka_controller/joint_impedance.hpp"
#include "franka_controller/kinematics.hpp"
#include "franka_controller/modes/mode.hpp"
#include "franka_controller/reporter.hpp"
#include "franka_controller/status_publisher.hpp"
#include "franka_controller/tip_pose.hpp"
#include "franka_controller/trajectory_planner.hpp"

namespace
{
std::string resolveConfigPath(int argc, char** argv)
{
    for (int i = 1; i + 1 < argc; ++i)
    {
        const std::string arg = argv[i];
        if (arg == "--config" || arg == "-c")
        {
            return argv[i + 1];
        }
    }
    if (const char* env = std::getenv("FRANKA_CONTROLLER_CONFIG"))
    {
        return env;
    }
    throw std::runtime_error(
        "config path required (pass --config <path> or set "
        "FRANKA_CONTROLLER_CONFIG)");
}
} // namespace

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    const std::string config_path = resolveConfigPath(argc, argv);
    const franka_controller::Config config =
        franka_controller::loadConfigFromYaml(config_path);

    RCLCPP_INFO(
        rclcpp::get_logger("franka_controller"),
        "loaded config from %s, connecting to %s",
        config_path.c_str(),
        config.ip_address.c_str());

    auto interface =
        std::make_shared<franka_controller::FrankaInterface>(config.ip_address);

    interface->applyBehavior([](franka::Robot& r) {
        r.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
        r.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
        r.setCollisionBehavior(
            {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
            {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
            {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
            {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
            {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
            {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
            {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
            {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});
    });

    const franka::RobotState initial_state = interface->readOnce();
    auto model = interface->model();

    franka_controller::JointImpedance::Config jic_config;
    jic_config.k_gains = config.k_gains;
    jic_config.d_gains = config.d_gains;
    auto jic = std::make_shared<franka_controller::JointImpedance>(
        model, initial_state.q, jic_config);

    franka_controller::Kinematics ik;

    const Eigen::Affine3d F_T_tip = franka_controller::makeAffine(
        config.flange_to_tip_translation_m,
        config.flange_to_tip_quaternion_xyzw);

    auto node = std::make_shared<rclcpp::Node>("franka_controller");

    auto joint_reporter = std::make_shared<franka_controller::JointStateReporter>(
        node,
        config.current_joint_state_topic,
        franka_controller::defaultPandaJointNames());
    auto tip_reporter = std::make_shared<franka_controller::TipPoseReporter>(
        node, config.current_tip_pose_topic, config.base_frame_id, F_T_tip);

    interface->registerReporter(
        [joint_reporter](const franka::RobotState& s) {
            joint_reporter->report(s);
        });
    interface->registerReporter([tip_reporter](const franka::RobotState& s) {
        tip_reporter->report(s);
    });

    const Eigen::Affine3d tip_T_F = F_T_tip.inverse();

    franka_controller::ModeServices services;
    services.node = node.get();
    services.config = &config;
    services.jic = jic.get();
    services.ik = &ik;
    services.F_T_tip = F_T_tip;
    services.tip_T_F = tip_T_F;

    auto status = std::make_shared<franka_controller::StatusPublisher>(
        node, config.trajectory_status_topic, config.status_publish_rate_hz);
    auto planner = std::make_shared<franka_controller::TrajectoryPlanner>(
        &ik, tip_T_F, config.execution_rate_hz);

    auto controller = std::make_shared<franka_controller::Controller>(
        node, interface, services, model, status, planner, initial_state);

    interface->start(
        [controller](
            const franka::RobotState& state, franka::Duration period) {
            return controller->control(state, period);
        });
    interface->startReporters(config.reporting_rate_hz);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    interface->stop();
    interface->join();
    rclcpp::shutdown();
    return 0;
}
