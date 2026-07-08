#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char* argv[]) {
    // Initialise ROS and create the node
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
        "goto_named",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    auto const logger = rclcpp::get_logger("goto_named");

    // Named state from the command line, default "home" (must exist as a
    // <group_state> in the SRDF: home, up, test_configuration)
    std::string const target = (argc > 1) ? argv[1] : "home";

    // Create the MoveIt MoveGroup Interface
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");

    if (!move_group_interface.setNamedTarget(target)) {
        RCLCPP_ERROR(logger, "Unknown named state '%s'", target.c_str());
        rclcpp::shutdown();
        return 1;
    }

    // Plan and execute
    auto const [success, plan] = [&move_group_interface]{
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group_interface.plan(msg));
        return std::make_pair(ok, msg);
    }();

    if (success) {
        move_group_interface.execute(plan);
        RCLCPP_INFO(logger, "Reached '%s'", target.c_str());
    } else {
        RCLCPP_ERROR(logger, "Planning to '%s' failed!!!", target.c_str());
    }

    rclcpp::shutdown();
    return 0;
}
