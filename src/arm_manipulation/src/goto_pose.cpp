#include <memory>

#include <rclcpp/rclcpp.hpp> 
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char* argv[]) {
    // Initialise ROS and create the node
    rclcpp::init(argc, argv) ; 
    auto const node = std::make_shared<rclcpp::Node>(
        "goto_pose", 
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    ) ; 

    auto const logger = rclcpp::get_logger("goto_pose") ; 

    // Create the MoveIt MoveGroup Interface 
    using moveit::planning_interface::MoveGroupInterface; 
    auto move_group_interface = MoveGroupInterface(node, "ur_manipulator") ; 

    // Set a target pose 
    auto const target_pose = []{
        geometry_msgs::msg::Pose msg; 
        msg.orientation.w = 1.0 ; 
        msg.position.x = 0.28 ; 
        msg.position.y = -0.2 ;
        msg.position.z = 0.5 ;  
        return msg ; 
    }() ; 
    move_group_interface.setPoseTarget(target_pose) ; 

    //Create a plan to that target pose 
    auto const [success, plan] = [&move_group_interface]{
        moveit::planning_interface::MoveGroupInterface::Plan msg ; 
        auto const ok = static_cast<bool>(move_group_interface.plan(msg)) ; 
        return std::make_pair(ok, msg) ; 
    }() ; 

    if (success) move_group_interface.execute(plan) ; 
    else RCLCPP_ERROR(logger, "Planning failed!!!") ; 

    rclcpp::shutdown() ; 

    return 0 ; 
}