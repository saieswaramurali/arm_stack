#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

int main(int argc, char* argv[]) {
    // Initialise ROS and create the node
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
        "scene_demo",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    auto const logger = rclcpp::get_logger("scene_demo");

    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");
    auto planning_scene_interface =
        moveit::planning_interface::PlanningSceneInterface();

    // ---- 1. Add a box collision object into the scene ----
    auto const collision_object = [&move_group_interface]{
        moveit_msgs::msg::CollisionObject obj;
        obj.header.frame_id = move_group_interface.getPlanningFrame();
        obj.id = "box1";

        shape_msgs::msg::SolidPrimitive box;
        box.type = box.BOX;
        box.dimensions = {0.1, 0.4, 0.1};   // x, y, z size in metres

        geometry_msgs::msg::Pose box_pose;  // placed between arm and target
        box_pose.orientation.w = 1.0;
        box_pose.position.x = 0.3;
        box_pose.position.y = 0.0;
        box_pose.position.z = 0.4;

        obj.primitives.push_back(box);
        obj.primitive_poses.push_back(box_pose);
        obj.operation = obj.ADD;
        return obj;
    }();
    planning_scene_interface.applyCollisionObject(collision_object);
    RCLCPP_INFO(logger, "Added box1 to the planning scene");

    // ---- 2. Plan to a pose on the far side of the box: watch it route AROUND ----
    auto const target_pose = []{
        geometry_msgs::msg::Pose msg;
        msg.orientation.w = 1.0;
        msg.position.x = 0.4;
        msg.position.y = -0.3;
        msg.position.z = 0.4;
        return msg;
    }();
    move_group_interface.setPoseTarget(target_pose);

    auto plan_and_execute = [&](std::string const & label) {
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (static_cast<bool>(move_group_interface.plan(plan))) {
            RCLCPP_INFO(logger, "[%s] planning succeeded, executing", label.c_str());
            move_group_interface.execute(plan);
            return true;
        }
        RCLCPP_ERROR(logger, "[%s] planning failed!!!", label.c_str());
        return false;
    };

    if (!plan_and_execute("around the box")) { rclcpp::shutdown(); return 1; }

    // ---- 3. ATTACH the box to the arm: it becomes part of the robot ----
    // "tool0" is the UR flange frame; use your gripper link once one exists
    move_group_interface.attachObject("box1", "tool0");
    RCLCPP_INFO(logger, "Attached box1 to tool0 — it now moves (and collides) with the arm");

    // ---- 4. Plan somewhere else: planner must now keep the BOX collision-free too ----
    move_group_interface.setNamedTarget("home");
    plan_and_execute("home while carrying box");

    // ---- 5. Clean up: detach and remove ----
    move_group_interface.detachObject("box1");
    planning_scene_interface.removeCollisionObjects({"box1"});
    RCLCPP_INFO(logger, "Detached and removed box1");

    rclcpp::shutdown();
    return 0;
}
