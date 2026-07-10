#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <thread>

#include <arm_interfaces/srv/detect_objects.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace {
using DetectObjects = arm_interfaces::srv::DetectObjects;
using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

constexpr char kArmGroup[] = "ur_manipulator";
constexpr char kCameraOpticalFrame[] = "wrist_camera_optical_frame";
constexpr double kVelocityScale = 0.25;
constexpr double kAccelerationScale = 0.25;

double trajectory_duration(const moveit_msgs::msg::RobotTrajectory& trajectory) {
    if (trajectory.joint_trajectory.points.empty()) {
        return 0.0;
    }
    const auto& last_point = trajectory.joint_trajectory.points.back();
    return static_cast<double>(last_point.time_from_start.sec) +
           static_cast<double>(last_point.time_from_start.nanosec) * 1e-9;
}

bool plan_and_execute(
    MoveGroupInterface& move_group,
    const rclcpp::Logger& logger,
    const std::string& step) {
    MoveGroupInterface::Plan plan;
    if (!static_cast<bool>(move_group.plan(plan))) {
        RCLCPP_ERROR(logger, "[%s] planning failed", step.c_str());
        return false;
    }
    RCLCPP_INFO(
        logger, "[%s] planned duration: %.2fs",
        step.c_str(), trajectory_duration(plan.trajectory_));
    if (!static_cast<bool>(move_group.execute(plan))) {
        RCLCPP_ERROR(logger, "[%s] execution failed", step.c_str());
        return false;
    }
    RCLCPP_INFO(logger, "[%s] complete", step.c_str());
    return true;
}

bool goto_joints(
    MoveGroupInterface& move_group,
    const rclcpp::Logger& logger,
    const std::string& step,
    const std::map<std::string, double>& joints) {
    move_group.setJointValueTarget(joints);
    return plan_and_execute(move_group, logger, step);
}

bool goto_camera_pose(
    MoveGroupInterface& move_group,
    const rclcpp::Logger& logger,
    const geometry_msgs::msg::Pose& pose) {
    move_group.setPoseTarget(pose, kCameraOpticalFrame);
    const bool ok = plan_and_execute(move_group, logger, "goto_detect_camera_pose");
    move_group.clearPoseTargets();
    return ok;
}

geometry_msgs::msg::Pose camera_down_pose(double x, double y, double z) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    tf2::Quaternion q;
    q.setRPY(M_PI, 0.0, 0.0);
    pose.orientation = tf2::toMsg(q);
    return pose;
}

std::map<std::string, double> ready_joints() {
    return {
        {"shoulder_pan_joint", 0.0},
        {"shoulder_lift_joint", -1.35},
        {"elbow_joint", 1.35},
        {"wrist_1_joint", -1.57},
        {"wrist_2_joint", -1.57},
        {"wrist_3_joint", 0.0},
    };
}

void log_pose(
    const rclcpp::Logger& logger,
    const std::string& id,
    const std::string& frame,
    const geometry_msgs::msg::Pose& pose) {
    tf2::Quaternion q;
    tf2::fromMsg(pose.orientation, q);
    double roll;
    double pitch;
    double yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    RCLCPP_INFO(
        logger,
        "%s %s: x=%.4f y=%.4f z=%.4f rpy=(%.1f %.1f %.1f)deg",
        id.c_str(),
        frame.c_str(),
        pose.position.x,
        pose.position.y,
        pose.position.z,
        roll * 180.0 / M_PI,
        pitch * 180.0 / M_PI,
        yaw * 180.0 / M_PI);
}

void log_current_camera_pose(
    MoveGroupInterface& move_group,
    const rclcpp::Logger& logger) {
    const auto pose = move_group.getCurrentPose(kCameraOpticalFrame);
    log_pose(logger, "actual_camera", pose.header.frame_id, pose.pose);
}
}  // namespace

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>(
        "tune_pick_place",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
    if (!node->get_parameter("use_sim_time").as_bool()) {
        node->set_parameter(rclcpp::Parameter("use_sim_time", true));
    }
    const auto logger = rclcpp::get_logger("tune_pick_place");

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner([&executor]() { executor.spin(); });

    MoveGroupInterface move_group(node, kArmGroup);
    move_group.setPlanningTime(10.0);
    move_group.setMaxVelocityScalingFactor(kVelocityScale);
    move_group.setMaxAccelerationScalingFactor(kAccelerationScale);

    double detect_x, detect_y, detect_camera_z;
    int settle_ms;
    node->get_parameter_or("detect_x", detect_x, 0.0);
    node->get_parameter_or("detect_y", detect_y, 0.75);
    node->get_parameter_or("detect_camera_z", detect_camera_z, 0.65);
    node->get_parameter_or("settle_ms", settle_ms, 1200);

    auto detect_client = node->create_client<DetectObjects>("/detect_objects");
    if (!detect_client->wait_for_service(std::chrono::seconds(10))) {
        RCLCPP_ERROR(logger, "Service /detect_objects unavailable");
        executor.cancel();
        spinner.join();
        rclcpp::shutdown();
        return 1;
    }

    bool ok = goto_joints(move_group, logger, "goto_ready", ready_joints()) &&
              goto_camera_pose(
                  move_group, logger, camera_down_pose(detect_x, detect_y, detect_camera_z));
    if (!ok) {
        executor.cancel();
        spinner.join();
        rclcpp::shutdown();
        return 1;
    }
    log_current_camera_pose(move_group, logger);

    std::this_thread::sleep_for(std::chrono::milliseconds(settle_ms));
    auto request = std::make_shared<DetectObjects::Request>();
    auto future = detect_client->async_send_request(request);
    if (future.wait_for(std::chrono::seconds(3)) != std::future_status::ready) {
        RCLCPP_ERROR(logger, "Detection service timed out");
        executor.cancel();
        spinner.join();
        rclcpp::shutdown();
        return 1;
    }

    const auto response = future.get();
    RCLCPP_INFO(
        logger,
        "detect_objects success=%s message='%s' count=%zu world_frame='%s' tcp_frame='%s'",
        response->success ? "true" : "false",
        response->message.c_str(),
        response->ids.size(),
        response->world_poses.header.frame_id.c_str(),
        response->tcp_poses.header.frame_id.c_str());

    for (std::size_t i = 0; i < response->ids.size(); ++i) {
        if (i < response->world_poses.poses.size()) {
            log_pose(logger, response->ids[i], "world_top", response->world_poses.poses[i]);
        }
        if (i < response->tcp_poses.poses.size()) {
            log_pose(logger, response->ids[i], "tcp_top", response->tcp_poses.poses[i]);
        }
    }

    executor.cancel();
    if (spinner.joinable()) {
        spinner.join();
    }
    rclcpp::shutdown();
    return 0;
}
