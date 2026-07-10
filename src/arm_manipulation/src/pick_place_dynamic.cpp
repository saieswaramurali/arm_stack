#include <chrono>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <arm_interfaces/srv/detect_objects.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace {
using DetectObjects = arm_interfaces::srv::DetectObjects;
using GripperCommand = control_msgs::action::GripperCommand;
using GripperClient = rclcpp_action::Client<GripperCommand>;
using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

enum class GripperStatus {
    kSucceeded,
    kContactTimeout,
    kFailed,
};

constexpr char kArmGroup[] = "ur_manipulator";
constexpr char kEndEffectorLink[] = "tcp";
constexpr char kCameraOpticalFrame[] = "wrist_camera_optical_frame";
constexpr double kVelocityScale = 0.25;
constexpr double kAccelerationScale = 0.25;
constexpr double kCartesianVelocityScale = 0.08;
constexpr double kCartesianAccelerationScale = 0.08;
constexpr double kBoxSizeX = 0.04;
constexpr double kBoxSizeY = 0.04;
constexpr double kBoxSizeZ = 0.06;
constexpr double kCartesianStep = 0.005;
constexpr double kJumpThreshold = 2.0;
constexpr double kMinCartesianFraction = 0.95;
constexpr int kPlanningSceneSettleMs = 150;
constexpr double kTableX = 0.0;
constexpr double kTableY = 0.75;
constexpr double kTableZ = 0.15;
constexpr double kTableSizeX = 0.8;
constexpr double kTableSizeY = 0.8;
constexpr double kTableSizeZ = 0.30;
constexpr double kPlatformX = 0.0;
constexpr double kPlatformY = 0.0;
constexpr double kPlatformZ = 0.15;
constexpr double kPlatformSizeX = 0.55;
constexpr double kPlatformSizeY = 0.55;
constexpr double kPlatformSizeZ = 0.30;
constexpr double kBinX = 0.48;
constexpr double kBinY = 0.35;
constexpr double kBinSupportHeight = 0.30;

geometry_msgs::msg::Pose make_pose(
    double x, double y, double z, double qx = 0.0, double qy = 0.0,
    double qz = 0.0, double qw = 1.0) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    pose.orientation.x = qx;
    pose.orientation.y = qy;
    pose.orientation.z = qz;
    pose.orientation.w = qw;
    return pose;
}

shape_msgs::msg::SolidPrimitive make_box(double x, double y, double z) {
    shape_msgs::msg::SolidPrimitive box;
    box.type = shape_msgs::msg::SolidPrimitive::BOX;
    box.dimensions = {x, y, z};
    return box;
}

moveit_msgs::msg::CollisionObject make_collision_box(
    const std::string& id, const std::string& frame_id,
    const geometry_msgs::msg::Pose& pose, double x, double y, double z) {
    moveit_msgs::msg::CollisionObject object;
    object.header.frame_id = frame_id;
    object.id = id;
    object.primitives.push_back(make_box(x, y, z));
    object.primitive_poses.push_back(pose);
    object.operation = moveit_msgs::msg::CollisionObject::ADD;
    return object;
}

void apply_static_scene(
    moveit::planning_interface::PlanningSceneInterface& planning_scene,
    const std::string& frame_id) {
    std::vector<moveit_msgs::msg::CollisionObject> objects;
    objects.push_back(make_collision_box(
        "table", frame_id, make_pose(kTableX, kTableY, kTableZ),
        kTableSizeX, kTableSizeY, kTableSizeZ));
    objects.push_back(make_collision_box(
        "robot_platform", frame_id, make_pose(kPlatformX, kPlatformY, kPlatformZ),
        kPlatformSizeX, kPlatformSizeY, kPlatformSizeZ));

    objects.push_back(make_collision_box(
        "drop_bin_support", frame_id, make_pose(kBinX, kBinY, kBinSupportHeight / 2.0),
        0.24, 0.20, kBinSupportHeight));
    objects.push_back(make_collision_box(
        "drop_bin_base", frame_id, make_pose(kBinX, kBinY, kBinSupportHeight + 0.005),
        0.24, 0.20, 0.01));
    objects.push_back(make_collision_box(
        "drop_bin_front_wall", frame_id, make_pose(kBinX, kBinY - 0.105, kBinSupportHeight + 0.055),
        0.24, 0.01, 0.11));
    objects.push_back(make_collision_box(
        "drop_bin_back_wall", frame_id, make_pose(kBinX, kBinY + 0.105, kBinSupportHeight + 0.055),
        0.24, 0.01, 0.11));
    objects.push_back(make_collision_box(
        "drop_bin_left_wall", frame_id, make_pose(kBinX - 0.125, kBinY, kBinSupportHeight + 0.055),
        0.01, 0.20, 0.11));
    objects.push_back(make_collision_box(
        "drop_bin_right_wall", frame_id, make_pose(kBinX + 0.125, kBinY, kBinSupportHeight + 0.055),
        0.01, 0.20, 0.11));

    planning_scene.applyCollisionObjects(objects);
    std::this_thread::sleep_for(std::chrono::milliseconds(kPlanningSceneSettleMs));
}

double trajectory_duration(const moveit_msgs::msg::RobotTrajectory& trajectory) {
    if (trajectory.joint_trajectory.points.empty()) {
        return 0.0;
    }
    auto const& last_point = trajectory.joint_trajectory.points.back();
    return static_cast<double>(last_point.time_from_start.sec) +
           static_cast<double>(last_point.time_from_start.nanosec) * 1e-9;
}

bool plan_and_execute(
    MoveGroupInterface& move_group, const rclcpp::Logger& logger,
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
    MoveGroupInterface& move_group, const rclcpp::Logger& logger,
    const std::string& step, const std::map<std::string, double>& joints) {
    move_group.setJointValueTarget(joints);
    return plan_and_execute(move_group, logger, step);
}

bool goto_pose(
    MoveGroupInterface& move_group, const rclcpp::Logger& logger,
    const std::string& step, const geometry_msgs::msg::Pose& pose) {
    move_group.setPoseTarget(pose, kEndEffectorLink);
    const bool ok = plan_and_execute(move_group, logger, step);
    move_group.clearPoseTargets();
    return ok;
}

bool retime_cartesian(
    MoveGroupInterface& move_group, moveit_msgs::msg::RobotTrajectory& trajectory,
    const rclcpp::Logger& logger, const std::string& step,
    double velocity_scale = kCartesianVelocityScale,
    double acceleration_scale = kCartesianAccelerationScale) {
    robot_trajectory::RobotTrajectory retimed(move_group.getRobotModel(), kArmGroup);
    retimed.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory);
    trajectory_processing::TimeOptimalTrajectoryGeneration totg;
    if (!totg.computeTimeStamps(
            retimed, velocity_scale, acceleration_scale)) {
        RCLCPP_ERROR(logger, "[%s] trajectory retiming failed", step.c_str());
        return false;
    }
    retimed.getRobotTrajectoryMsg(trajectory);
    RCLCPP_INFO(
        logger, "[%s] retimed duration: %.2fs",
        step.c_str(), trajectory_duration(trajectory));
    return true;
}

bool cartesian_to_pose(
    MoveGroupInterface& move_group, const rclcpp::Logger& logger,
    const std::string& step, const geometry_msgs::msg::Pose& target,
    double velocity_scale = kCartesianVelocityScale,
    double acceleration_scale = kCartesianAccelerationScale) {
    std::vector<geometry_msgs::msg::Pose> waypoints{target};
    moveit_msgs::msg::RobotTrajectory trajectory;
    move_group.setStartStateToCurrentState();
    const double fraction = move_group.computeCartesianPath(
        waypoints, kCartesianStep, kJumpThreshold, trajectory);
    RCLCPP_INFO(logger, "[%s] Cartesian coverage: %.1f%%", step.c_str(), fraction * 100.0);
    if (fraction < kMinCartesianFraction) {
        RCLCPP_WARN(logger, "[%s] Cartesian rejected; falling back to pose plan", step.c_str());
        return goto_pose(move_group, logger, step, target);
    }
    if (!retime_cartesian(move_group, trajectory, logger, step, velocity_scale, acceleration_scale)) {
        return false;
    }
    if (!static_cast<bool>(move_group.execute(trajectory))) {
        RCLCPP_ERROR(logger, "[%s] Cartesian execution failed", step.c_str());
        return false;
    }
    RCLCPP_INFO(logger, "[%s] complete", step.c_str());
    return true;
}

bool goto_camera_pose(
    MoveGroupInterface& move_group, const rclcpp::Logger& logger,
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

std::map<std::string, double> bin_carry_joints() {
    return {
        {"shoulder_pan_joint", -0.75},
        {"shoulder_lift_joint", -1.20},
        {"elbow_joint", 1.35},
        {"wrist_1_joint", -1.72},
        {"wrist_2_joint", -1.57},
        {"wrist_3_joint", 0.0},
    };
}

std::vector<std::string> gripper_touch_links() {
    return {
        "robotiq_85_left_finger_link",
        "robotiq_85_right_finger_link",
        "robotiq_85_left_finger_tip_link",
        "robotiq_85_right_finger_tip_link",
        "robotiq_85_left_inner_knuckle_link",
        "robotiq_85_right_inner_knuckle_link",
    };
}

class GripperCommander {
public:
    explicit GripperCommander(rclcpp::Node::SharedPtr node)
        : node_(std::move(node)),
          logger_(rclcpp::get_logger("pick_place_dynamic")),
          client_(rclcpp_action::create_client<GripperCommand>(
              node_, "/gripper_controller/gripper_cmd")) {}

    GripperStatus send_command(
        double position, double max_effort = 60.0,
        std::chrono::milliseconds result_timeout = std::chrono::seconds(10),
        bool timeout_means_contact = false) {
        if (!client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(logger_, "[gripper %.2f] action server unavailable", position);
            return GripperStatus::kFailed;
        }

        GripperCommand::Goal goal;
        goal.command.position = position;
        goal.command.max_effort = max_effort;
        RCLCPP_INFO(logger_, "[gripper %.2f] sending command", position);

        auto goal_handle_future = client_->async_send_goal(goal);
        if (goal_handle_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
            RCLCPP_ERROR(logger_, "[gripper %.2f] goal response timed out", position);
            return GripperStatus::kFailed;
        }
        auto goal_handle = goal_handle_future.get();
        if (!goal_handle) {
            RCLCPP_ERROR(logger_, "[gripper %.2f] goal rejected", position);
            return GripperStatus::kFailed;
        }

        auto result_future = client_->async_get_result(goal_handle);
        if (result_future.wait_for(result_timeout) != std::future_status::ready) {
            if (timeout_means_contact) {
                RCLCPP_WARN(
                    logger_,
                    "[gripper %.2f] result timed out; assuming contact and continuing",
                    position);
                return GripperStatus::kContactTimeout;
            }
            RCLCPP_ERROR(logger_, "[gripper %.2f] result timed out", position);
            return GripperStatus::kFailed;
        }

        auto const wrapped_result = result_future.get();
        if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_ERROR(logger_, "[gripper %.2f] action failed", position);
            return GripperStatus::kFailed;
        }

        RCLCPP_INFO(
            logger_, "[gripper %.2f] complete: position=%.3f effort=%.3f stalled=%s reached=%s",
            position,
            wrapped_result.result->position,
            wrapped_result.result->effort,
            wrapped_result.result->stalled ? "true" : "false",
            wrapped_result.result->reached_goal ? "true" : "false");
        return GripperStatus::kSucceeded;
    }

    bool command(double position, double max_effort = 60.0) {
        return send_command(position, max_effort) == GripperStatus::kSucceeded;
    }

    bool close_slowly(
        double target_position, double max_effort, double step,
        int pause_ms, int settle_ms, int contact_timeout_ms) {
        for (double position = step; position < target_position; position += step) {
            auto const status = send_command(
                position, max_effort, std::chrono::milliseconds(contact_timeout_ms), true);
            if (status == GripperStatus::kContactTimeout) {
                if (position < target_position) {
                    RCLCPP_INFO(
                        logger_, "[gripper %.2f] contact detected; applying final squeeze %.2f",
                        position, target_position);
                    auto const squeeze_status = send_command(
                        target_position, max_effort,
                        std::chrono::milliseconds(contact_timeout_ms), true);
                    if (squeeze_status == GripperStatus::kFailed) {
                        return false;
                    }
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(settle_ms));
                return true;
            }
            if (status == GripperStatus::kFailed) {
                return false;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(pause_ms));
        }
        auto const status = send_command(
            target_position, max_effort, std::chrono::milliseconds(contact_timeout_ms), true);
        if (status == GripperStatus::kFailed) {
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(settle_ms));
        return true;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Logger logger_;
    GripperClient::SharedPtr client_;
};

struct PickTarget {
    std::string id;
    geometry_msgs::msg::Pose tcp_pose;
    geometry_msgs::msg::Pose world_top_pose;
};

std::optional<PickTarget> choose_target(
    const DetectObjects::Response& response,
    const std::set<std::string>& used_ids) {
    for (std::size_t i = 0; i < response.ids.size(); ++i) {
        if (used_ids.count(response.ids[i]) != 0) {
            continue;
        }
        if (i >= response.world_poses.poses.size() || i >= response.tcp_poses.poses.size()) {
            continue;
        }
        return PickTarget{
            response.ids[i],
            response.tcp_poses.poses[i],
            response.world_poses.poses[i],
        };
    }
    return std::nullopt;
}

void apply_detected_boxes(
    moveit::planning_interface::PlanningSceneInterface& planning_scene,
    const std::string& frame_id,
    const DetectObjects::Response& response) {
    std::vector<moveit_msgs::msg::CollisionObject> objects;
    for (std::size_t i = 0; i < response.ids.size() && i < response.world_poses.poses.size(); ++i) {
        auto center_pose = response.world_poses.poses[i];
        center_pose.position.z -= kBoxSizeZ / 2.0;
        objects.push_back(make_collision_box(
            response.ids[i], frame_id, center_pose, kBoxSizeX, kBoxSizeY, kBoxSizeZ));
    }
    if (!objects.empty()) {
        planning_scene.applyCollisionObjects(objects);
        std::this_thread::sleep_for(std::chrono::milliseconds(kPlanningSceneSettleMs));
    }
}
}  // namespace

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
        "pick_place_dynamic",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );
    if (!node->get_parameter("use_sim_time").as_bool()) {
        node->set_parameter(rclcpp::Parameter("use_sim_time", true));
    }
    auto const logger = rclcpp::get_logger("pick_place_dynamic");

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner([&executor]() { executor.spin(); });

    MoveGroupInterface move_group(node, kArmGroup);
    move_group.setPlanningTime(10.0);
    move_group.setMaxVelocityScalingFactor(kVelocityScale);
    move_group.setMaxAccelerationScalingFactor(kAccelerationScale);
    move_group.setEndEffectorLink(kEndEffectorLink);

    double detect_x, detect_y, detect_camera_z;
    double approach_clearance, lift_height, grasp_tcp_z_offset;
    double place_x, place_y, place_z, place_tcp_z_offset;
    double grasp_close_position, grasp_max_effort, grasp_step;
    int max_picks, grasp_pause_ms, grasp_settle_ms, grasp_contact_timeout_ms;
    node->get_parameter_or("detect_x", detect_x, 0.0);
    node->get_parameter_or("detect_y", detect_y, 0.75);
    node->get_parameter_or("detect_camera_z", detect_camera_z, 0.65);
    node->get_parameter_or("approach_clearance", approach_clearance, 0.12);
    node->get_parameter_or("lift_height", lift_height, 0.12);
    node->get_parameter_or("grasp_tcp_z_offset", grasp_tcp_z_offset, -0.005);
    node->get_parameter_or("place_x", place_x, 0.48);
    node->get_parameter_or("place_y", place_y, 0.35);
    node->get_parameter_or("place_z", place_z, 0.38);
    node->get_parameter_or("place_tcp_z_offset", place_tcp_z_offset, 0.04);
    node->get_parameter_or("grasp_close_position", grasp_close_position, 0.62);
    node->get_parameter_or("grasp_max_effort", grasp_max_effort, 32.0);
    node->get_parameter_or("grasp_step", grasp_step, 0.04);
    node->get_parameter_or("grasp_pause_ms", grasp_pause_ms, 120);
    node->get_parameter_or("grasp_settle_ms", grasp_settle_ms, 700);
    node->get_parameter_or("grasp_contact_timeout_ms", grasp_contact_timeout_ms, 1500);
    node->get_parameter_or("max_picks", max_picks, 3);
    if (grasp_step <= 0.0) {
        RCLCPP_WARN(logger, "grasp_step must be positive; using 0.04");
        grasp_step = 0.04;
    }
    if (grasp_tcp_z_offset < -0.02) {
        RCLCPP_WARN(logger, "grasp_tcp_z_offset too low; clamping to -0.02");
        grasp_tcp_z_offset = -0.02;
    }
    if (place_tcp_z_offset < 0.02) {
        RCLCPP_WARN(logger, "place_tcp_z_offset too low; using 0.02");
        place_tcp_z_offset = 0.02;
    }
    if (grasp_pause_ms < 0) {
        RCLCPP_WARN(logger, "grasp_pause_ms must be non-negative; using 120");
        grasp_pause_ms = 120;
    }
    if (grasp_settle_ms < 0) {
        RCLCPP_WARN(logger, "grasp_settle_ms must be non-negative; using 700");
        grasp_settle_ms = 700;
    }
    if (grasp_contact_timeout_ms < 500) {
        RCLCPP_WARN(logger, "grasp_contact_timeout_ms too low; using 500");
        grasp_contact_timeout_ms = 500;
    }

    auto detect_client = node->create_client<DetectObjects>("/detect_objects");
    if (!detect_client->wait_for_service(std::chrono::seconds(10))) {
        RCLCPP_ERROR(logger, "Service /detect_objects unavailable");
        executor.cancel();
        spinner.join();
        rclcpp::shutdown();
        return 1;
    }

    GripperCommander gripper(node);
    moveit::planning_interface::PlanningSceneInterface planning_scene;
    std::set<std::string> used_ids;
    const auto frame_id = move_group.getPlanningFrame();
    apply_static_scene(planning_scene, frame_id);
    RCLCPP_INFO(logger, "Planning scene added table, robot platform, and drop bin");

    if (!goto_joints(move_group, logger, "goto_ready", ready_joints()) ||
        !gripper.command(0.0)) {
        executor.cancel();
        spinner.join();
        rclcpp::shutdown();
        return 1;
    }

    for (int pick_index = 0; pick_index < max_picks; ++pick_index) {
        if (!goto_camera_pose(
                move_group, logger, camera_down_pose(detect_x, detect_y, detect_camera_z))) {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1200));

        auto request = std::make_shared<DetectObjects::Request>();
        auto future = detect_client->async_send_request(request);
        if (future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
            RCLCPP_ERROR(logger, "Detection service timed out");
            break;
        }
        auto response = future.get();
        if (!response->success) {
            RCLCPP_WARN(logger, "Detection failed: %s", response->message.c_str());
            break;
        }
        apply_static_scene(planning_scene, frame_id);
        apply_detected_boxes(planning_scene, frame_id, *response);

        auto target = choose_target(*response, used_ids);
        if (!target.has_value()) {
            RCLCPP_WARN(logger, "No unused detections left");
            break;
        }
        used_ids.insert(target->id);
        RCLCPP_INFO(
            logger,
            "Using %s: world_top=(%.3f %.3f %.3f), tcp=(%.3f %.3f %.3f)",
            target->id.c_str(),
            target->world_top_pose.position.x,
            target->world_top_pose.position.y,
            target->world_top_pose.position.z,
            target->tcp_pose.position.x,
            target->tcp_pose.position.y,
            target->tcp_pose.position.z);

        const double center_z = target->world_top_pose.position.z - kBoxSizeZ / 2.0;
        auto pre_grasp = make_pose(
            target->world_top_pose.position.x,
            target->world_top_pose.position.y,
            center_z + approach_clearance,
            1.0, 0.0, 0.0, 0.0);
        if (!goto_pose(move_group, logger, "pre_grasp", pre_grasp)) {
            break;
        }

        auto grasp_pose = pre_grasp;
        grasp_pose.position.z = center_z + grasp_tcp_z_offset;
        if (!cartesian_to_pose(move_group, logger, "descend_to_grasp", grasp_pose)) {
            break;
        }

        auto target_center_pose = target->world_top_pose;
        target_center_pose.position.z = center_z;

        planning_scene.removeCollisionObjects({target->id});
        std::this_thread::sleep_for(std::chrono::milliseconds(kPlanningSceneSettleMs));
        if (!gripper.close_slowly(
                grasp_close_position, grasp_max_effort, grasp_step,
                grasp_pause_ms, grasp_settle_ms, grasp_contact_timeout_ms)) {
            break;
        }
        planning_scene.applyCollisionObject(make_collision_box(
            target->id, frame_id, target_center_pose, kBoxSizeX, kBoxSizeY, kBoxSizeZ));
        std::this_thread::sleep_for(std::chrono::milliseconds(kPlanningSceneSettleMs));
        move_group.attachObject(target->id, kEndEffectorLink, gripper_touch_links());
        std::this_thread::sleep_for(std::chrono::milliseconds(kPlanningSceneSettleMs));
        RCLCPP_INFO(logger, "[attach_object] attached %s to %s", target->id.c_str(), kEndEffectorLink);

        auto lifted_pose = grasp_pose;
        lifted_pose.position.z += lift_height;
        if (!cartesian_to_pose(move_group, logger, "retreat_with_box", lifted_pose)) {
            break;
        }

        if (!goto_joints(move_group, logger, "transport_to_bin", bin_carry_joints())) {
            break;
        }

        auto place_pose = move_group.getCurrentPose(kEndEffectorLink).pose;
        place_pose.position.x = place_x;
        place_pose.position.y = place_y;
        place_pose.position.z = place_z + place_tcp_z_offset;
        if (!cartesian_to_pose(move_group, logger, "descend_to_bin", place_pose)) {
            break;
        }
        if (!gripper.command(0.0)) {
            break;
        }
        move_group.detachObject(target->id);
        std::this_thread::sleep_for(std::chrono::milliseconds(kPlanningSceneSettleMs));
        planning_scene.removeCollisionObjects({target->id});
        RCLCPP_INFO(logger, "[detach_object] detached %s at bin", target->id.c_str());

        auto retreat_pose = place_pose;
        retreat_pose.position.z += lift_height;
        if (!cartesian_to_pose(move_group, logger, "retreat_after_place", retreat_pose)) {
            break;
        }
    }

    goto_joints(move_group, logger, "return_ready", ready_joints());
    executor.cancel();
    if (spinner.joinable()) {
        spinner.join();
    }
    rclcpp::shutdown();
    return 0;
}
