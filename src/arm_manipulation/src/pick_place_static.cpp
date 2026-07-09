#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

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

namespace {
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
constexpr char kTableId[] = "table";
constexpr char kPlatformId[] = "robot_platform";
constexpr char kPickBoxId[] = "pick_box";
constexpr double kVelocityScale = 0.30;
constexpr double kAccelerationScale = 0.30;
constexpr double kSlowCartesianVelocityScale = 0.08;
constexpr double kSlowCartesianAccelerationScale = 0.08;
constexpr double kCarryCartesianVelocityScale = 0.15;
constexpr double kCarryCartesianAccelerationScale = 0.15;
constexpr double kPlaceCartesianVelocityScale = 0.15;
constexpr double kPlaceCartesianAccelerationScale = 0.15;
constexpr double kCartesianStep = 0.005;
constexpr double kCarryCartesianStep = 0.025;
constexpr double kShortJumpThreshold = 2.0;
constexpr double kCarryJumpThreshold = 2.0;
constexpr double kMaxShortCartesianDuration = 5.0;
constexpr double kMaxCarryDuration = 8.0;
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
constexpr double kBoxSizeX = 0.04;
constexpr double kBoxSizeY = 0.04;
constexpr double kBoxSizeZ = 0.06;

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
    std::string const& id, std::string const& frame_id,
    geometry_msgs::msg::Pose const& pose, double x, double y, double z) {
    moveit_msgs::msg::CollisionObject obj;
    obj.header.frame_id = frame_id;
    obj.id = id;
    obj.primitives.push_back(make_box(x, y, z));
    obj.primitive_poses.push_back(pose);
    obj.operation = moveit_msgs::msg::CollisionObject::ADD;
    return obj;
}

void apply_scene_objects(
    moveit::planning_interface::PlanningSceneInterface& planning_scene,
    std::string const& frame_id, geometry_msgs::msg::Pose const& box_pose) {
    auto table = make_collision_box(
        kTableId, frame_id, make_pose(kTableX, kTableY, kTableZ),
        kTableSizeX, kTableSizeY, kTableSizeZ);
    auto platform = make_collision_box(
        kPlatformId, frame_id, make_pose(kPlatformX, kPlatformY, kPlatformZ),
        kPlatformSizeX, kPlatformSizeY, kPlatformSizeZ);
    auto box = make_collision_box(
        kPickBoxId, frame_id, box_pose, kBoxSizeX, kBoxSizeY, kBoxSizeZ);
    planning_scene.applyCollisionObjects({table, platform, box});
    std::this_thread::sleep_for(std::chrono::milliseconds(kPlanningSceneSettleMs));
}

void readd_pick_box(
    moveit::planning_interface::PlanningSceneInterface& planning_scene,
    std::string const& frame_id, geometry_msgs::msg::Pose const& box_pose) {
    planning_scene.applyCollisionObject(make_collision_box(
        kPickBoxId, frame_id, box_pose, kBoxSizeX, kBoxSizeY, kBoxSizeZ));
    std::this_thread::sleep_for(std::chrono::milliseconds(kPlanningSceneSettleMs));
}

double trajectory_duration(moveit_msgs::msg::RobotTrajectory const& trajectory) {
    if (trajectory.joint_trajectory.points.empty()) {
        return 0.0;
    }
    auto const& last_point = trajectory.joint_trajectory.points.back();
    return static_cast<double>(last_point.time_from_start.sec) +
           static_cast<double>(last_point.time_from_start.nanosec) * 1e-9;
}

bool plan_and_execute(
    MoveGroupInterface& move_group, rclcpp::Logger const& logger,
    std::string const& step) {
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

bool goto_ready(MoveGroupInterface& move_group, rclcpp::Logger const& logger) {
    std::map<std::string, double> ready_joints = {
        {"shoulder_pan_joint", 0.0},
        {"shoulder_lift_joint", -1.35},
        {"elbow_joint", 1.35},
        {"wrist_1_joint", -1.57},
        {"wrist_2_joint", -1.57},
        {"wrist_3_joint", 0.0},
    };
    move_group.setJointValueTarget(ready_joints);
    return plan_and_execute(move_group, logger, "goto_ready");
}

bool goto_pose(
    MoveGroupInterface& move_group, rclcpp::Logger const& logger,
    std::string const& step, geometry_msgs::msg::Pose const& pose) {
    move_group.setPoseTarget(pose, kEndEffectorLink);
    bool const ok = plan_and_execute(move_group, logger, step);
    move_group.clearPoseTargets();
    return ok;
}

bool retime_cartesian(
    MoveGroupInterface& move_group, moveit_msgs::msg::RobotTrajectory& trajectory,
    rclcpp::Logger const& logger, std::string const& step,
    double velocity_scale, double acceleration_scale, double max_duration = 0.0) {
    robot_trajectory::RobotTrajectory retimed(move_group.getRobotModel(), kArmGroup);
    retimed.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory);
    trajectory_processing::TimeOptimalTrajectoryGeneration totg;
    if (!totg.computeTimeStamps(retimed, velocity_scale, acceleration_scale)) {
        RCLCPP_ERROR(logger, "[%s] trajectory retiming failed", step.c_str());
        return false;
    }
    retimed.getRobotTrajectoryMsg(trajectory);
    RCLCPP_INFO(
        logger, "[%s] retimed duration: %.2fs",
        step.c_str(), trajectory_duration(trajectory));
    if (max_duration > 0.0 && trajectory_duration(trajectory) > max_duration) {
        RCLCPP_WARN(
            logger, "[%s] rejected %.2fs trajectory; limit is %.2fs",
            step.c_str(), trajectory_duration(trajectory), max_duration);
        return false;
    }
    return true;
}

bool cartesian_to_pose(
    MoveGroupInterface& move_group, rclcpp::Logger const& logger,
    std::string const& step, geometry_msgs::msg::Pose const& target,
    double velocity_scale = kSlowCartesianVelocityScale,
    double acceleration_scale = kSlowCartesianAccelerationScale,
    double eef_step = kCartesianStep, double jump_threshold = 0.0,
    double max_duration = 0.0) {
    std::vector<geometry_msgs::msg::Pose> waypoints{target};
    moveit_msgs::msg::RobotTrajectory trajectory;
    move_group.setStartStateToCurrentState();
    double const fraction =
        move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    RCLCPP_INFO(logger, "[%s] Cartesian coverage: %.1f%%", step.c_str(), fraction * 100.0);
    if (fraction < kMinCartesianFraction) {
        RCLCPP_ERROR(
            logger, "[%s] Cartesian coverage below %.1f%%",
            step.c_str(), kMinCartesianFraction * 100.0);
        return false;
    }
    if (!retime_cartesian(
            move_group, trajectory, logger, step, velocity_scale, acceleration_scale,
            max_duration)) {
        return false;
    }
    if (!static_cast<bool>(move_group.execute(trajectory))) {
        RCLCPP_ERROR(logger, "[%s] execution failed", step.c_str());
        return false;
    }
    RCLCPP_INFO(logger, "[%s] complete", step.c_str());
    return true;
}

bool guarded_cartesian_or_pose(
    MoveGroupInterface& move_group, rclcpp::Logger const& logger,
    std::string const& step, geometry_msgs::msg::Pose const& target,
    double velocity_scale = kSlowCartesianVelocityScale,
    double acceleration_scale = kSlowCartesianAccelerationScale,
    double max_duration = kMaxShortCartesianDuration) {
    if (cartesian_to_pose(
            move_group, logger, step, target, velocity_scale, acceleration_scale,
            kCartesianStep, kShortJumpThreshold, max_duration)) {
        return true;
    }

    RCLCPP_WARN(logger, "[%s] Cartesian path rejected; trying pose plan", step.c_str());
    return goto_pose(move_group, logger, step, target);
}

class GripperCommander {
public:
    explicit GripperCommander(rclcpp::Node::SharedPtr node)
        : node_(std::move(node)),
          logger_(rclcpp::get_logger("pick_place_static")),
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

void shutdown(
    rclcpp::executors::SingleThreadedExecutor& executor, std::thread& spinner) {
    executor.cancel();
    if (spinner.joinable()) {
        spinner.join();
    }
    rclcpp::shutdown();
}
}  // namespace

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
        "pick_place_static",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );
    // The whole stack runs on Gazebo's clock; wall-clock trajectories get
    // aborted by the controller, so default to sim time unless overridden
    if (!node->get_parameter("use_sim_time").as_bool()) {
        node->set_parameter(rclcpp::Parameter("use_sim_time", true));
    }
    auto const logger = rclcpp::get_logger("pick_place_static");

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner([&executor]() { executor.spin(); });

    MoveGroupInterface move_group(node, kArmGroup);
    move_group.setPlanningTime(10.0);
    move_group.setMaxVelocityScalingFactor(kVelocityScale);
    move_group.setMaxAccelerationScalingFactor(kAccelerationScale);
    move_group.setEndEffectorLink(kEndEffectorLink);

    double box_x, box_y, box_z;
    double place_x, place_y, place_z;
    double approach_clearance, lift_height;
    double grasp_tcp_z_offset, place_tcp_z_offset;
    double grasp_close_position, grasp_max_effort, grasp_step;
    int grasp_pause_ms, grasp_settle_ms, grasp_contact_timeout_ms;
    node->get_parameter_or("box_x", box_x, 0.0);
    node->get_parameter_or("box_y", box_y, 0.75);
    node->get_parameter_or("box_z", box_z, 0.33);
    node->get_parameter_or("place_x", place_x, 0.25);
    node->get_parameter_or("place_y", place_y, 0.75);
    node->get_parameter_or("place_z", place_z, 0.33);
    node->get_parameter_or("approach_clearance", approach_clearance, 0.12);
    node->get_parameter_or("lift_height", lift_height, 0.10);
    node->get_parameter_or("grasp_tcp_z_offset", grasp_tcp_z_offset, -0.005);
    node->get_parameter_or("place_tcp_z_offset", place_tcp_z_offset, 0.035);
    node->get_parameter_or("grasp_close_position", grasp_close_position, 0.60);
    node->get_parameter_or("grasp_max_effort", grasp_max_effort, 28.0);
    node->get_parameter_or("grasp_step", grasp_step, 0.05);
    node->get_parameter_or("grasp_pause_ms", grasp_pause_ms, 80);
    node->get_parameter_or("grasp_settle_ms", grasp_settle_ms, 350);
    node->get_parameter_or("grasp_contact_timeout_ms", grasp_contact_timeout_ms, 1000);
    if (grasp_step <= 0.0) {
        RCLCPP_WARN(logger, "grasp_step must be positive; using 0.05");
        grasp_step = 0.05;
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
        RCLCPP_WARN(logger, "grasp_pause_ms must be non-negative; using 80");
        grasp_pause_ms = 80;
    }
    if (grasp_settle_ms < 0) {
        RCLCPP_WARN(logger, "grasp_settle_ms must be non-negative; using 350");
        grasp_settle_ms = 350;
    }
    if (grasp_contact_timeout_ms < 500) {
        RCLCPP_WARN(logger, "grasp_contact_timeout_ms too low; using 500");
        grasp_contact_timeout_ms = 500;
    }

    auto const frame_id = move_group.getPlanningFrame();
    auto const box_pose = make_pose(box_x, box_y, box_z);
    auto const place_pose = make_pose(place_x, place_y, place_z);

    moveit::planning_interface::PlanningSceneInterface planning_scene;
    apply_scene_objects(planning_scene, frame_id, box_pose);
    RCLCPP_INFO(
        logger,
        "Planning scene mirrors Gazebo in frame '%s': table and pick_box added",
        frame_id.c_str());

    GripperCommander gripper(node);
    auto fail = [&](std::string const& step) {
        RCLCPP_ERROR(logger, "Aborting pick_place_static at step '%s'", step.c_str());
        shutdown(executor, spinner);
        return 1;
    };

    auto const tool_down = [](double x, double y, double z) {
        return make_pose(x, y, z, 1.0, 0.0, 0.0, 0.0);
    };

    if (!goto_ready(move_group, logger)) {
        return fail("ready");
    }
    if (!gripper.command(0.0)) {
        return fail("open_gripper");
    }

    auto pre_grasp = tool_down(box_x, box_y, box_z + approach_clearance);
    if (!goto_pose(move_group, logger, "pre_grasp", pre_grasp)) {
        return fail("pre_grasp");
    }

    auto grasp_pose = pre_grasp;
    grasp_pose.position.z = box_z + grasp_tcp_z_offset;
    if (!guarded_cartesian_or_pose(move_group, logger, "descend_to_grasp", grasp_pose)) {
        return fail("descend_to_grasp");
    }

    planning_scene.removeCollisionObjects({kPickBoxId});
    std::this_thread::sleep_for(std::chrono::milliseconds(kPlanningSceneSettleMs));
    RCLCPP_INFO(logger, "[allow_contact] removed pick_box from world collision objects");

    if (!gripper.close_slowly(
            grasp_close_position, grasp_max_effort,
            grasp_step, grasp_pause_ms, grasp_settle_ms, grasp_contact_timeout_ms)) {
        return fail("close_gripper");
    }

    readd_pick_box(planning_scene, frame_id, box_pose);
    move_group.attachObject(kPickBoxId, kEndEffectorLink, gripper_touch_links());
    std::this_thread::sleep_for(std::chrono::milliseconds(kPlanningSceneSettleMs));
    RCLCPP_INFO(logger, "[attach_pick_box] attached pick_box to %s", kEndEffectorLink);

    auto lifted_pose = grasp_pose;
    lifted_pose.position.z += lift_height;
    if (!guarded_cartesian_or_pose(move_group, logger, "retreat_with_box", lifted_pose)) {
        return fail("retreat_with_box");
    }

    auto pre_place = tool_down(place_x, place_y, place_z + approach_clearance);
    if (!cartesian_to_pose(
            move_group, logger, "transport_to_place", pre_place,
            kCarryCartesianVelocityScale, kCarryCartesianAccelerationScale,
            kCarryCartesianStep, kCarryJumpThreshold, kMaxCarryDuration)) {
        RCLCPP_WARN(logger, "[transport_to_place] Cartesian carry rejected; trying pose plan");
        if (!goto_pose(move_group, logger, "transport_to_place", pre_place)) {
            return fail("transport_to_place");
        }
    }

    auto place_tcp_pose = pre_place;
    place_tcp_pose.position.z = place_z + place_tcp_z_offset;
    if (!guarded_cartesian_or_pose(
            move_group, logger, "descend_to_place", place_tcp_pose,
            kPlaceCartesianVelocityScale, kPlaceCartesianAccelerationScale,
            kMaxShortCartesianDuration)) {
        return fail("descend_to_place");
    }

    if (!gripper.command(0.0)) {
        return fail("open_at_place");
    }

    move_group.detachObject(kPickBoxId);
    std::this_thread::sleep_for(std::chrono::milliseconds(kPlanningSceneSettleMs));
    readd_pick_box(planning_scene, frame_id, place_pose);
    RCLCPP_INFO(logger, "[detach_pick_box] detached pick_box and updated planning scene");

    auto retreat_pose = place_tcp_pose;
    retreat_pose.position.z += lift_height;
    if (!guarded_cartesian_or_pose(move_group, logger, "retreat_after_place", retreat_pose)) {
        return fail("retreat_after_place");
    }

    if (!goto_ready(move_group, logger)) {
        return fail("return_ready");
    }

    RCLCPP_INFO(logger, "pick_place_static completed successfully");
    shutdown(executor, spinner);
    return 0;
}
