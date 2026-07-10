#include <chrono>
#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include <arm_interfaces/srv/detect_objects.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/task_constructor/container.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/task.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
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
namespace mtc = moveit::task_constructor;
namespace stages = moveit::task_constructor::stages;
namespace solvers = moveit::task_constructor::solvers;
using StageList = std::vector<mtc::Stage::pointer>;

enum class GripperStatus {
    kSucceeded,
    kContactTimeout,
    kFailed,
};

constexpr char kArmGroup[] = "ur_manipulator";
constexpr char kEndEffectorLink[] = "tcp";
constexpr char kCameraOpticalFrame[] = "wrist_camera_optical_frame";
constexpr double kBoxSizeX = 0.04;
constexpr double kBoxSizeY = 0.04;
constexpr double kBoxSizeZ = 0.06;
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
constexpr double kCartesianStep = 0.005;
constexpr double kJumpThreshold = 2.0;
constexpr double kMinCartesianFraction = 0.90;
constexpr double kCartesianVelocityScale = 0.08;
constexpr double kCartesianAccelerationScale = 0.08;

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

geometry_msgs::msg::PoseStamped stamped_pose(
    const std::string& frame_id, const geometry_msgs::msg::Pose& pose) {
    geometry_msgs::msg::PoseStamped stamped;
    stamped.header.frame_id = frame_id;
    stamped.pose = pose;
    return stamped;
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

std::map<std::string, double> startup_joints() {
    return {
        {"shoulder_pan_joint", 0.0},
        {"shoulder_lift_joint", -1.35},
        {"elbow_joint", 1.35},
        {"wrist_1_joint", -1.57},
        {"wrist_2_joint", -1.57},
        {"wrist_3_joint", 0.0},
    };
}

std::map<std::string, double> detect_elbow_up_joints() {
    return {
        {"shoulder_pan_joint", 1.6057},
        {"shoulder_lift_joint", -0.9076},
        {"elbow_joint", 0.9076},
        {"wrist_1_joint", -1.5708},
        {"wrist_2_joint", -1.5708},
        {"wrist_3_joint", 0.0},
    };
}

geometry_msgs::msg::Pose tool_down(double x, double y, double z) {
    return make_pose(x, y, z, 1.0, 0.0, 0.0, 0.0);
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
          logger_(rclcpp::get_logger("pick_place_mtc")),
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

        auto const result = result_future.get();
        if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_ERROR(logger_, "[gripper %.2f] action failed", position);
            return GripperStatus::kFailed;
        }
        return GripperStatus::kSucceeded;
    }

    bool command(double position, double effort = 60.0) {
        return send_command(position, effort) == GripperStatus::kSucceeded;
    }

    bool close_slowly(
        double target_position, double max_effort, double step,
        int pause_ms, int settle_ms, int contact_timeout_ms) {
        for (double position = step; position < target_position; position += step) {
            auto const status = send_command(
                position, max_effort, std::chrono::milliseconds(contact_timeout_ms), true);
            if (status == GripperStatus::kContactTimeout) {
                if (position < target_position) {
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
    geometry_msgs::msg::Pose world_top_pose;
};

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
    planning_scene.applyCollisionObjects(objects);
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
    }
}

std::optional<PickTarget> choose_target(
    const DetectObjects::Response& response,
    const std::set<std::string>& used_ids) {
    for (std::size_t i = 0; i < response.ids.size(); ++i) {
        if (used_ids.count(response.ids[i]) != 0) {
            continue;
        }
        if (i >= response.world_poses.poses.size()) {
            continue;
        }
        return PickTarget{response.ids[i], response.world_poses.poses[i]};
    }
    return std::nullopt;
}

std::unique_ptr<stages::MoveTo> move_to_joint_stage(
    const std::string& name,
    const solvers::PlannerInterfacePtr& planner,
    const std::map<std::string, double>& joints) {
    auto stage = std::make_unique<stages::MoveTo>(name, planner);
    stage->setGroup(kArmGroup);
    stage->setGoal(joints);
    return stage;
}

std::unique_ptr<stages::MoveTo> move_to_pose_stage(
    const std::string& name,
    const solvers::PlannerInterfacePtr& planner,
    const std::string& frame_id,
    const geometry_msgs::msg::Pose& pose,
    const std::string& ik_frame = kEndEffectorLink) {
    auto stage = std::make_unique<stages::MoveTo>(name, planner);
    stage->setGroup(kArmGroup);
    stage->setIKFrame(ik_frame);
    stage->setGoal(stamped_pose(frame_id, pose));
    return stage;
}

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
    const std::string& step,
    int attempts = 3) {
    for (int attempt = 1; attempt <= attempts; ++attempt) {
        move_group.setStartStateToCurrentState();
        MoveGroupInterface::Plan plan;
        if (!static_cast<bool>(move_group.plan(plan))) {
            RCLCPP_WARN(logger, "[%s] planning attempt %d failed", step.c_str(), attempt);
            continue;
        }
        RCLCPP_INFO(
            logger, "[%s] planned duration: %.2fs",
            step.c_str(), trajectory_duration(plan.trajectory_));
        if (static_cast<bool>(move_group.execute(plan))) {
            RCLCPP_INFO(logger, "[%s] complete", step.c_str());
            return true;
        }
        RCLCPP_WARN(logger, "[%s] execution attempt %d failed", step.c_str(), attempt);
    }
    RCLCPP_ERROR(logger, "[%s] failed after %d attempts", step.c_str(), attempts);
    return false;
}

bool goto_joints(
    MoveGroupInterface& move_group,
    const rclcpp::Logger& logger,
    const std::string& step,
    const std::map<std::string, double>& joints) {
    move_group.setJointValueTarget(joints);
    return plan_and_execute(move_group, logger, step);
}

bool goto_pose(
    MoveGroupInterface& move_group,
    const rclcpp::Logger& logger,
    const std::string& step,
    const geometry_msgs::msg::Pose& pose,
    const std::string& link = kEndEffectorLink) {
    move_group.setPoseTarget(pose, link);
    const bool ok = plan_and_execute(move_group, logger, step);
    move_group.clearPoseTargets();
    return ok;
}

bool retime_cartesian(
    MoveGroupInterface& move_group,
    moveit_msgs::msg::RobotTrajectory& trajectory,
    const rclcpp::Logger& logger,
    const std::string& step) {
    robot_trajectory::RobotTrajectory retimed(move_group.getRobotModel(), kArmGroup);
    retimed.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory);
    trajectory_processing::TimeOptimalTrajectoryGeneration totg;
    if (!totg.computeTimeStamps(
            retimed, kCartesianVelocityScale, kCartesianAccelerationScale)) {
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
    MoveGroupInterface& move_group,
    const rclcpp::Logger& logger,
    const std::string& step,
    const geometry_msgs::msg::Pose& target) {
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
    if (!retime_cartesian(move_group, trajectory, logger, step)) {
        return false;
    }
    if (!static_cast<bool>(move_group.execute(trajectory))) {
        RCLCPP_ERROR(logger, "[%s] Cartesian execution failed", step.c_str());
        return false;
    }
    RCLCPP_INFO(logger, "[%s] complete", step.c_str());
    return true;
}

bool validate_task(
    const rclcpp::Node::SharedPtr& node,
    const rclcpp::Logger& logger,
    const std::string& name,
    StageList stages_to_add,
    std::size_t max_solutions = 8) {
    mtc::Task task(name);
    task.loadRobotModel(node);
    task.setTimeout(12.0);
    task.add(std::make_unique<stages::CurrentState>("current"));
    for (auto& stage : stages_to_add) {
        task.add(std::move(stage));
    }

    try {
        task.init();
        auto const result = task.plan(max_solutions);
        if (!result || task.numSolutions() == 0) {
            RCLCPP_WARN(logger, "[%s] MTC found no solution", name.c_str());
            task.explainFailure(std::cout);
            return false;
        }
        task.publishAllSolutions(false);
        RCLCPP_INFO(
            logger, "[%s] MTC validated %zu solution(s)",
            name.c_str(), task.numSolutions());
        return true;
    } catch (const std::exception& ex) {
        RCLCPP_ERROR(logger, "[%s] MTC exception: %s", name.c_str(), ex.what());
        return false;
    }
}

}  // namespace

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>(
        "pick_place_mtc",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
    if (!node->get_parameter("use_sim_time").as_bool()) {
        node->set_parameter(rclcpp::Parameter("use_sim_time", true));
    }
    auto logger = rclcpp::get_logger("pick_place_mtc");

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner([&executor]() { executor.spin(); });

    double approach_clearance, lift_height, grasp_tcp_z_offset;
    double place_x, place_y, place_z, place_tcp_z_offset;
    double grasp_close_position, grasp_max_effort, grasp_step;
    int max_picks, grasp_pause_ms, grasp_settle_ms, grasp_contact_timeout_ms;
    node->get_parameter_or("approach_clearance", approach_clearance, 0.12);
    node->get_parameter_or("lift_height", lift_height, 0.12);
    node->get_parameter_or("grasp_tcp_z_offset", grasp_tcp_z_offset, -0.005);
    node->get_parameter_or("place_x", place_x, 0.48);
    node->get_parameter_or("place_y", place_y, 0.35);
    node->get_parameter_or("place_z", place_z, 0.46);
    node->get_parameter_or("place_tcp_z_offset", place_tcp_z_offset, 0.04);
    node->get_parameter_or("grasp_close_position", grasp_close_position, 0.62);
    node->get_parameter_or("grasp_max_effort", grasp_max_effort, 32.0);
    node->get_parameter_or("grasp_step", grasp_step, 0.04);
    node->get_parameter_or("grasp_pause_ms", grasp_pause_ms, 120);
    node->get_parameter_or("grasp_settle_ms", grasp_settle_ms, 700);
    node->get_parameter_or("grasp_contact_timeout_ms", grasp_contact_timeout_ms, 1500);
    node->get_parameter_or("max_picks", max_picks, 3);

    auto pipeline_planner = std::make_shared<solvers::PipelinePlanner>(node);
    pipeline_planner->setPlannerId("RRTConnectkConfigDefault");
    auto joint_planner = std::make_shared<solvers::JointInterpolationPlanner>();
    auto cartesian_planner = std::make_shared<solvers::CartesianPath>();
    cartesian_planner->setStepSize(0.005);
    cartesian_planner->setJumpThreshold(2.0);
    cartesian_planner->setMinFraction(0.90);
    cartesian_planner->setMaxVelocityScalingFactor(0.08);
    cartesian_planner->setMaxAccelerationScalingFactor(0.08);

    MoveGroupInterface move_group(node, kArmGroup);
    move_group.setPlanningTime(10.0);
    move_group.setNumPlanningAttempts(10);
    move_group.setMaxVelocityScalingFactor(0.25);
    move_group.setMaxAccelerationScalingFactor(0.25);

    moveit::planning_interface::PlanningSceneInterface planning_scene;
    apply_static_scene(planning_scene, "world");

    GripperCommander gripper(node);
    if (!gripper.command(0.0)) {
        RCLCPP_ERROR(logger, "failed to open gripper");
        executor.cancel();
        spinner.join();
        rclcpp::shutdown();
        return 1;
    }

    auto detect_client = node->create_client<DetectObjects>("/detect_objects");
    if (!detect_client->wait_for_service(std::chrono::seconds(10))) {
        RCLCPP_ERROR(logger, "Service /detect_objects unavailable");
        executor.cancel();
        spinner.join();
        rclcpp::shutdown();
        return 1;
    }

    std::set<std::string> used_ids;
    for (int pick_index = 0; pick_index < max_picks; ++pick_index) {
        StageList detect_stages;
        detect_stages.push_back(
            move_to_joint_stage("detect elbow up joints", joint_planner, detect_elbow_up_joints()));
        validate_task(node, logger, "goto_detect_camera_pose", std::move(detect_stages));
        if (!goto_joints(
                move_group, logger, "goto_detect_camera_pose", detect_elbow_up_joints())) {
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1200));
        auto request = std::make_shared<DetectObjects::Request>();
        auto future = detect_client->async_send_request(request);
        if (future.wait_for(std::chrono::seconds(3)) != std::future_status::ready) {
            RCLCPP_ERROR(logger, "Detection service timed out");
            break;
        }
        auto response = future.get();
        if (!response->success) {
            RCLCPP_WARN(logger, "Detection failed: %s", response->message.c_str());
            break;
        }
        apply_static_scene(planning_scene, "world");
        apply_detected_boxes(planning_scene, "world", *response);

        auto target = choose_target(*response, used_ids);
        if (!target.has_value()) {
            RCLCPP_WARN(logger, "No unused detections left");
            break;
        }
        used_ids.insert(target->id);
        RCLCPP_INFO(
            logger,
            "Using %s at world_top=(%.3f %.3f %.3f)",
            target->id.c_str(),
            target->world_top_pose.position.x,
            target->world_top_pose.position.y,
            target->world_top_pose.position.z);

        if (!goto_joints(
                move_group, logger, "pre_grasp_elbow_up_seed", detect_elbow_up_joints())) {
            break;
        }

        const double center_z = target->world_top_pose.position.z - kBoxSizeZ / 2.0;
        const auto pre_grasp = tool_down(
            target->world_top_pose.position.x,
            target->world_top_pose.position.y,
            center_z + approach_clearance);
        auto grasp_pose = pre_grasp;
        grasp_pose.position.z = center_z + grasp_tcp_z_offset;

        StageList approach_stages;
        approach_stages.push_back(
            move_to_pose_stage("pre grasp", pipeline_planner, "world", pre_grasp));
        approach_stages.push_back(
            move_to_pose_stage("descend grasp", cartesian_planner, "world", grasp_pose));
        validate_task(node, logger, "approach_and_descend", std::move(approach_stages));
        if (!cartesian_to_pose(move_group, logger, "pre_grasp", pre_grasp)) {
            break;
        }

        planning_scene.removeCollisionObjects({target->id});
        std::this_thread::sleep_for(std::chrono::milliseconds(150));
        if (!cartesian_to_pose(move_group, logger, "descend_grasp", grasp_pose)) {
            break;
        }

        if (!gripper.close_slowly(
                grasp_close_position, grasp_max_effort, grasp_step,
                grasp_pause_ms, grasp_settle_ms, grasp_contact_timeout_ms)) {
            break;
        }
        auto target_center_pose = target->world_top_pose;
        target_center_pose.position.z = center_z;
        planning_scene.applyCollisionObject(make_collision_box(
            target->id, "world", target_center_pose, kBoxSizeX, kBoxSizeY, kBoxSizeZ));
        std::this_thread::sleep_for(std::chrono::milliseconds(150));

        auto attach = std::make_unique<stages::ModifyPlanningScene>("attach object");
        attach->attachObject(target->id, kEndEffectorLink);
        StageList attach_stages;
        attach_stages.push_back(std::move(attach));
        validate_task(node, logger, "attach_object", std::move(attach_stages));
        move_group.attachObject(target->id, kEndEffectorLink, gripper_touch_links());
        std::this_thread::sleep_for(std::chrono::milliseconds(150));

        auto retreat_pose = grasp_pose;
        retreat_pose.position.z += lift_height;
        auto place_pose = tool_down(place_x, place_y, place_z + place_tcp_z_offset);
        auto above_bin_pose = place_pose;
        above_bin_pose.position.z += lift_height;
        auto bin_retreat_pose = place_pose;
        bin_retreat_pose.position.z += lift_height;

        StageList carry_stages;
        carry_stages.push_back(
            move_to_pose_stage("retreat with box", cartesian_planner, "world", retreat_pose));
        carry_stages.push_back(
            move_to_pose_stage("above bin", pipeline_planner, "world", above_bin_pose));
        carry_stages.push_back(
            move_to_pose_stage("descend bin", cartesian_planner, "world", place_pose));
        validate_task(node, logger, "carry_to_bin", std::move(carry_stages));
        if (!cartesian_to_pose(move_group, logger, "retreat_with_box", retreat_pose) ||
            !cartesian_to_pose(move_group, logger, "transport_to_bin", above_bin_pose) ||
            !cartesian_to_pose(move_group, logger, "descend_to_bin", place_pose)) {
            break;
        }

        if (!gripper.command(0.0)) {
            break;
        }
        auto detach = std::make_unique<stages::ModifyPlanningScene>("detach object");
        detach->detachObject(target->id, kEndEffectorLink);
        StageList detach_stages;
        detach_stages.push_back(std::move(detach));
        validate_task(node, logger, "detach_object", std::move(detach_stages));
        move_group.detachObject(target->id);
        std::this_thread::sleep_for(std::chrono::milliseconds(150));
        planning_scene.removeCollisionObjects({target->id});

        StageList retreat_stages;
        retreat_stages.push_back(
            move_to_pose_stage("retreat after place", cartesian_planner, "world", bin_retreat_pose));
        validate_task(node, logger, "retreat_after_place", std::move(retreat_stages));
        if (!cartesian_to_pose(move_group, logger, "retreat_after_place", bin_retreat_pose)) {
            break;
        }
    }

    StageList return_stages;
    return_stages.push_back(move_to_joint_stage("startup", joint_planner, startup_joints()));
    validate_task(node, logger, "return_startup", std::move(return_stages));
    goto_joints(move_group, logger, "return_startup", startup_joints());

    executor.cancel();
    if (spinner.joinable()) {
        spinner.join();
    }
    rclcpp::shutdown();
    return 0;
}
