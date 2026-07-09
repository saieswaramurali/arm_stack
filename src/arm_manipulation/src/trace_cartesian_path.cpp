#include <cmath>
#include <memory>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace {
constexpr double kPi = 3.14159265358979323846;

std::vector<geometry_msgs::msg::Pose> make_heart_waypoints(
    geometry_msgs::msg::Pose pose, double center_x, double center_y, double center_z,
    double scale, int samples, double plane_yaw_deg) {
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.reserve(samples + 1);

    // The heart's width axis is rotated about world Z by plane_yaw_deg:
    // 0 deg keeps the old Y-Z plane, 90 deg puts the heart in the X-Z plane
    double const yaw = plane_yaw_deg * kPi / 180.0;

    for (int i = 0; i <= samples; ++i) {
        double const t = 2.0 * kPi * static_cast<double>(i) / static_cast<double>(samples);
        double const heart_x = 16.0 * std::pow(std::sin(t), 3.0);
        double const heart_y =
            13.0 * std::cos(t) - 5.0 * std::cos(2.0 * t) -
            2.0 * std::cos(3.0 * t) - std::cos(4.0 * t);

        pose.position.x = center_x + scale * heart_x * std::sin(yaw);
        pose.position.y = center_y + scale * heart_x * std::cos(yaw);
        pose.position.z = center_z + scale * heart_y;
        waypoints.push_back(pose);
    }

    return waypoints;
}

visualization_msgs::msg::Marker make_heart_marker(
    std::vector<geometry_msgs::msg::Pose> const& waypoints,
    std::string const& frame_id, int id, int type) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = rclcpp::Clock().now();
    marker.ns = "heart_cartesian_path";
    marker.id = id;
    marker.type = type;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = (type == visualization_msgs::msg::Marker::LINE_STRIP) ? 0.008 : 0.018;
    marker.scale.y = 0.018;
    marker.scale.z = 0.018;
    marker.color.r = 1.0;
    marker.color.g = (type == visualization_msgs::msg::Marker::LINE_STRIP) ? 0.05 : 0.35;
    marker.color.b = (type == visualization_msgs::msg::Marker::LINE_STRIP) ? 0.12 : 0.45;
    marker.color.a = 1.0;
    marker.lifetime = rclcpp::Duration::from_seconds(0.0);

    marker.points.reserve(waypoints.size());
    for (auto const& waypoint : waypoints) {
        geometry_msgs::msg::Point point;
        point.x = waypoint.position.x;
        point.y = waypoint.position.y;
        point.z = waypoint.position.z;
        marker.points.push_back(point);
    }

    return marker;
}
}  // namespace

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
        "trace_cartesian_path",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );
    // The whole stack runs on Gazebo's clock; wall-clock trajectories get
    // aborted by the controller, so default to sim time unless overridden
    if (!node->get_parameter("use_sim_time").as_bool()) {
        node->set_parameter(rclcpp::Parameter("use_sim_time", true));
    }
    auto const logger = rclcpp::get_logger("trace_cartesian_path");
    auto marker_pub = node->create_publisher<visualization_msgs::msg::Marker>(
        "heart_cartesian_path", rclcpp::QoS(1).transient_local()
    );

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner([&executor]() { executor.spin(); });

    using moveit::planning_interface::MoveGroupInterface;
    MoveGroupInterface move_group(node, "ur_manipulator");
    move_group.setPlanningTime(10.0);
    move_group.setMaxVelocityScalingFactor(0.15);
    move_group.setMaxAccelerationScalingFactor(0.15);

    move_group.setEndEffectorLink("tcp");
    geometry_msgs::msg::Pose base_pose = move_group.getCurrentPose("tcp").pose;

    // Params may already be auto-declared from overrides, so don't re-declare
    double center_x, center_y, center_z, scale;
    int samples;
    bool execute_path;
    node->get_parameter_or("center_x", center_x, 0.0);
    node->get_parameter_or("center_y", center_y, 0.45);
    node->get_parameter_or("center_z", center_z, 0.6);
    node->get_parameter_or("scale", scale, 0.02);
    node->get_parameter_or("samples", samples, 120);
    node->get_parameter_or("execute_path", execute_path, true);
    double plane_yaw_deg;
    node->get_parameter_or("plane_yaw_deg", plane_yaw_deg, 90.0);

    // Default: point the tool into the heart plane (tool Z along world +Y)
    // instead of whatever pose the arm happens to be in; a vertical-up tool
    // orientation makes most waypoints IK-infeasible
    bool use_current_orientation;
    node->get_parameter_or("use_current_orientation", use_current_orientation, false);
    if (!use_current_orientation) {
        base_pose.orientation.x = -0.7071068;  // rotate -90 deg about X: Z -> +Y
        base_pose.orientation.y = 0.0;
        base_pose.orientation.z = 0.0;
        base_pose.orientation.w = 0.7071068;
    }

    auto waypoints = make_heart_waypoints(
        base_pose, center_x, center_y, center_z, scale, samples, plane_yaw_deg);
    auto const frame_id = move_group.getPlanningFrame();

    marker_pub->publish(make_heart_marker(
        waypoints, frame_id, 0, visualization_msgs::msg::Marker::LINE_STRIP
    ));
    marker_pub->publish(make_heart_marker(
        waypoints, frame_id, 1, visualization_msgs::msg::Marker::SPHERE_LIST
    ));
    RCLCPP_INFO(
        logger,
        "Published heart path marker on /heart_cartesian_path in frame '%s'",
        frame_id.c_str()
    );

    if (!execute_path) {
        RCLCPP_INFO(logger,
            "execute_path is false; keeping node alive so the marker stays visible (Ctrl+C to quit)");
        spinner.join();  // marker is transient_local; it lives as long as this node does
        rclcpp::shutdown();
        return 0;
    }

    move_group.setPoseTarget(waypoints.front());
    moveit::planning_interface::MoveGroupInterface::Plan start_plan;
    if (!static_cast<bool>(move_group.plan(start_plan))) {
        RCLCPP_ERROR(logger, "Could not plan to the first heart waypoint");
        executor.cancel();
        spinner.join();
        rclcpp::shutdown();
        return 1;
    }
    move_group.execute(start_plan);

    moveit_msgs::msg::RobotTrajectory trajectory;
    double const fraction = move_group.computeCartesianPath(
        waypoints, 0.005, 0.0, trajectory
    );

    RCLCPP_INFO(logger, "Cartesian heart path coverage: %.1f%%", fraction * 100.0);
    if (fraction < 0.95) {
        RCLCPP_ERROR(logger, "Heart path coverage too low; refusing to execute");
        executor.cancel();
        spinner.join();
        rclcpp::shutdown();
        return 1;
    }

    // computeCartesianPath ignores the velocity/acceleration scaling factors,
    // so retime the trajectory or the trace runs at full joint speed
    robot_trajectory::RobotTrajectory retimed(move_group.getRobotModel(), "ur_manipulator");
    retimed.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory);
    trajectory_processing::TimeOptimalTrajectoryGeneration totg;
    if (totg.computeTimeStamps(retimed, 0.15, 0.15)) {
        retimed.getRobotTrajectoryMsg(trajectory);
    } else {
        RCLCPP_WARN(logger, "Retiming failed; executing with raw timing");
    }

    move_group.execute(trajectory);
    RCLCPP_INFO(logger, "Finished tracing heart path");

    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 0;
}
