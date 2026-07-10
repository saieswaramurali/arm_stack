#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <arm_interfaces/srv/detect_objects.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace {

struct ColorThreshold {
    std::string name;
    cv::Scalar low;
    cv::Scalar high;
};

struct TopFaceDetection {
    cv::RotatedRect rect;
    std::vector<cv::Point> contour;
    double depth{std::numeric_limits<double>::quiet_NaN()};
};

struct DetectionResult {
    std::string id;
    geometry_msgs::msg::PoseStamped world_pose;
};

template <typename T>
T get_or_declare_param(
    const rclcpp::Node::SharedPtr& node,
    const std::string& name,
    const T& default_value
) {
    T value{};
    if (node->get_parameter(name, value)) {
        return value;
    }
    return node->declare_parameter<T>(name, default_value);
}

cv::Scalar to_scalar(const std::vector<int64_t>& values) {
    return cv::Scalar(values.at(0), values.at(1), values.at(2));
}

double median_depth(const cv::Mat& depth, int u, int v) {
    std::vector<float> values;
    values.reserve(25);
    for (int y = v - 2; y <= v + 2; ++y) {
        if (y < 0 || y >= depth.rows) {
            continue;
        }
        for (int x = u - 2; x <= u + 2; ++x) {
            if (x < 0 || x >= depth.cols) {
                continue;
            }
            const float d = depth.at<float>(y, x);
            if (std::isfinite(d) && d > 0.0f) {
                values.push_back(d);
            }
        }
    }

    if (values.empty()) {
        return std::numeric_limits<double>::quiet_NaN();
    }

    const auto middle = values.begin() + static_cast<long>(values.size() / 2);
    std::nth_element(values.begin(), middle, values.end());
    return *middle;
}

double median_value(std::vector<float> values) {
    if (values.empty()) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    const auto middle = values.begin() + static_cast<long>(values.size() / 2);
    std::nth_element(values.begin(), middle, values.end());
    return *middle;
}

double normalize_square_yaw(double yaw) {
    return std::remainder(yaw, M_PI / 2.0);
}

}  // namespace

class ObjectDetector {
public:
    explicit ObjectDetector(const rclcpp::Node::SharedPtr& node)
        : node_(node),
          tf_buffer_(node_->get_clock()),
          tf_listener_(tf_buffer_),
          image_sub_(node_.get(), image_topic(), rmw_qos_profile_sensor_data),
          depth_sub_(node_.get(), depth_topic(), rmw_qos_profile_sensor_data),
          info_sub_(node_.get(), camera_info_topic(), rmw_qos_profile_sensor_data),
        sync_(SyncPolicy(10), image_sub_, depth_sub_, info_sub_) {
        pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseArray>(
            get_or_declare_param<std::string>(
                node_, "detected_objects_topic", "/detected_objects"
            ),
            rclcpp::QoS(1)
        );
        debug_image_pub_ = node_->create_publisher<sensor_msgs::msg::Image>(
            get_or_declare_param<std::string>(
                node_, "debug_image_topic", "/object_detector/debug_image"
            ),
            rclcpp::QoS(1)
        );

        target_frame_ = get_or_declare_param<std::string>(node_, "target_frame", "world");
        tcp_frame_ = get_or_declare_param<std::string>(node_, "tcp_frame", "tcp");
        camera_frame_ = get_or_declare_param<std::string>(
            node_, "camera_frame", "wrist_camera_optical_frame"
        );
        min_contour_area_ = get_or_declare_param<double>(node_, "min_contour_area", 300.0);
        min_top_face_area_ = get_or_declare_param<double>(node_, "min_top_face_area", 120.0);
        top_depth_percentile_ = get_or_declare_param<double>(node_, "top_depth_percentile", 0.20);
        top_depth_tolerance_ = get_or_declare_param<double>(node_, "top_depth_tolerance", 0.015);
        box_height_ = get_or_declare_param<double>(node_, "box_height", 0.06);
        horizontal_fov_ = get_or_declare_param<double>(node_, "horizontal_fov", 1.21);
        print_predictions_ = get_or_declare_param<bool>(node_, "print_predictions", true);
        max_detection_age_ = get_or_declare_param<double>(node_, "max_detection_age", 0.75);
        log_static_pick_box_ = get_or_declare_param<bool>(node_, "log_static_pick_box", true);
        static_pick_box_x_ = get_or_declare_param<double>(node_, "static_pick_box.x", 0.0);
        static_pick_box_y_ = get_or_declare_param<double>(node_, "static_pick_box.y", 0.75);
        static_pick_box_z_ = get_or_declare_param<double>(node_, "static_pick_box.z", 0.33);
        static_pick_box_yaw_ = get_or_declare_param<double>(node_, "static_pick_box.yaw", 0.0);
        publish_debug_image_ = get_or_declare_param<bool>(node_, "publish_debug_image", true);
        show_debug_window_ = get_or_declare_param<bool>(node_, "show_debug_window", false);
        use_image_header_frame_ = get_or_declare_param<bool>(node_, "use_image_header_frame", false);
        debug_window_name_ = get_or_declare_param<std::string>(
            node_, "debug_window_name", "object_detector"
        );
        publish_period_ = rclcpp::Duration::from_seconds(
            1.0 / std::max(0.1, get_or_declare_param<double>(node_, "publish_rate", 5.0))
        );

        thresholds_ = {
            make_threshold("red", {0, 80, 40}, {12, 255, 255}),
            make_threshold("green", {35, 60, 35}, {90, 255, 255}),
            make_threshold("blue", {95, 60, 35}, {130, 255, 255}),
        };

        sync_.registerCallback(
            std::bind(
                &ObjectDetector::on_images,
                this,
                std::placeholders::_1,
                std::placeholders::_2,
                std::placeholders::_3
            )
        );
        detect_service_ = node_->create_service<arm_interfaces::srv::DetectObjects>(
            get_or_declare_param<std::string>(node_, "detect_service", "/detect_objects"),
            std::bind(
                &ObjectDetector::on_detect_service,
                this,
                std::placeholders::_1,
                std::placeholders::_2
            )
        );
    }

private:
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image,
        sensor_msgs::msg::Image,
        sensor_msgs::msg::CameraInfo>;

    std::string image_topic() {
        return get_or_declare_param<std::string>(node_, "image_topic", "/wrist_camera/image");
    }

    std::string depth_topic() {
        return get_or_declare_param<std::string>(
            node_, "depth_topic", "/wrist_camera/depth_image"
        );
    }

    std::string camera_info_topic() {
        return get_or_declare_param<std::string>(
            node_, "camera_info_topic", "/wrist_camera/camera_info"
        );
    }

    ColorThreshold make_threshold(
        const std::string& color,
        const std::vector<int64_t>& low_default,
        const std::vector<int64_t>& high_default
    ) {
        return ColorThreshold{
            color,
            to_scalar(get_or_declare_param<std::vector<int64_t>>(
                node_, "colors." + color + ".hsv_low", low_default
            )),
            to_scalar(get_or_declare_param<std::vector<int64_t>>(
                node_, "colors." + color + ".hsv_high", high_default
            )),
        };
    }

    void on_images(
        const sensor_msgs::msg::Image::ConstSharedPtr& image_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg,
        const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info_msg
    ) {
        const auto now = node_->now();
        if (last_publish_time_.nanoseconds() != 0 &&
            now - last_publish_time_ < publish_period_) {
            return;
        }
        last_publish_time_ = now;

        cv_bridge::CvImageConstPtr bgr_image;
        cv_bridge::CvImageConstPtr depth_image;
        try {
            bgr_image = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::BGR8);
            depth_image = cv_bridge::toCvShare(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
        } catch (const cv_bridge::Exception& ex) {
            RCLCPP_WARN_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 2000,
                "cv_bridge conversion failed: %s", ex.what()
            );
            return;
        }

        cv::Mat hsv;
        cv::cvtColor(bgr_image->image, hsv, cv::COLOR_BGR2HSV);
        cv::Mat debug_image = bgr_image->image.clone();

        // Published poses are the detected box top-face centers in world.
        geometry_msgs::msg::PoseArray detected;
        detected.header.frame_id = target_frame_;
        detected.header.stamp = image_msg->header.stamp;
        const std::string source_frame =
            (use_image_header_frame_ && !image_msg->header.frame_id.empty())
                ? image_msg->header.frame_id
                : camera_frame_;

        std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
        std::vector<DetectionResult> detection_results;
        for (const auto& threshold : thresholds_) {
            detect_color(
                threshold,
                hsv,
                depth_image->image,
                *info_msg,
                image_msg->header.stamp,
                source_frame,
                detected,
                collision_objects,
                debug_image,
                detection_results
            );
        }

        pose_pub_->publish(detected);
        publish_debug_image(image_msg->header, debug_image, detected.poses.size());
        if (!collision_objects.empty()) {
            planning_scene_interface_.applyCollisionObjects(collision_objects);
        }
        update_latest_detections(detection_results, image_msg->header.stamp);
        log_static_pick_box_error(detected);
    }

    void detect_color(
        const ColorThreshold& threshold,
        const cv::Mat& hsv,
        const cv::Mat& depth,
        const sensor_msgs::msg::CameraInfo& camera_info,
        const rclcpp::Time& stamp,
        const std::string& source_frame,
        geometry_msgs::msg::PoseArray& detected,
        std::vector<moveit_msgs::msg::CollisionObject>& collision_objects,
        cv::Mat& debug_image,
        std::vector<DetectionResult>& detection_results
    ) {
        cv::Mat mask;
        cv::inRange(hsv, threshold.low, threshold.high, mask);
        if (threshold.name == "red") {
            cv::Mat upper_red_mask;
            cv::inRange(
                hsv,
                cv::Scalar(170, threshold.low[1], threshold.low[2]),
                cv::Scalar(179, threshold.high[1], threshold.high[2]),
                upper_red_mask
            );
            cv::bitwise_or(mask, upper_red_mask, mask);
        }

        const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        int color_index = 0;
        for (const auto& contour : contours) {
            if (cv::contourArea(contour) < min_contour_area_) {
                continue;
            }

            const auto top_face = detect_top_face(contour, depth);
            if (!top_face.has_value()) {
                continue;
            }

            const int u = static_cast<int>(std::round(top_face->rect.center.x));
            const int v = static_cast<int>(std::round(top_face->rect.center.y));
            double d = median_depth(depth, u, v);
            if (!std::isfinite(d)) {
                d = top_face->depth;
            }
            if (!std::isfinite(d)) {
                continue;
            }
            draw_detection_debug(threshold.name, top_face->contour, u, v, d, debug_image);

            geometry_msgs::msg::PoseStamped camera_pose;
            const double yaw = top_face->rect.angle * M_PI / 180.0;
            if (!make_camera_pose(
                    camera_info, stamp, source_frame, u, v, d, yaw, camera_pose)) {
                continue;
            }
            log_camera_prediction(threshold.name, color_index, source_frame, camera_pose.pose);

            geometry_msgs::msg::PoseStamped world_pose;
            if (!transform_pose(camera_pose, target_frame_, world_pose, "camera_to_world")) {
                continue;
            }
            normalize_world_yaw(world_pose.pose);

            detected.poses.push_back(world_pose.pose);
            const std::string id = "box_" + threshold.name + "_" + std::to_string(color_index);
            collision_objects.push_back(make_collision_object(id, world_pose));
            detection_results.push_back(DetectionResult{id, world_pose});
            log_prediction(threshold.name, color_index, world_pose.pose);
            ++color_index;
        }
    }

    std::optional<TopFaceDetection> detect_top_face(
        const std::vector<cv::Point>& contour,
        const cv::Mat& depth
    ) const {
        cv::Mat contour_mask = cv::Mat::zeros(depth.size(), CV_8UC1);
        std::vector<std::vector<cv::Point>> contours{contour};
        cv::drawContours(contour_mask, contours, 0, cv::Scalar(255), cv::FILLED);

        std::vector<float> depths;
        const cv::Rect bounds = cv::boundingRect(contour) & cv::Rect(0, 0, depth.cols, depth.rows);
        for (int y = bounds.y; y < bounds.y + bounds.height; ++y) {
            for (int x = bounds.x; x < bounds.x + bounds.width; ++x) {
                if (contour_mask.at<unsigned char>(y, x) == 0) {
                    continue;
                }
                const float d = depth.at<float>(y, x);
                if (std::isfinite(d) && d > 0.0f) {
                    depths.push_back(d);
                }
            }
        }
        if (depths.empty()) {
            return std::nullopt;
        }

        std::sort(depths.begin(), depths.end());
        const auto clamped_percentile = std::clamp(top_depth_percentile_, 0.0, 1.0);
        const auto percentile_index = static_cast<std::size_t>(
            clamped_percentile * static_cast<double>(depths.size() - 1)
        );
        const float top_depth = depths.at(percentile_index);

        cv::Mat top_mask = cv::Mat::zeros(depth.size(), CV_8UC1);
        for (int y = bounds.y; y < bounds.y + bounds.height; ++y) {
            for (int x = bounds.x; x < bounds.x + bounds.width; ++x) {
                if (contour_mask.at<unsigned char>(y, x) == 0) {
                    continue;
                }
                const float d = depth.at<float>(y, x);
                if (std::isfinite(d) && d > 0.0f && d <= top_depth + top_depth_tolerance_) {
                    top_mask.at<unsigned char>(y, x) = 255;
                }
            }
        }

        const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        cv::morphologyEx(top_mask, top_mask, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(top_mask, top_mask, cv::MORPH_CLOSE, kernel);

        std::vector<std::vector<cv::Point>> top_contours;
        cv::findContours(top_mask, top_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        if (top_contours.empty()) {
            return std::nullopt;
        }

        const auto largest = std::max_element(
            top_contours.begin(),
            top_contours.end(),
            [](const auto& a, const auto& b) {
                return cv::contourArea(a) < cv::contourArea(b);
            }
        );
        if (largest == top_contours.end() || cv::contourArea(*largest) < min_top_face_area_) {
            return std::nullopt;
        }

        std::vector<float> top_depths;
        cv::Mat largest_mask = cv::Mat::zeros(depth.size(), CV_8UC1);
        std::vector<std::vector<cv::Point>> largest_contours{*largest};
        cv::drawContours(largest_mask, largest_contours, 0, cv::Scalar(255), cv::FILLED);
        const cv::Rect top_bounds = cv::boundingRect(*largest) & cv::Rect(0, 0, depth.cols, depth.rows);
        for (int y = top_bounds.y; y < top_bounds.y + top_bounds.height; ++y) {
            for (int x = top_bounds.x; x < top_bounds.x + top_bounds.width; ++x) {
                if (largest_mask.at<unsigned char>(y, x) == 0) {
                    continue;
                }
                const float d = depth.at<float>(y, x);
                if (std::isfinite(d) && d > 0.0f) {
                    top_depths.push_back(d);
                }
            }
        }

        TopFaceDetection result;
        result.rect = cv::minAreaRect(*largest);
        result.contour = *largest;
        result.depth = median_value(top_depths);
        return result;
    }

    cv::Scalar debug_color(const std::string& color) const {
        if (color == "red") {
            return cv::Scalar(0, 0, 255);
        }
        if (color == "green") {
            return cv::Scalar(0, 255, 0);
        }
        if (color == "blue") {
            return cv::Scalar(255, 80, 0);
        }
        return cv::Scalar(255, 255, 255);
    }

    void draw_detection_debug(
        const std::string& color,
        const std::vector<cv::Point>& contour,
        int u,
        int v,
        double depth,
        cv::Mat& debug_image
    ) const {
        const cv::Scalar draw_color = debug_color(color);
        std::vector<std::vector<cv::Point>> contours{contour};
        cv::drawContours(debug_image, contours, 0, draw_color, 2);
        cv::drawMarker(
            debug_image,
            cv::Point(u, v),
            draw_color,
            cv::MARKER_CROSS,
            16,
            2
        );

        char label[64];
        std::snprintf(label, sizeof(label), "%s %.2fm", color.c_str(), depth);
        cv::putText(
            debug_image,
            label,
            cv::Point(std::max(0, u + 8), std::max(18, v - 8)),
            cv::FONT_HERSHEY_SIMPLEX,
            0.45,
            draw_color,
            1,
            cv::LINE_AA
        );
    }

    void publish_debug_image(
        const std_msgs::msg::Header& header,
        const cv::Mat& debug_image,
        std::size_t detection_count
    ) {
        if (!publish_debug_image_ && !show_debug_window_) {
            return;
        }

        cv::Mat output = debug_image.clone();
        const std::string count_label = "detections: " + std::to_string(detection_count);
        cv::putText(
            output,
            count_label,
            cv::Point(12, 24),
            cv::FONT_HERSHEY_SIMPLEX,
            0.65,
            cv::Scalar(255, 255, 255),
            2,
            cv::LINE_AA
        );

        if (publish_debug_image_) {
            cv_bridge::CvImage debug_msg;
            debug_msg.header = header;
            debug_msg.encoding = sensor_msgs::image_encodings::BGR8;
            debug_msg.image = output;
            debug_image_pub_->publish(*debug_msg.toImageMsg());
        }

        if (show_debug_window_) {
            cv::imshow(debug_window_name_, output);
            cv::waitKey(1);
        }
    }

    bool make_camera_pose(
        const sensor_msgs::msg::CameraInfo& camera_info,
        const rclcpp::Time& stamp,
        const std::string& source_frame,
        int u,
        int v,
        double depth,
        double yaw,
        geometry_msgs::msg::PoseStamped& camera_pose
    ) const {
        const double width = camera_info.width > 0 ? camera_info.width : 640.0;
        const double height = camera_info.height > 0 ? camera_info.height : 480.0;
        const double fx = width / (2.0 * std::tan(horizontal_fov_ / 2.0));
        const double fy = fx;
        const double cx = width / 2.0;
        const double cy = height / 2.0;
        if (fx == 0.0 || fy == 0.0) {
            return false;
        }

        camera_pose.header.frame_id = source_frame;
        camera_pose.header.stamp = stamp;
        camera_pose.pose.position.x = (static_cast<double>(u) - cx) * depth / fx;
        camera_pose.pose.position.y = (static_cast<double>(v) - cy) * depth / fy;
        camera_pose.pose.position.z = depth;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw);
        camera_pose.pose.orientation = tf2::toMsg(q);
        return true;
    }

    void normalize_world_yaw(geometry_msgs::msg::Pose& pose) const {
        const double yaw = normalize_square_yaw(tf2::getYaw(pose.orientation));
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw);
        pose.orientation = tf2::toMsg(q);
    }

    moveit_msgs::msg::CollisionObject make_collision_object(
        const std::string& id,
        const geometry_msgs::msg::PoseStamped& top_face_pose
    ) const {
        shape_msgs::msg::SolidPrimitive box;
        box.type = shape_msgs::msg::SolidPrimitive::BOX;
        box.dimensions = {BOX_SIDE, BOX_SIDE, box_height_};

        geometry_msgs::msg::Pose center_pose = top_face_pose.pose;
        center_pose.position.z -= box_height_ / 2.0;

        moveit_msgs::msg::CollisionObject object;
        object.header.frame_id = target_frame_;
        object.header.stamp = top_face_pose.header.stamp;
        object.id = id;
        object.primitives.push_back(box);
        object.primitive_poses.push_back(center_pose);
        object.operation = moveit_msgs::msg::CollisionObject::ADD;
        return object;
    }

    void update_latest_detections(
        const std::vector<DetectionResult>& detections,
        const rclcpp::Time& stamp
    ) {
        geometry_msgs::msg::PoseArray world_poses;
        geometry_msgs::msg::PoseArray tcp_poses;
        world_poses.header.frame_id = target_frame_;
        world_poses.header.stamp = stamp;
        tcp_poses.header.frame_id = tcp_frame_;
        tcp_poses.header.stamp = stamp;

        std::vector<std::string> ids;
        for (const auto& detection : detections) {
            geometry_msgs::msg::PoseStamped tcp_pose;
            if (!transform_pose(detection.world_pose, tcp_frame_, tcp_pose, "world_to_tcp")) {
                continue;
            }
            ids.push_back(detection.id);
            world_poses.poses.push_back(detection.world_pose.pose);
            tcp_poses.poses.push_back(tcp_pose.pose);
        }

        std::lock_guard<std::mutex> lock(latest_mutex_);
        latest_ids_ = ids;
        latest_world_poses_ = world_poses;
        latest_tcp_poses_ = tcp_poses;
        latest_update_time_ = node_->now();
    }

    bool transform_pose(
        const geometry_msgs::msg::PoseStamped& source,
        const std::string& target_frame,
        geometry_msgs::msg::PoseStamped& target,
        const std::string& label
    ) {
        try {
            target = tf_buffer_.transform(source, target_frame, tf2::durationFromSec(0.1));
            return true;
        } catch (const tf2::TransformException& ex) {
            try {
                auto latest_source = source;
                latest_source.header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
                target = tf_buffer_.transform(
                    latest_source, target_frame, tf2::durationFromSec(0.1));
                RCLCPP_WARN_THROTTLE(
                    node_->get_logger(), *node_->get_clock(), 2000,
                    "%s TF used latest transform after timestamp failure from %s to %s: %s",
                    label.c_str(),
                    source.header.frame_id.c_str(),
                    target_frame.c_str(),
                    ex.what());
                return true;
            } catch (const tf2::TransformException& latest_ex) {
                RCLCPP_WARN_THROTTLE(
                    node_->get_logger(), *node_->get_clock(), 2000,
                    "%s TF failed from %s to %s: %s",
                    label.c_str(),
                    source.header.frame_id.c_str(),
                    target_frame.c_str(),
                    latest_ex.what());
                return false;
            }
        }
    }

    void on_detect_service(
        const std::shared_ptr<arm_interfaces::srv::DetectObjects::Request>,
        const std::shared_ptr<arm_interfaces::srv::DetectObjects::Response> response
    ) {
        std::lock_guard<std::mutex> lock(latest_mutex_);
        response->ids = latest_ids_;
        response->world_poses = latest_world_poses_;
        response->tcp_poses = latest_tcp_poses_;
        const double age = latest_update_time_.nanoseconds() == 0
            ? std::numeric_limits<double>::infinity()
            : (node_->now() - latest_update_time_).seconds();
        response->success = !latest_ids_.empty() && age <= max_detection_age_;
        if (response->success) {
            response->message = "returned fresh synchronized detector result age=" +
                std::to_string(age) + "s";
        } else if (latest_ids_.empty()) {
            response->message = "no detections available yet";
        } else {
            response->message = "latest detections are stale age=" +
                std::to_string(age) + "s";
        }
    }

    void log_camera_prediction(
        const std::string& color,
        int index,
        const std::string& frame,
        const geometry_msgs::msg::Pose& pose
    ) const {
        if (!print_predictions_) {
            return;
        }
        RCLCPP_INFO(
            node_->get_logger(),
            "predicted box_%s_%d camera_top[%s]: x=%.3f y=%.3f z=%.3f yaw=%.1f deg",
            color.c_str(),
            index,
            frame.c_str(),
            pose.position.x,
            pose.position.y,
            pose.position.z,
            tf2::getYaw(pose.orientation) * 180.0 / M_PI
        );
    }

    void log_prediction(
        const std::string& color,
        int index,
        const geometry_msgs::msg::Pose& pose
    ) const {
        if (!print_predictions_) {
            return;
        }
        RCLCPP_INFO(
            node_->get_logger(),
            "predicted box_%s_%d top_center: x=%.3f y=%.3f z=%.3f yaw=%.1f deg",
            color.c_str(),
            index,
            pose.position.x,
            pose.position.y,
            pose.position.z,
            tf2::getYaw(pose.orientation) * 180.0 / M_PI
        );
    }

    void log_static_pick_box_error(const geometry_msgs::msg::PoseArray& detected) const {
        if (!log_static_pick_box_ || detected.poses.empty()) {
            return;
        }

        const geometry_msgs::msg::Pose* closest = nullptr;
        double best_xy_error = std::numeric_limits<double>::infinity();
        for (const auto& pose : detected.poses) {
            const double dx = pose.position.x - static_pick_box_x_;
            const double dy = pose.position.y - static_pick_box_y_;
            const double xy_error = std::hypot(dx, dy);
            if (xy_error < best_xy_error) {
                best_xy_error = xy_error;
                closest = &pose;
            }
        }
        if (closest == nullptr) {
            return;
        }

        const double true_top_z = static_pick_box_z_ + box_height_ / 2.0;
        const double dx = closest->position.x - static_pick_box_x_;
        const double dy = closest->position.y - static_pick_box_y_;
        const double dz = closest->position.z - true_top_z;
        const double position_error = std::sqrt(dx * dx + dy * dy + dz * dz);

        const double yaw = tf2::getYaw(closest->orientation);
        double yaw_error = std::remainder(yaw - static_pick_box_yaw_, 2.0 * M_PI);
        yaw_error = std::abs(yaw_error);

        RCLCPP_INFO_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 1000,
            "static pick_box perceived-vs-true: position_error=%.3f m yaw_error=%.1f deg",
            position_error, yaw_error * 180.0 / M_PI
        );
    }

    static constexpr double BOX_SIDE = 0.04;

    rclcpp::Node::SharedPtr node_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

    message_filters::Subscriber<sensor_msgs::msg::Image> image_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
    message_filters::Subscriber<sensor_msgs::msg::CameraInfo> info_sub_;
    message_filters::Synchronizer<SyncPolicy> sync_;

    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;
    rclcpp::Service<arm_interfaces::srv::DetectObjects>::SharedPtr detect_service_;
    std::vector<ColorThreshold> thresholds_;

    std::string target_frame_;
    std::string tcp_frame_;
    std::string camera_frame_;
    double min_contour_area_{300.0};
    double min_top_face_area_{120.0};
    double top_depth_percentile_{0.20};
    double top_depth_tolerance_{0.015};
    double box_height_{0.06};
    double horizontal_fov_{1.21};
    double max_detection_age_{0.75};
    bool print_predictions_{true};
    bool log_static_pick_box_{true};
    bool publish_debug_image_{true};
    bool show_debug_window_{false};
    bool use_image_header_frame_{false};
    std::string debug_window_name_{"object_detector"};
    double static_pick_box_x_{0.0};
    double static_pick_box_y_{0.75};
    double static_pick_box_z_{0.33};
    double static_pick_box_yaw_{0.0};
    rclcpp::Duration publish_period_{rclcpp::Duration::from_seconds(0.2)};
    rclcpp::Time last_publish_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time latest_update_time_{0, 0, RCL_ROS_TIME};
    std::mutex latest_mutex_;
    std::vector<std::string> latest_ids_;
    geometry_msgs::msg::PoseArray latest_world_poses_;
    geometry_msgs::msg::PoseArray latest_tcp_poses_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
        "object_detector",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );
    // The whole stack runs on Gazebo's clock; wall-clock trajectories get
    // aborted by the controller, so default to sim time unless overridden
    if (!node->get_parameter("use_sim_time").as_bool()) {
        node->set_parameter(rclcpp::Parameter("use_sim_time", true));
    }

    auto detector = std::make_shared<ObjectDetector>(node);
    (void)detector;
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
