#include <rclcpp/rclcpp.hpp>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "lane_boundary_transformer/path_processing.hpp"
#include "lane_parameter_msg/msg/lane_marking_projected.hpp"
#include "lane_parameter_msg/msg/lane_marking_projected_array_both.hpp"

class LaneTransformerNode : public rclcpp::Node {
 public:
  LaneTransformerNode() : Node("lane_transformer_node") {
    declare_parameter("fusion_alpha", 0.3);
    declare_parameter("interp_resolution", 0.5);
    declare_parameter("smooth_window", 5);
    declare_parameter("lowpass_alpha", 0.2);
    declare_parameter("max_missing_frames", 3);
    declare_parameter("jump_distance_threshold", 1.5);
    declare_parameter("jump_blend_alpha", 0.25);
    declare_parameter("enable_interpolation", true);
    declare_parameter("output_delay_frames", 0);
    declare_parameter("enable_debug_topics", true);
    declare_parameter("throttled_log_interval_frames", 20);

    lane_boundary_transformer::PathProcessingConfig config;
    config.fusion_alpha = get_parameter("fusion_alpha").as_double();
    config.interp_resolution = get_parameter("interp_resolution").as_double();
    config.smooth_window = get_parameter("smooth_window").as_int();
    config.lowpass_alpha = get_parameter("lowpass_alpha").as_double();
    config.max_missing_frames = get_parameter("max_missing_frames").as_int();
    config.jump_distance_threshold = get_parameter("jump_distance_threshold").as_double();
    config.jump_blend_alpha = get_parameter("jump_blend_alpha").as_double();
    config.enable_interpolation = get_parameter("enable_interpolation").as_bool();
    config.output_delay_frames = get_parameter("output_delay_frames").as_int();

    enable_debug_topics_ = get_parameter("enable_debug_topics").as_bool();
    throttled_log_interval_frames_ = get_parameter("throttled_log_interval_frames").as_int();
    path_processor_ = std::make_unique<lane_boundary_transformer::LanePathProcessor>(config);

    lane_markings_sub_ = create_subscription<lane_parameter_msg::msg::LaneMarkingProjectedArrayBoth>(
        "/detection/lane/lane_markings_projected", 10,
        std::bind(&LaneTransformerNode::laneMarkingsCallback, this, std::placeholders::_1));

    path_pub_ = create_publisher<nav_msgs::msg::Path>("/reference_path", 10);
    raw_path_pub_ = create_publisher<nav_msgs::msg::Path>("/reference_path/debug/raw", 10);
    processed_path_pub_ = create_publisher<nav_msgs::msg::Path>("/reference_path/debug/processed", 10);
    metrics_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/reference_path/debug/metrics", 10);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

 private:
  void laneMarkingsCallback(const lane_parameter_msg::msg::LaneMarkingProjectedArrayBoth::SharedPtr msg) {
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_->lookupTransform("odom", "base_link", tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "Failed to get transform: %s", ex.what());
      return;
    }

    auto transformed_msg = transformLaneMarkings(*msg, transform);
    auto raw_path = lane_boundary_transformer::LanePathProcessor::GenerateReferencePath(transformed_msg);

    lane_boundary_transformer::PathProcessingMetrics metrics;
    auto processed_path = path_processor_->Process(transformed_msg, &metrics);

    if (enable_debug_topics_) {
      publishPath(raw_path_pub_, raw_path, msg->header.stamp);
      publishPath(processed_path_pub_, processed_path, msg->header.stamp);
      publishMetrics(metrics, msg->header.stamp);
    }

    if (processed_path.empty()) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Reference path empty. status=%s missing_frames=%d jump_metric=%.3f", metrics.status.c_str(),
          metrics.missing_frames, metrics.jump_metric);
      return;
    }

    publishPath(path_pub_, processed_path, msg->header.stamp);

    ++frame_counter_;
    if (throttled_log_interval_frames_ > 0 && frame_counter_ % throttled_log_interval_frames_ == 0) {
      RCLCPP_INFO(
          get_logger(),
          "reference_path status=%s raw_pts=%zu output_pts=%zu missing_frames=%d used_history=%d jump_suppressed=%d jump_metric=%.3f delayed=%d",
          metrics.status.c_str(), metrics.raw_point_count, metrics.output_point_count, metrics.missing_frames,
          metrics.used_history ? 1 : 0, metrics.jump_suppressed ? 1 : 0, metrics.jump_metric,
          metrics.output_delayed ? 1 : 0);
    }
  }

  lane_parameter_msg::msg::LaneMarkingProjectedArrayBoth transformLaneMarkings(
      const lane_parameter_msg::msg::LaneMarkingProjectedArrayBoth &msg,
      const geometry_msgs::msg::TransformStamped &transform) {
    lane_parameter_msg::msg::LaneMarkingProjectedArrayBoth transformed;
    transformed.header.stamp = get_clock()->now();
    transformed.header.frame_id = "odom";

    const tf2::Quaternion q(
        transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z,
        transform.transform.rotation.w);
    const tf2::Vector3 translation(
        transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z);
    const tf2::Transform tf_transform(q, translation);

    auto transform_point = [&](const auto &point) {
      lane_parameter_msg::msg::LaneMarkingProjected p;
      const tf2::Vector3 vec(point.x, point.y, point.z);
      const tf2::Vector3 transformed_vec = tf_transform * vec;
      p.x = transformed_vec.x();
      p.y = transformed_vec.y();
      p.z = transformed_vec.z();
      return p;
    };

    for (const auto &p : msg.markings_left) {
      transformed.markings_left.push_back(transform_point(p));
    }
    for (const auto &p : msg.markings_right) {
      transformed.markings_right.push_back(transform_point(p));
    }

    return transformed;
  }

  void publishPath(const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr &publisher,
                   const std::vector<geometry_msgs::msg::PoseStamped> &path,
                   const builtin_interfaces::msg::Time &stamp) {
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = stamp;
    path_msg.header.frame_id = "odom";
    path_msg.poses = path;
    publisher->publish(path_msg);
  }

  void publishMetrics(const lane_boundary_transformer::PathProcessingMetrics &metrics,
                      const builtin_interfaces::msg::Time &stamp) {
    (void)stamp;
    std_msgs::msg::Float64MultiArray msg;
    msg.data = {
        static_cast<double>(metrics.input_valid ? 1 : 0),
        static_cast<double>(metrics.used_history ? 1 : 0),
        static_cast<double>(metrics.jump_suppressed ? 1 : 0),
        static_cast<double>(metrics.output_delayed ? 1 : 0),
        static_cast<double>(metrics.missing_frames),
        metrics.jump_metric,
        static_cast<double>(metrics.raw_point_count),
        static_cast<double>(metrics.output_point_count),
    };
    metrics_pub_->publish(msg);
  }

  rclcpp::Subscription<lane_parameter_msg::msg::LaneMarkingProjectedArrayBoth>::SharedPtr lane_markings_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr raw_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr processed_path_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr metrics_pub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<lane_boundary_transformer::LanePathProcessor> path_processor_;
  bool enable_debug_topics_{true};
  int throttled_log_interval_frames_{20};
  int frame_counter_{0};
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaneTransformerNode>());
  rclcpp::shutdown();
  return 0;
}
