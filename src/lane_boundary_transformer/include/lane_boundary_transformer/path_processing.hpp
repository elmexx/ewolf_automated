#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <lane_parameter_msg/msg/lane_marking_projected_array_both.hpp>
#include <spline.h>

namespace lane_boundary_transformer {

struct PathProcessingConfig {
  double fusion_alpha{0.3};
  double interp_resolution{0.5};
  int smooth_window{5};
  double lowpass_alpha{0.2};
  int max_missing_frames{3};
  double jump_distance_threshold{1.5};
  double jump_blend_alpha{0.25};
  bool enable_interpolation{true};
  int output_delay_frames{0};
};

struct PathProcessingMetrics {
  bool input_valid{false};
  bool used_history{false};
  bool jump_suppressed{false};
  bool output_delayed{false};
  int missing_frames{0};
  double jump_metric{0.0};
  std::size_t raw_point_count{0};
  std::size_t output_point_count{0};
  std::string status{"empty"};
};

class LanePathProcessor {
 public:
  explicit LanePathProcessor(PathProcessingConfig config) : config_(config) {}

  std::vector<geometry_msgs::msg::PoseStamped> Process(
      const lane_parameter_msg::msg::LaneMarkingProjectedArrayBoth &current_markings,
      PathProcessingMetrics *metrics) {
    if (metrics == nullptr) {
      return {};
    }

    *metrics = PathProcessingMetrics{};

    lane_parameter_msg::msg::LaneMarkingProjectedArrayBoth working_markings = current_markings;
    metrics->input_valid = HasValidInput(working_markings);

    if (metrics->input_valid) {
      missing_frames_ = 0;
      FuseWithHistory(working_markings);
      last_valid_markings_ = working_markings;
      metrics->status = "fresh_input";
    } else {
      ++missing_frames_;
      metrics->missing_frames = missing_frames_;
      if (missing_frames_ <= config_.max_missing_frames && HasValidInput(last_valid_markings_)) {
        working_markings = last_valid_markings_;
        metrics->used_history = true;
        metrics->status = "hold_last_valid";
      } else {
        delay_queue_.clear();
        metrics->status = "missing_timeout";
        return {};
      }
    }

    auto raw_path = GenerateReferencePath(working_markings);
    metrics->raw_point_count = raw_path.size();
    if (raw_path.empty()) {
      metrics->status = "raw_path_empty";
      return {};
    }

    auto interpolated_path = config_.enable_interpolation ? InterpolatePath(raw_path) : raw_path;
    auto smoothed_path = SmoothPath(interpolated_path);
    auto filtered_path = LowpassFilter(smoothed_path);
    auto jump_handled_path = SuppressJump(filtered_path, metrics);
    auto output_path = ApplyDelay(jump_handled_path, metrics);

    metrics->missing_frames = missing_frames_;
    metrics->output_point_count = output_path.size();
    if (!metrics->used_history && !metrics->jump_suppressed && metrics->status == "fresh_input") {
      metrics->status = "fresh_processed";
    }
    if (metrics->jump_suppressed) {
      metrics->status = metrics->used_history ? "hold_with_jump_suppression" : "jump_suppressed";
    }

    if (!output_path.empty()) {
      last_output_path_ = output_path;
    }
    return output_path;
  }

  static std::vector<geometry_msgs::msg::PoseStamped> GenerateReferencePath(
      const lane_parameter_msg::msg::LaneMarkingProjectedArrayBoth &markings) {
    std::vector<geometry_msgs::msg::PoseStamped> path;
    const std::size_t min_size = std::min(markings.markings_left.size(), markings.markings_right.size());
    path.reserve(min_size);

    for (std::size_t i = 0; i < min_size; ++i) {
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position.x = (markings.markings_left[i].x + markings.markings_right[i].x) / 2.0;
      pose.pose.position.y = (markings.markings_left[i].y + markings.markings_right[i].y) / 2.0;
      pose.pose.orientation.w = 1.0;
      path.push_back(pose);
    }
    return path;
  }

  std::vector<geometry_msgs::msg::PoseStamped> InterpolatePath(
      const std::vector<geometry_msgs::msg::PoseStamped> &raw_path) const {
    if (raw_path.size() < 3 || config_.interp_resolution <= 0.0) {
      return raw_path;
    }

    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> s;
    x.reserve(raw_path.size());
    y.reserve(raw_path.size());
    s.reserve(raw_path.size());

    double total_s = 0.0;
    for (std::size_t i = 0; i < raw_path.size(); ++i) {
      x.push_back(raw_path[i].pose.position.x);
      y.push_back(raw_path[i].pose.position.y);
      if (i > 0) {
        const double dx = x[i] - x[i - 1];
        const double dy = y[i] - y[i - 1];
        total_s += std::hypot(dx, dy);
      }
      s.push_back(total_s);
    }

    tk::spline spline_x;
    tk::spline spline_y;
    spline_x.set_points(s, x);
    spline_y.set_points(s, y);

    std::vector<geometry_msgs::msg::PoseStamped> interpolated_path;
    for (double s_i = 0.0; s_i <= total_s; s_i += config_.interp_resolution) {
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position.x = spline_x(s_i);
      pose.pose.position.y = spline_y(s_i);
      pose.pose.orientation.w = 1.0;
      interpolated_path.push_back(pose);
    }
    if (interpolated_path.empty()) {
      return raw_path;
    }
    return interpolated_path;
  }

  std::vector<geometry_msgs::msg::PoseStamped> SmoothPath(
      const std::vector<geometry_msgs::msg::PoseStamped> &path) const {
    std::vector<geometry_msgs::msg::PoseStamped> smoothed;
    const int n = static_cast<int>(path.size());
    if (n == 0) {
      return smoothed;
    }

    smoothed.reserve(path.size());
    for (int i = 0; i < n; ++i) {
      double sum_x = 0.0;
      double sum_y = 0.0;
      int count = 0;
      for (int j = std::max(0, i - config_.smooth_window);
           j <= std::min(n - 1, i + config_.smooth_window); ++j) {
        sum_x += path[j].pose.position.x;
        sum_y += path[j].pose.position.y;
        ++count;
      }
      auto pose = path[i];
      pose.pose.position.x = sum_x / static_cast<double>(count);
      pose.pose.position.y = sum_y / static_cast<double>(count);
      smoothed.push_back(pose);
    }
    return smoothed;
  }

  std::vector<geometry_msgs::msg::PoseStamped> LowpassFilter(
      const std::vector<geometry_msgs::msg::PoseStamped> &path) const {
    if (path.empty()) {
      return path;
    }

    std::vector<geometry_msgs::msg::PoseStamped> filtered = path;
    for (std::size_t i = 1; i < path.size(); ++i) {
      filtered[i].pose.position.x =
          config_.lowpass_alpha * path[i].pose.position.x +
          (1.0 - config_.lowpass_alpha) * filtered[i - 1].pose.position.x;
      filtered[i].pose.position.y =
          config_.lowpass_alpha * path[i].pose.position.y +
          (1.0 - config_.lowpass_alpha) * filtered[i - 1].pose.position.y;
    }
    return filtered;
  }

 private:
  bool HasValidInput(const lane_parameter_msg::msg::LaneMarkingProjectedArrayBoth &markings) const {
    return !markings.markings_left.empty() && !markings.markings_right.empty();
  }

  void FuseWithHistory(lane_parameter_msg::msg::LaneMarkingProjectedArrayBoth &current) {
    if (!HasValidInput(last_valid_markings_)) {
      return;
    }

    const std::size_t min_size = std::min(last_valid_markings_.markings_left.size(), current.markings_left.size());
    for (std::size_t i = 0; i < min_size; ++i) {
      auto &last_left = last_valid_markings_.markings_left[i];
      auto &curr_left = current.markings_left[i];
      curr_left.x = config_.fusion_alpha * curr_left.x + (1.0 - config_.fusion_alpha) * last_left.x;
      curr_left.y = config_.fusion_alpha * curr_left.y + (1.0 - config_.fusion_alpha) * last_left.y;

      auto &last_right = last_valid_markings_.markings_right[i];
      auto &curr_right = current.markings_right[i];
      curr_right.x = config_.fusion_alpha * curr_right.x + (1.0 - config_.fusion_alpha) * last_right.x;
      curr_right.y = config_.fusion_alpha * curr_right.y + (1.0 - config_.fusion_alpha) * last_right.y;
    }
  }

  std::vector<geometry_msgs::msg::PoseStamped> SuppressJump(
      const std::vector<geometry_msgs::msg::PoseStamped> &candidate_path,
      PathProcessingMetrics *metrics) const {
    if (last_output_path_.empty() || candidate_path.empty()) {
      metrics->jump_metric = 0.0;
      return candidate_path;
    }

    const std::size_t min_size = std::min(last_output_path_.size(), candidate_path.size());
    if (min_size == 0) {
      metrics->jump_metric = 0.0;
      return candidate_path;
    }

    double distance_sum = 0.0;
    for (std::size_t i = 0; i < min_size; ++i) {
      const double dx = candidate_path[i].pose.position.x - last_output_path_[i].pose.position.x;
      const double dy = candidate_path[i].pose.position.y - last_output_path_[i].pose.position.y;
      distance_sum += std::hypot(dx, dy);
    }
    metrics->jump_metric = distance_sum / static_cast<double>(min_size);

    if (metrics->jump_metric <= config_.jump_distance_threshold) {
      return candidate_path;
    }

    metrics->jump_suppressed = true;
    std::vector<geometry_msgs::msg::PoseStamped> blended = candidate_path;
    for (std::size_t i = 0; i < min_size; ++i) {
      blended[i].pose.position.x =
          config_.jump_blend_alpha * candidate_path[i].pose.position.x +
          (1.0 - config_.jump_blend_alpha) * last_output_path_[i].pose.position.x;
      blended[i].pose.position.y =
          config_.jump_blend_alpha * candidate_path[i].pose.position.y +
          (1.0 - config_.jump_blend_alpha) * last_output_path_[i].pose.position.y;
    }
    return blended;
  }

  std::vector<geometry_msgs::msg::PoseStamped> ApplyDelay(
      const std::vector<geometry_msgs::msg::PoseStamped> &path,
      PathProcessingMetrics *metrics) {
    if (config_.output_delay_frames <= 0) {
      return path;
    }

    delay_queue_.push_back(path);
    if (static_cast<int>(delay_queue_.size()) <= config_.output_delay_frames) {
      metrics->output_delayed = false;
      return path;
    }

    auto delayed_path = delay_queue_.front();
    delay_queue_.pop_front();
    metrics->output_delayed = true;
    return delayed_path;
  }

  PathProcessingConfig config_;
  lane_parameter_msg::msg::LaneMarkingProjectedArrayBoth last_valid_markings_;
  std::vector<geometry_msgs::msg::PoseStamped> last_output_path_;
  std::deque<std::vector<geometry_msgs::msg::PoseStamped>> delay_queue_;
  int missing_frames_{0};
};

}  // namespace lane_boundary_transformer
