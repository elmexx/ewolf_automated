#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

namespace
{
constexpr double kDegToRad = M_PI / 180.0;

struct RadarPoint
{
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
  float doppler{std::numeric_limits<float>::quiet_NaN()};
  float range{0.0F};
  float azimuth{0.0F};
  float azimuth_std{0.0F};
  bool is_infra{false};
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
};

struct FieldOffsets
{
  int x{-1};
  int y{-1};
  int z{-1};
  int doppler{-1};
  int range{-1};
  int azimuth{-1};
  int azimuth_std{-1};
  uint32_t point_step{0};
};

struct CandidateTarget
{
  RadarPoint point;
  int stable_hits{0};
  int nearby_count{0};
  double lateral_motion_score{0.0};
};

bool isFinite(const float value)
{
  return std::isfinite(value);
}

float readFloat32(const sensor_msgs::msg::PointCloud2 & msg, const size_t point_index, const int offset)
{
  if (offset < 0) {
    return std::numeric_limits<float>::quiet_NaN();
  }

  const size_t base = point_index * msg.point_step + static_cast<size_t>(offset);
  if (base + sizeof(float) > msg.data.size()) {
    return std::numeric_limits<float>::quiet_NaN();
  }

  float value;
  std::memcpy(&value, &msg.data[base], sizeof(float));
  return value;
}

FieldOffsets parseFieldOffsets(const sensor_msgs::msg::PointCloud2 & msg)
{
  FieldOffsets offsets;
  offsets.point_step = msg.point_step;

  for (const auto & field : msg.fields) {
    if (field.datatype != sensor_msgs::msg::PointField::FLOAT32) {
      continue;
    }

    if (field.name == "x") {
      offsets.x = static_cast<int>(field.offset);
    } else if (field.name == "y") {
      offsets.y = static_cast<int>(field.offset);
    } else if (field.name == "z") {
      offsets.z = static_cast<int>(field.offset);
    } else if (field.name == "doppler") {
      offsets.doppler = static_cast<int>(field.offset);
    } else if (field.name == "range") {
      offsets.range = static_cast<int>(field.offset);
    } else if (field.name == "azimuth") {
      offsets.azimuth = static_cast<int>(field.offset);
    } else if (field.name == "azimuth_std") {
      offsets.azimuth_std = static_cast<int>(field.offset);
    }
  }

  return offsets;
}

std::int64_t makeGridKey(const int gx, const int gy)
{
  return (static_cast<std::int64_t>(gx) << 32) ^ static_cast<std::uint32_t>(gy);
}
}  // namespace

class RadarBevVisualizer : public rclcpp::Node
{
public:
  RadarBevVisualizer()
  : Node("radar_bev_visualizer")
  {
    infra_topic_ = this->declare_parameter<std::string>("infra_topic", "/radar_detection_pcl_infra");
    non_infra_topic_ = this->declare_parameter<std::string>("non_infra_topic", "/radar_detection_pcl_non_infra");
    output_image_topic_ = this->declare_parameter<std::string>("output_image_topic", "/radar_bev_image");
    frame_id_ = this->declare_parameter<std::string>("frame_id", "radar_bev");

    image_width_ = this->declare_parameter<int>("image_width", 1280);
    image_height_ = this->declare_parameter<int>("image_height", 720);

    max_range_m_ = this->declare_parameter<double>("max_range_m", 10.0);
    min_range_m_ = this->declare_parameter<double>("min_range_m", 0.8);
    max_abs_azimuth_deg_ = this->declare_parameter<double>("max_abs_azimuth_deg", 25.0);
    min_z_m_ = this->declare_parameter<double>("min_z_m", -2.0);
    max_z_m_ = this->declare_parameter<double>("max_z_m", 2.0);
    max_azimuth_std_deg_ = this->declare_parameter<double>("max_azimuth_std_deg", 6.0);
    static_doppler_threshold_mps_ = this->declare_parameter<double>("static_doppler_threshold_mps", 0.2);
    history_window_sec_ = this->declare_parameter<double>("history_window_sec", 0.45);
    publish_rate_hz_ = this->declare_parameter<double>("publish_rate_hz", 15.0);
    track_grid_m_ = this->declare_parameter<double>("track_grid_m", 0.35);
    stable_track_min_hits_ = this->declare_parameter<int>("stable_track_min_hits", 3);
    lateral_motion_min_shift_m_ = this->declare_parameter<double>("lateral_motion_min_shift_m", 0.25);
    label_top_k_ = this->declare_parameter<int>("label_top_k", 3);
    dynamic_label_top_k_ = this->declare_parameter<int>("dynamic_label_top_k", 1);

    show_infra_ = this->declare_parameter<bool>("show_infra", true);
    show_non_infra_ = this->declare_parameter<bool>("show_non_infra", true);
    show_grid_ = this->declare_parameter<bool>("show_grid", true);
    show_labels_ = this->declare_parameter<bool>("show_labels", true);
    show_history_ = this->declare_parameter<bool>("show_history", true);
    show_status_overlay_ = this->declare_parameter<bool>("show_status_overlay", true);
    emphasize_non_infra_ = this->declare_parameter<bool>("emphasize_non_infra", true);
    show_target_rings_ = this->declare_parameter<bool>("show_target_rings", true);
    emphasize_lateral_motion_ = this->declare_parameter<bool>("emphasize_lateral_motion", true);

    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(output_image_topic_, 10);
    infra_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      infra_topic_, 10,
      std::bind(&RadarBevVisualizer::onInfraCloud, this, std::placeholders::_1));
    non_infra_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      non_infra_topic_, 10,
      std::bind(&RadarBevVisualizer::onNonInfraCloud, this, std::placeholders::_1));

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_hz_));
    render_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&RadarBevVisualizer::renderAndPublish, this));

    RCLCPP_INFO(this->get_logger(), "radar_bev_visualizer started. infra=%s non_infra=%s out=%s",
      infra_topic_.c_str(), non_infra_topic_.c_str(), output_image_topic_.c_str());
  }

private:
  void onInfraCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    last_infra_rx_time_ = this->now();
    last_infra_points_count_ = static_cast<int>(msg->width * msg->height);
    appendPoints(*msg, true);
  }

  void onNonInfraCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    last_non_infra_rx_time_ = this->now();
    last_non_infra_points_count_ = static_cast<int>(msg->width * msg->height);
    appendPoints(*msg, false);
  }

  void appendPoints(const sensor_msgs::msg::PointCloud2 & msg, const bool is_infra)
  {
    const auto offsets = parseFieldOffsets(msg);
    const size_t point_count = static_cast<size_t>(msg.width) * static_cast<size_t>(msg.height);

    const auto stamp = msg.header.stamp.sec == 0 && msg.header.stamp.nanosec == 0 ? this->now() :
      rclcpp::Time(msg.header.stamp);

    for (size_t i = 0; i < point_count; ++i) {
      RadarPoint p;
      p.x = readFloat32(msg, i, offsets.x);
      p.y = readFloat32(msg, i, offsets.y);
      p.z = readFloat32(msg, i, offsets.z);
      p.doppler = readFloat32(msg, i, offsets.doppler);
      p.range = readFloat32(msg, i, offsets.range);
      p.azimuth = readFloat32(msg, i, offsets.azimuth);
      p.azimuth_std = readFloat32(msg, i, offsets.azimuth_std);
      p.is_infra = is_infra;
      p.stamp = stamp;

      if (!isFinite(p.x) || !isFinite(p.y)) {
        continue;
      }
      if (!passesFilter(p)) {
        continue;
      }

      history_.push_back(p);
    }

    pruneHistory();
  }

  bool passesFilter(const RadarPoint & p) const
  {
    if (!isFinite(p.range)) {
      const double inferred_range = std::sqrt(static_cast<double>(p.x) * p.x + static_cast<double>(p.y) * p.y);
      if (inferred_range < min_range_m_ || inferred_range > max_range_m_) {
        return false;
      }
    } else {
      if (p.range < min_range_m_ || p.range > max_range_m_) {
        return false;
      }
    }

    double azimuth_deg = p.azimuth;
    if (!isFinite(p.azimuth)) {
      azimuth_deg = std::atan2(static_cast<double>(p.y), static_cast<double>(p.x)) / kDegToRad;
    }
    if (std::abs(azimuth_deg) > max_abs_azimuth_deg_) {
      return false;
    }

    if (isFinite(p.z) && (p.z < min_z_m_ || p.z > max_z_m_)) {
      return false;
    }

    if (isFinite(p.azimuth_std) && p.azimuth_std > max_azimuth_std_deg_) {
      return false;
    }

    if (p.is_infra && !show_infra_) {
      return false;
    }
    if (!p.is_infra && !show_non_infra_) {
      return false;
    }

    return true;
  }

  void pruneHistory()
  {
    const rclcpp::Time now = this->now();
    const rclcpp::Duration max_age = rclcpp::Duration::from_seconds(std::max(0.0, history_window_sec_));

    history_.erase(
      std::remove_if(
        history_.begin(), history_.end(),
        [&](const RadarPoint & p) {
          return (now - p.stamp) > max_age;
        }),
      history_.end());
  }

  cv::Point2i projectToCanvas(const RadarPoint & p) const
  {
    const double width_m = 2.0 * max_range_m_ * std::sin(max_abs_azimuth_deg_ * kDegToRad);
    const double px_per_meter_x = static_cast<double>(image_width_) / std::max(0.1, width_m);
    const double px_per_meter_y = static_cast<double>(image_height_ - top_margin_ - bottom_margin_) / std::max(0.1, max_range_m_);

    const int center_x = image_width_ / 2;
    const int base_y = image_height_ - bottom_margin_;

    const int px = center_x + static_cast<int>(std::round(p.y * px_per_meter_x));
    const int py = base_y - static_cast<int>(std::round(p.x * px_per_meter_y));
    return {px, py};
  }

  cv::Scalar colorForPoint(const RadarPoint & p, const double age_sec, const bool lateral_candidate) const
  {
    const double fade = show_history_ ? std::max(0.12, 1.0 - age_sec / std::max(0.01, history_window_sec_)) : 1.0;

    if (p.is_infra) {
      return cv::Scalar(70.0 * fade, 70.0 * fade, 70.0 * fade);
    }

    const bool static_like = !isFinite(p.doppler) || std::abs(p.doppler) < static_doppler_threshold_mps_;
    if (static_like) {
      if (lateral_candidate && emphasize_lateral_motion_) {
        return cv::Scalar(60.0 * fade, 215.0 * fade, 90.0 * fade);
      }
      return cv::Scalar(165.0 * fade, 165.0 * fade, 165.0 * fade);
    }

    const double speed = std::min(1.0, std::abs(static_cast<double>(p.doppler)) / 3.0);
    if (p.doppler < 0.0F) {
      return cv::Scalar(40.0 * fade, 90.0 * fade + 90.0 * speed * fade, 190.0 * fade + 55.0 * speed * fade);
    }
    return cv::Scalar(190.0 * fade + 55.0 * speed * fade, 80.0 * fade, 40.0 * fade);
  }

  void drawGrid(cv::Mat & image) const
  {
    const cv::Scalar grid_color(50, 50, 50);
    const int center_x = image_width_ / 2;
    const int base_y = image_height_ - bottom_margin_;

    cv::line(image, cv::Point(center_x, base_y), cv::Point(center_x, top_margin_), grid_color, 1, cv::LINE_AA);

    for (int r = 2; r <= static_cast<int>(max_range_m_); r += 2) {
      const RadarPoint left{static_cast<float>(r * std::cos(max_abs_azimuth_deg_ * kDegToRad)), static_cast<float>(-r * std::sin(max_abs_azimuth_deg_ * kDegToRad))};
      const RadarPoint right{static_cast<float>(r * std::cos(max_abs_azimuth_deg_ * kDegToRad)), static_cast<float>(r * std::sin(max_abs_azimuth_deg_ * kDegToRad))};
      const auto p1 = projectToCanvas(left);
      const auto p2 = projectToCanvas(right);
      cv::ellipse(image, cv::Point(center_x, base_y), cv::Size(std::abs(p2.x - center_x), std::abs(base_y - p1.y)), 0.0, 180.0, 360.0, grid_color, 1, cv::LINE_AA);
      cv::putText(image, std::to_string(r) + "m", cv::Point(center_x + 6, p1.y - 4), cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(120, 120, 120), 1, cv::LINE_AA);
    }

    for (int deg = -static_cast<int>(max_abs_azimuth_deg_); deg <= static_cast<int>(max_abs_azimuth_deg_); deg += 10) {
      const double rad = deg * kDegToRad;
      const RadarPoint edge{static_cast<float>(max_range_m_ * std::cos(rad)), static_cast<float>(max_range_m_ * std::sin(rad))};
      const auto p = projectToCanvas(edge);
      cv::line(image, cv::Point(center_x, base_y), p, cv::Scalar(35, 35, 35), 1, cv::LINE_AA);
    }
  }

  void drawVehicle(cv::Mat & image) const
  {
    const int center_x = image_width_ / 2;
    const int base_y = image_height_ - bottom_margin_;
    const cv::Rect rect(center_x - 18, base_y - 10, 36, 20);
    cv::rectangle(image, rect, cv::Scalar(220, 220, 220), cv::FILLED, cv::LINE_AA);
    cv::rectangle(image, rect, cv::Scalar(90, 90, 90), 1, cv::LINE_AA);
    cv::line(image, cv::Point(center_x, base_y - 18), cv::Point(center_x, base_y - 30), cv::Scalar(220, 220, 220), 2, cv::LINE_AA);
  }

  void drawStatusOverlay(cv::Mat & image, const int infra_count, const int non_infra_count, const int lateral_targets) const
  {
    std::vector<std::string> lines;
    std::ostringstream oss;
    oss.setf(std::ios::fixed);
    oss.precision(2);

    lines.push_back("RADAR BEV");
    oss.str("");
    oss << "Infra topic: " << (isTopicFresh(last_infra_rx_time_) ? "ONLINE" : "STALE")
        << "  pts=" << last_infra_points_count_;
    lines.push_back(oss.str());

    oss.str("");
    oss << "Non-infra topic: " << (isTopicFresh(last_non_infra_rx_time_) ? "ONLINE" : "STALE")
        << "  pts=" << last_non_infra_points_count_;
    lines.push_back(oss.str());

    oss.str("");
    oss << "Shown infra=" << infra_count << "  non-infra=" << non_infra_count;
    lines.push_back(oss.str());

    oss.str("");
    oss << "Stable lateral targets=" << lateral_targets;
    lines.push_back(oss.str());

    oss.str("");
    oss << "ROI: " << min_range_m_ << "-" << max_range_m_ << "m  |az|<" << max_abs_azimuth_deg_ << "deg";
    lines.push_back(oss.str());

    const int panel_x = 18;
    const int panel_y = 18;
    const int panel_w = 420;
    const int panel_h = 24 + static_cast<int>(lines.size()) * 26;
    cv::rectangle(image, cv::Rect(panel_x, panel_y, panel_w, panel_h), cv::Scalar(20, 20, 20), cv::FILLED);
    cv::rectangle(image, cv::Rect(panel_x, panel_y, panel_w, panel_h), cv::Scalar(70, 70, 70), 1);

    int y = panel_y + 28;
    for (size_t i = 0; i < lines.size(); ++i) {
      const double scale = (i == 0) ? 0.7 : 0.55;
      const int thickness = (i == 0) ? 2 : 1;
      const cv::Scalar color = (i == 0) ? cv::Scalar(255, 255, 255) : cv::Scalar(200, 200, 200);
      cv::putText(image, lines[i], cv::Point(panel_x + 12, y), cv::FONT_HERSHEY_SIMPLEX, scale, color, thickness, cv::LINE_AA);
      y += 26;
    }
  }

  void drawLegend(cv::Mat & image) const
  {
    const int origin_x = image_width_ - 350;
    const int origin_y = 22;
    cv::rectangle(image, cv::Rect(origin_x, origin_y, 320, 148), cv::Scalar(20, 20, 20), cv::FILLED);
    cv::rectangle(image, cv::Rect(origin_x, origin_y, 320, 148), cv::Scalar(70, 70, 70), 1);

    cv::putText(image, "Legend", cv::Point(origin_x + 12, origin_y + 24), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);

    drawLegendItem(image, origin_x + 12, origin_y + 46, cv::Scalar(210, 210, 210), "Infra clutter");
    drawLegendItem(image, origin_x + 12, origin_y + 70, cv::Scalar(180, 180, 180), "Static / near-static");
    drawLegendItem(image, origin_x + 12, origin_y + 94, cv::Scalar(220, 80, 40), "Approaching (toward radar)");
    drawLegendItem(image, origin_x + 12, origin_y + 118, cv::Scalar(40, 160, 220), "Receding (away from radar)");
    drawLegendItem(image, origin_x + 12, origin_y + 142, cv::Scalar(60, 215, 90), "Lateral motion hint");
  }

  void drawLegendItem(cv::Mat & image, const int x, const int y, const cv::Scalar & color, const std::string & text) const
  {
    cv::circle(image, cv::Point(x + 8, y - 4), 5, color, cv::FILLED, cv::LINE_AA);
    cv::putText(image, text, cv::Point(x + 24, y), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(210, 210, 210), 1, cv::LINE_AA);
  }

  bool isTopicFresh(const rclcpp::Time & time) const
  {
    if (time.nanoseconds() == 0) {
      return false;
    }
    return (this->now() - time).seconds() < std::max(1.5, 3.0 / std::max(1.0, publish_rate_hz_));
  }

  std::vector<CandidateTarget> findCandidateTargets(const std::vector<RadarPoint> & render_points) const
  {
    std::unordered_map<std::int64_t, int> hits;
    std::unordered_map<std::int64_t, RadarPoint> newest_point;
    std::unordered_map<std::int64_t, std::vector<RadarPoint>> grouped_points;
    std::unordered_map<std::int64_t, double> min_y;
    std::unordered_map<std::int64_t, double> max_y;

    for (const auto & p : render_points) {
      if (p.is_infra) {
        continue;
      }
      const int gx = static_cast<int>(std::floor(static_cast<double>(p.x) / track_grid_m_));
      const int gy = static_cast<int>(std::floor(static_cast<double>(p.y) / track_grid_m_));
      const std::int64_t key = makeGridKey(gx, gy);
      ++hits[key];
      newest_point[key] = p;
      grouped_points[key].push_back(p);

      const auto yv = static_cast<double>(p.y);
      if (min_y.find(key) == min_y.end()) {
        min_y[key] = yv;
        max_y[key] = yv;
      } else {
        min_y[key] = std::min(min_y[key], yv);
        max_y[key] = std::max(max_y[key], yv);
      }
    }

    std::vector<CandidateTarget> candidates;
    for (const auto & kv : hits) {
      if (kv.second < stable_track_min_hits_) {
        continue;
      }
      CandidateTarget c;
      c.point = newest_point[kv.first];
      c.stable_hits = kv.second;
      c.nearby_count = static_cast<int>(grouped_points[kv.first].size());
      c.lateral_motion_score = std::abs(max_y[kv.first] - min_y[kv.first]);
      candidates.push_back(c);
    }

    std::sort(candidates.begin(), candidates.end(), [](const CandidateTarget & a, const CandidateTarget & b) {
      if (a.lateral_motion_score == b.lateral_motion_score) {
        return a.stable_hits > b.stable_hits;
      }
      return a.lateral_motion_score > b.lateral_motion_score;
    });
    return candidates;
  }

  std::set<std::int64_t> selectLateralCandidateKeys(const std::vector<CandidateTarget> & candidates) const
  {
    std::set<std::int64_t> selected;
    int selected_count = 0;
    for (const auto & c : candidates) {
      if (c.lateral_motion_score < lateral_motion_min_shift_m_) {
        continue;
      }
      const int gx = static_cast<int>(std::floor(static_cast<double>(c.point.x) / track_grid_m_));
      const int gy = static_cast<int>(std::floor(static_cast<double>(c.point.y) / track_grid_m_));
      selected.insert(makeGridKey(gx, gy));
      ++selected_count;
      if (selected_count >= label_top_k_) {
        break;
      }
    }
    return selected;
  }

  void drawTracksForLateralTargets(cv::Mat & image, const std::vector<RadarPoint> & render_points, const std::set<std::int64_t> & lateral_keys) const
  {
    if (!emphasize_lateral_motion_ || lateral_keys.empty()) {
      return;
    }

    std::unordered_map<std::int64_t, std::vector<RadarPoint>> grouped;
    for (const auto & p : render_points) {
      if (p.is_infra) {
        continue;
      }
      const int gx = static_cast<int>(std::floor(static_cast<double>(p.x) / track_grid_m_));
      const int gy = static_cast<int>(std::floor(static_cast<double>(p.y) / track_grid_m_));
      const std::int64_t key = makeGridKey(gx, gy);
      if (lateral_keys.count(key) == 0U) {
        continue;
      }
      grouped[key].push_back(p);
    }

    for (auto & kv : grouped) {
      auto & pts = kv.second;
      std::sort(pts.begin(), pts.end(), [](const RadarPoint & a, const RadarPoint & b) {
        return a.stamp < b.stamp;
      });

      for (size_t i = 1; i < pts.size(); ++i) {
        const auto p0 = projectToCanvas(pts[i - 1]);
        const auto p1 = projectToCanvas(pts[i]);
        cv::line(image, p0, p1, cv::Scalar(60, 215, 90), 2, cv::LINE_AA);
      }
    }
  }

  void drawCandidateRings(cv::Mat & image, const std::vector<CandidateTarget> & candidates) const
  {
    if (!show_target_rings_) {
      return;
    }

    int label_count = 0;
    int dynamic_count = 0;
    for (const auto & c : candidates) {
      const bool dynamic_like = isFinite(c.point.doppler) && std::abs(c.point.doppler) >= static_doppler_threshold_mps_;
      const bool lateral_like = c.lateral_motion_score >= lateral_motion_min_shift_m_;
      if (!dynamic_like && !lateral_like) {
        continue;
      }
      if (dynamic_like && dynamic_count >= dynamic_label_top_k_) {
        continue;
      }
      if (label_count >= label_top_k_) {
        break;
      }

      const auto px = projectToCanvas(c.point);
      cv::circle(image, px, 11, lateral_like ? cv::Scalar(60, 215, 90) : cv::Scalar(255, 255, 255), 2, cv::LINE_AA);

      if (show_labels_) {
        std::ostringstream label;
        label.setf(std::ios::fixed);
        label.precision(1);
        label << "R=" << c.point.range << "m";
        if (isFinite(c.point.doppler)) {
          label << " Vr=" << c.point.doppler << "m/s";
        }
        if (lateral_like) {
          label << " Ly=" << c.lateral_motion_score << "m";
        }
        cv::putText(image, label.str(), cv::Point(px.x + 10, px.y - 10), cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(235, 235, 235), 1, cv::LINE_AA);
      }

      ++label_count;
      if (dynamic_like) {
        ++dynamic_count;
      }
    }
  }

  void renderAndPublish()
  {
    pruneHistory();

    cv::Mat image(image_height_, image_width_, CV_8UC3, cv::Scalar(12, 12, 12));
    if (show_grid_) {
      drawGrid(image);
    }
    drawVehicle(image);

    std::vector<RadarPoint> render_points = history_;
    std::sort(render_points.begin(), render_points.end(), [](const RadarPoint & a, const RadarPoint & b) {
      return a.stamp < b.stamp;
    });

    const auto candidates = findCandidateTargets(render_points);
    const auto lateral_keys = selectLateralCandidateKeys(candidates);
    drawTracksForLateralTargets(image, render_points, lateral_keys);

    int shown_infra = 0;
    int shown_non_infra = 0;
    const rclcpp::Time now = this->now();

    for (const auto & p : render_points) {
      const auto px = projectToCanvas(p);
      if (px.x < 0 || px.x >= image_width_ || px.y < 0 || px.y >= image_height_) {
        continue;
      }

      const int gx = static_cast<int>(std::floor(static_cast<double>(p.x) / track_grid_m_));
      const int gy = static_cast<int>(std::floor(static_cast<double>(p.y) / track_grid_m_));
      const bool lateral_candidate = lateral_keys.count(makeGridKey(gx, gy)) > 0U;

      const double age_sec = std::max(0.0, (now - p.stamp).seconds());
      const auto color = colorForPoint(p, age_sec, lateral_candidate);
      int radius = p.is_infra ? 2 : 4;

      if (emphasize_non_infra_ && !p.is_infra) {
        const bool dynamic_like = isFinite(p.doppler) && std::abs(p.doppler) >= static_doppler_threshold_mps_;
        if (lateral_candidate) {
          radius = 6;
        } else if (dynamic_like) {
          radius = 5;
        } else {
          radius = 3;
        }
      }

      cv::circle(image, px, radius, color, cv::FILLED, cv::LINE_AA);

      if (p.is_infra) {
        ++shown_infra;
      } else {
        ++shown_non_infra;
      }
    }

    drawCandidateRings(image, candidates);

    if (show_status_overlay_) {
      int lateral_targets = 0;
      for (const auto & c : candidates) {
        if (c.lateral_motion_score >= lateral_motion_min_shift_m_) {
          ++lateral_targets;
        }
      }
      drawStatusOverlay(image, shown_infra, shown_non_infra, lateral_targets);
    }
    drawLegend(image);

    auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();
    msg->header.stamp = this->now();
    msg->header.frame_id = frame_id_;
    image_pub_->publish(*msg);
  }

  std::string infra_topic_;
  std::string non_infra_topic_;
  std::string output_image_topic_;
  std::string frame_id_;

  int image_width_{1280};
  int image_height_{720};
  const int top_margin_{70};
  const int bottom_margin_{50};

  double max_range_m_{10.0};
  double min_range_m_{0.8};
  double max_abs_azimuth_deg_{25.0};
  double min_z_m_{-2.0};
  double max_z_m_{2.0};
  double max_azimuth_std_deg_{6.0};
  double static_doppler_threshold_mps_{0.2};
  double history_window_sec_{0.45};
  double publish_rate_hz_{15.0};
  double track_grid_m_{0.35};
  int stable_track_min_hits_{3};
  double lateral_motion_min_shift_m_{0.25};
  int label_top_k_{3};
  int dynamic_label_top_k_{1};

  bool show_infra_{true};
  bool show_non_infra_{true};
  bool show_grid_{true};
  bool show_labels_{true};
  bool show_history_{true};
  bool show_status_overlay_{true};
  bool emphasize_non_infra_{true};
  bool show_target_rings_{true};
  bool emphasize_lateral_motion_{true};

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr infra_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr non_infra_sub_;
  rclcpp::TimerBase::SharedPtr render_timer_;

  std::vector<RadarPoint> history_;

  rclcpp::Time last_infra_rx_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_non_infra_rx_time_{0, 0, RCL_ROS_TIME};
  int last_infra_points_count_{0};
  int last_non_infra_points_count_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RadarBevVisualizer>());
  rclcpp::shutdown();
  return 0;
}
