#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>

class TargetExtractor : public rclcpp::Node {
public:
    TargetExtractor() : Node("target_extractor_node") {
        input_topic_ = declare_parameter<std::string>("input_topic", "/radar_detection_pcl_non_infra");

        min_x_ = declare_parameter<double>("min_x", 1.0);
        max_x_ = declare_parameter<double>("max_x", 150.0);
        max_abs_y_ = declare_parameter<double>("max_abs_y", 2.5);
        max_abs_azimuth_deg_ = declare_parameter<double>("max_abs_azimuth_deg", 10.0);
        max_azimuth_std_deg_ = declare_parameter<double>("max_azimuth_std_deg", 10.0);

        min_hits_to_validate_ = declare_parameter<int>("min_hits_to_validate", 3);
        max_misses_to_invalidate_ = declare_parameter<int>("max_misses_to_invalidate", 5);

        use_range_instead_of_x_ = declare_parameter<bool>("use_range_instead_of_x", false);
        invert_doppler_sign_ = declare_parameter<bool>("invert_doppler_sign", false);

        sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic_, 10,
            std::bind(&TargetExtractor::pointCloudCallback, this, std::placeholders::_1));

        dist_pub_ = create_publisher<std_msgs::msg::Float32>("/radar_target/dist_rel", 10);
        vrel_pub_ = create_publisher<std_msgs::msg::Float32>("/radar_target/v_rel", 10);
        valid_pub_ = create_publisher<std_msgs::msg::Bool>("/radar_target/obj_valid", 10);

        RCLCPP_INFO(get_logger(), "Target extractor started. Subscribing to: %s", input_topic_.c_str());
    }

private:
    struct RadarPoint {
        float x;
        float y;
        float z;
        float doppler;
        float range;
        float azimuth;
        float azimuth_std;
    };

    struct Candidate {
        float x;
        float y;
        float doppler;
        float range;
        float azimuth;
        float azimuth_std;
    };

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr dist_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr vrel_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr valid_pub_;

    std::string input_topic_;

    double min_x_;
    double max_x_;
    double max_abs_y_;
    double max_abs_azimuth_deg_;
    double max_azimuth_std_deg_;

    int min_hits_to_validate_;
    int max_misses_to_invalidate_;

    bool use_range_instead_of_x_;
    bool invert_doppler_sign_;

    int hit_count_ = 0;
    int miss_count_ = 0;
    bool obj_valid_ = false;

    static bool getFieldOffset(
        const sensor_msgs::msg::PointCloud2 &msg,
        const std::string &field_name,
        uint32_t &offset_out)
    {
        for (const auto &field : msg.fields) {
            if (field.name == field_name) {
                if (field.datatype != sensor_msgs::msg::PointField::FLOAT32 || field.count != 1) {
                    return false;
                }
                offset_out = field.offset;
                return true;
            }
        }
        return false;
    }

    static float readFloat32(
        const std::vector<uint8_t> &data,
        size_t base,
        uint32_t offset)
    {
        float value;
        std::memcpy(&value, data.data() + base + offset, sizeof(float));
        return value;
    }

    bool parseCandidates(
        const sensor_msgs::msg::PointCloud2 &msg,
        std::vector<Candidate> &candidates)
    {
        candidates.clear();

        if (msg.height != 1) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "Expected unorganized point cloud with height=1, got height=%u", msg.height);
        }

        if (msg.point_step == 0 || msg.data.empty() || msg.width == 0) {
            return true;
        }

        uint32_t offset_x, offset_y, offset_doppler, offset_range, offset_azimuth, offset_azimuth_std;
        bool ok = true;
        ok &= getFieldOffset(msg, "x", offset_x);
        ok &= getFieldOffset(msg, "y", offset_y);
        ok &= getFieldOffset(msg, "doppler", offset_doppler);
        ok &= getFieldOffset(msg, "range", offset_range);
        ok &= getFieldOffset(msg, "azimuth", offset_azimuth);
        ok &= getFieldOffset(msg, "azimuth_std", offset_azimuth_std);

        if (!ok) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                                  "PointCloud2 does not contain expected float32 fields: x, y, doppler, range, azimuth, azimuth_std");
            return false;
        }

        candidates.reserve(msg.width);

        for (uint32_t i = 0; i < msg.width; ++i) {
            size_t base = static_cast<size_t>(i) * msg.point_step;
            if (base + msg.point_step > msg.data.size()) {
                break;
            }

            Candidate c;
            c.x = readFloat32(msg.data, base, offset_x);
            c.y = readFloat32(msg.data, base, offset_y);
            c.doppler = readFloat32(msg.data, base, offset_doppler);
            c.range = readFloat32(msg.data, base, offset_range);
            c.azimuth = readFloat32(msg.data, base, offset_azimuth);
            c.azimuth_std = readFloat32(msg.data, base, offset_azimuth_std);

            if (!std::isfinite(c.x) || !std::isfinite(c.y) ||
                !std::isfinite(c.range) || !std::isfinite(c.azimuth) ||
                !std::isfinite(c.azimuth_std) || !std::isfinite(c.doppler)) {
                continue;
            }

            if (c.x < min_x_ || c.x > max_x_) {
                continue;
            }

            if (std::abs(c.y) > max_abs_y_) {
                continue;
            }

            if (std::abs(c.azimuth) > max_abs_azimuth_deg_) {
                continue;
            }

            if (c.azimuth_std > max_azimuth_std_deg_) {
                continue;
            }

            candidates.push_back(c);
        }

        return true;
    }

    bool selectMainTarget(
        const std::vector<Candidate> &candidates,
        Candidate &target)
    {
        if (candidates.empty()) {
            return false;
        }

        auto it = std::min_element(
            candidates.begin(), candidates.end(),
            [](const Candidate &a, const Candidate &b) {
                if (a.x != b.x) {
                    return a.x < b.x;
                }
                return std::abs(a.y) < std::abs(b.y);
            });

        target = *it;
        return true;
    }

    void publishOutputs(float dist_rel, float v_rel, bool obj_valid)
    {
        std_msgs::msg::Float32 dist_msg;
        dist_msg.data = dist_rel;
        dist_pub_->publish(dist_msg);

        std_msgs::msg::Float32 vrel_msg;
        vrel_msg.data = v_rel;
        vrel_pub_->publish(vrel_msg);

        std_msgs::msg::Bool valid_msg;
        valid_msg.data = obj_valid;
        valid_pub_->publish(valid_msg);
    }

    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        std::vector<Candidate> candidates;
        if (!parseCandidates(*msg, candidates)) {
            return;
        }

        Candidate target;
        const bool found = selectMainTarget(candidates, target);

        if (found) {
            hit_count_++;
            miss_count_ = 0;

            if (hit_count_ >= min_hits_to_validate_) {
                obj_valid_ = true;
            }

            float dist_rel = use_range_instead_of_x_ ? target.range : target.x;
            float v_rel = invert_doppler_sign_ ? (-target.doppler) : target.doppler;

            publishOutputs(dist_rel, v_rel, obj_valid_);

            RCLCPP_DEBUG(
                get_logger(),
                "Target found: x=%.3f y=%.3f range=%.3f azimuth=%.3f azimuth_std=%.3f doppler=%.3f valid=%d",
                target.x, target.y, target.range, target.azimuth, target.azimuth_std, target.doppler, obj_valid_);
        } else {
            hit_count_ = 0;
            miss_count_++;

            if (miss_count_ >= max_misses_to_invalidate_) {
                obj_valid_ = false;
            }

            publishOutputs(0.0f, 0.0f, obj_valid_);

            RCLCPP_DEBUG(get_logger(), "No valid target found. valid=%d miss_count=%d", obj_valid_, miss_count_);
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TargetExtractor>());
    rclcpp::shutdown();
    return 0;
}