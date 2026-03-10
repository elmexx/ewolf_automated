#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "lane_parameter_msg/msg/lane_marking_projected_array_both.hpp"
#include "lane_parameter_msg/msg/lane_marking_projected.hpp"
#include <vector>
#include <cmath>
#include <deque>
#include <algorithm>
#include <spline.h>  

class LaneTransformerNode : public rclcpp::Node {
public:
    LaneTransformerNode() : Node("lane_transformer_node") {
        declare_parameter("fusion_alpha", 0.3);      
        declare_parameter("interp_resolution", 0.5); 
        declare_parameter("smooth_window", 5);      
        declare_parameter("lowpass_alpha", 0.2);    

        alpha_ = get_parameter("fusion_alpha").as_double();
        interp_resolution_ = get_parameter("interp_resolution").as_double();
        smooth_window_ = get_parameter("smooth_window").as_int();
        lowpass_alpha_ = get_parameter("lowpass_alpha").as_double();

        lane_markings_sub_ = create_subscription<lane_parameter_msg::msg::LaneMarkingProjectedArrayBoth>(
            "/detection/lane/lane_markings_projected", 10,
            std::bind(&LaneTransformerNode::laneMarkingsCallback, this, std::placeholders::_1));
        
        path_pub_ = create_publisher<nav_msgs::msg::Path>("/reference_path", 10);
        
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

private:
    void laneMarkingsCallback(const lane_parameter_msg::msg::LaneMarkingProjectedArrayBoth::SharedPtr msg) {
        geometry_msgs::msg::TransformStamped transform;
        try
        {
            transform = tf_buffer_->lookupTransform("odom", "base_link", tf2::TimePointZero);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Failed to get transform: %s", ex.what());
            return;
        }

        auto transformed_msg = transformLaneMarkings(*msg, transform);

        if (!transformed_msg.markings_left.empty()) {
            fuseWithHistory(transformed_msg);
        } else {
            transformed_msg = last_valid_markings_;
        }

        auto raw_path = generateReferencePath(transformed_msg);

        auto interp_path = interpolatePath(raw_path);
        auto smooth_path = smoothPath(raw_path); //smoothPath(interp_path);
        auto filtered_path = lowpassFilter(smooth_path);

        publishPath(filtered_path, msg->header.stamp);
    }

    lane_parameter_msg::msg::LaneMarkingProjectedArrayBoth transformLaneMarkings(
        const lane_parameter_msg::msg::LaneMarkingProjectedArrayBoth &msg,
        const geometry_msgs::msg::TransformStamped &transform
    ) {
        lane_parameter_msg::msg::LaneMarkingProjectedArrayBoth transformed;
        transformed.header.stamp = get_clock()->now();
        transformed.header.frame_id = "odom";


        auto transform_point = [&](const auto &point) {
            lane_parameter_msg::msg::LaneMarkingProjected p;
            
            tf2::Quaternion q(
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            );
            tf2::Vector3 translation(
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z
            );

            tf2::Vector3 vec(point.x, point.y, point.z);

            tf2::Transform tf_transform(q, translation);
            tf2::Vector3 transformed = tf_transform * vec; 

            p.x = transformed.x();
            p.y = transformed.y();
            p.z = transformed.z();
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

    void fuseWithHistory(lane_parameter_msg::msg::LaneMarkingProjectedArrayBoth &current) {
        if (last_valid_markings_.markings_left.empty()) {
            last_valid_markings_ = current;
            return;
        }

        size_t min_size = std::min(
            last_valid_markings_.markings_left.size(),
            current.markings_left.size()
        );

        for (size_t i = 0; i < min_size; ++i) {
            auto& last_left = last_valid_markings_.markings_left[i];
            auto& curr_left = current.markings_left[i];
            last_left.x = alpha_ * curr_left.x + (1 - alpha_) * last_left.x;
            last_left.y = alpha_ * curr_left.y + (1 - alpha_) * last_left.y;

            auto& last_right = last_valid_markings_.markings_right[i];
            auto& curr_right = current.markings_right[i];
            last_right.x = alpha_ * curr_right.x + (1 - alpha_) * last_right.x;
            last_right.y = alpha_ * curr_right.y + (1 - alpha_) * last_right.y;
        }

        for (size_t i = min_size; i < current.markings_left.size(); ++i) {
            last_valid_markings_.markings_left.push_back(current.markings_left[i]);
            last_valid_markings_.markings_right.push_back(current.markings_right[i]);
        }
    }

    std::vector<geometry_msgs::msg::PoseStamped> generateReferencePath(
        const lane_parameter_msg::msg::LaneMarkingProjectedArrayBoth &markings
    ) {
        std::vector<geometry_msgs::msg::PoseStamped> path;
        size_t min_size = std::min(markings.markings_left.size(), markings.markings_right.size());

        for (size_t i = 0; i < min_size; ++i) {
            geometry_msgs::msg::PoseStamped pose;
            pose.pose.position.x = (markings.markings_left[i].x + markings.markings_right[i].x) / 2.0;
            pose.pose.position.y = (markings.markings_left[i].y + markings.markings_right[i].y) / 2.0;
            pose.pose.orientation.w = 1.0;
            path.push_back(pose);
        }
        return path;
    }

    std::vector<geometry_msgs::msg::PoseStamped> interpolatePath(
        const std::vector<geometry_msgs::msg::PoseStamped> &raw_path
    ) {
        if (raw_path.size() < 3) return raw_path;

        std::vector<double> x, y, s;
        double total_s = 0.0;
        for (size_t i = 0; i < raw_path.size(); ++i) {
            x.push_back(raw_path[i].pose.position.x);
            y.push_back(raw_path[i].pose.position.y);
            if (i > 0) {
                double dx = x[i] - x[i-1];
                double dy = y[i] - y[i-1];
                total_s += std::hypot(dx, dy);
            }
            s.push_back(total_s);
        }

        tk::spline spline_x, spline_y;
        spline_x.set_points(s, x);
        spline_y.set_points(s, y);

        std::vector<geometry_msgs::msg::PoseStamped> interp_path;
        for (double s_i = 0.0; s_i <= total_s; s_i += interp_resolution_) {
            geometry_msgs::msg::PoseStamped pose;
            pose.pose.position.x = spline_x(s_i);
            pose.pose.position.y = spline_y(s_i);
            interp_path.push_back(pose);
        }

        return interp_path;
    }

    std::vector<geometry_msgs::msg::PoseStamped> smoothPath(
        const std::vector<geometry_msgs::msg::PoseStamped> &path
    ) {
        std::vector<geometry_msgs::msg::PoseStamped> smoothed;
        int n = path.size();
        if (n == 0) return smoothed;

        for (int i = 0; i < n; ++i) {
            double sum_x = 0.0, sum_y = 0.0;
            int count = 0;
            for (int j = std::max(0, i - smooth_window_); j <= std::min(n-1, i + smooth_window_); ++j) {
                sum_x += path[j].pose.position.x;
                sum_y += path[j].pose.position.y;
                count++;
            }
            geometry_msgs::msg::PoseStamped p = path[i];
            p.pose.position.x = sum_x / count;
            p.pose.position.y = sum_y / count;
            smoothed.push_back(p);
        }
        return smoothed;
    }

    std::vector<geometry_msgs::msg::PoseStamped> lowpassFilter(
        const std::vector<geometry_msgs::msg::PoseStamped> &path
    ) {
        if (path.empty()) return path;

        std::vector<geometry_msgs::msg::PoseStamped> filtered = path;
        for (size_t i = 1; i < path.size(); ++i) {
            filtered[i].pose.position.x = lowpass_alpha_ * path[i].pose.position.x + 
                                        (1 - lowpass_alpha_) * filtered[i-1].pose.position.x;
            filtered[i].pose.position.y = lowpass_alpha_ * path[i].pose.position.y + 
                                        (1 - lowpass_alpha_) * filtered[i-1].pose.position.y;
        }
        return filtered;
    }

    void publishPath(
        const std::vector<geometry_msgs::msg::PoseStamped> &path,
        const builtin_interfaces::msg::Time &stamp
    ) {
        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = stamp;
        path_msg.header.frame_id = "odom";
        path_msg.poses = path;
        path_pub_->publish(path_msg);
    }

    rclcpp::Subscription<lane_parameter_msg::msg::LaneMarkingProjectedArrayBoth>::SharedPtr lane_markings_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    lane_parameter_msg::msg::LaneMarkingProjectedArrayBoth last_valid_markings_;
    double alpha_;                
    double interp_resolution_;    
    int smooth_window_;           
    double lowpass_alpha_;        
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaneTransformerNode>());
    rclcpp::shutdown();
    return 0;
}
// #include <rclcpp/rclcpp.hpp>
// #include <nav_msgs/msg/path.hpp>
// #include <geometry_msgs/msg/point.hpp>
// #include <geometry_msgs/msg/transform_stamped.hpp>
// #include <tf2_ros/buffer.h>
// #include <tf2_ros/transform_listener.h>
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2/LinearMath/Matrix3x3.h>
// #include "lane_parameter_msg/msg/lane_marking_projected_array_both.hpp"
// #include "lane_parameter_msg/msg/lane_marking_projected.hpp"
// #include <vector>
// #include <cmath>
// #include <deque>

// class LaneTransformerNode : public rclcpp::Node
// {
// public:
//     LaneTransformerNode() : Node("lane_transformer_node")
//     {
//         lane_markings_subscription_ = this->create_subscription<lane_parameter_msg::msg::LaneMarkingProjectedArrayBoth>(
//             "/detection/lane/lane_markings_projected", 10,
//             std::bind(&LaneTransformerNode::laneMarkingsCallback, this, std::placeholders::_1));

//         lane_markings_publisher_ = this->create_publisher<lane_parameter_msg::msg::LaneMarkingProjectedArrayBoth>(
//             "/lane_markings_in_odom", 10);

//         path_publisher_ = this->create_publisher<nav_msgs::msg::Path>(
//             "/reference_path", 10);

//         tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
//         tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

//         RCLCPP_INFO(this->get_logger(), "Lane Transformer Node Initialized.");
//     }

// private:
//     void laneMarkingsCallback(const lane_parameter_msg::msg::LaneMarkingProjectedArrayBoth::SharedPtr msg)
//     {
//         geometry_msgs::msg::TransformStamped transform;
//         try
//         {
//             transform = tf_buffer_->lookupTransform("odom", "base_link", tf2::TimePointZero);
//         }
//         catch (const tf2::TransformException &ex)
//         {
//             RCLCPP_WARN(this->get_logger(), "Failed to get transform: %s", ex.what());
//             return;
//         }

//         lane_parameter_msg::msg::LaneMarkingProjectedArrayBoth transformed_msg;
//         transformed_msg.header.stamp = this->get_clock()->now();
//         transformed_msg.header.frame_id = "odom";

//         size_t left_size = msg->markings_left.size();
//         size_t right_size = msg->markings_right.size();
//         if (left_size != right_size)
//         {
//             RCLCPP_WARN(this->get_logger(), "Left and right lane markings have different sizes: %zu vs %zu", left_size, right_size);
//         }

//         size_t min_size = std::min(left_size, right_size);
//         for (size_t i = 0; i < min_size; ++i)
//         {
//             auto left_transformed = transformPoint(msg->markings_left[i], transform);
//             auto right_transformed = transformPoint(msg->markings_right[i], transform);

//             transformed_msg.markings_left.push_back(left_transformed);
//             transformed_msg.markings_right.push_back(right_transformed);
//         }

//         if (!transformed_msg.markings_left.empty() && !transformed_msg.markings_right.empty()) {
            
//             updateHistory(transformed_msg);
//         } else {
            
//             transformed_msg = last_valid_lane_markings_;
//         }

        
//         nav_msgs::msg::Path path_msg;
//         path_msg.header = transformed_msg.header;
//         for (size_t i = 0; i < transformed_msg.markings_left.size(); ++i) {
//             geometry_msgs::msg::PoseStamped pose;
//             pose.header = path_msg.header;
//             pose.pose.position.x = (transformed_msg.markings_left[i].x + transformed_msg.markings_right[i].x) / 2.0;
//             pose.pose.position.y = (transformed_msg.markings_left[i].y + transformed_msg.markings_right[i].y) / 2.0;
//             pose.pose.position.z = (transformed_msg.markings_left[i].z + transformed_msg.markings_right[i].z) / 2.0;
//             pose.pose.orientation.w = 1.0;
//             path_msg.poses.push_back(pose);
//         }

       
//         path_msg.poses = smoothPath(path_msg.poses, 5);

       
//         lane_markings_publisher_->publish(transformed_msg);
//         path_publisher_->publish(path_msg);
//     }

//     void updateHistory(lane_parameter_msg::msg::LaneMarkingProjectedArrayBoth &current_frame)
//     {
//         if (last_valid_lane_markings_.markings_left.empty() || last_valid_lane_markings_.markings_right.empty()) {
            
//             last_valid_lane_markings_ = current_frame;
//             return;
//         }

//         size_t overlap_size = std::min(last_valid_lane_markings_.markings_left.size(), current_frame.markings_left.size());
//         for (size_t i = 0; i < overlap_size; ++i) {
//             last_valid_lane_markings_.markings_left[i].x = (last_valid_lane_markings_.markings_left[i].x + current_frame.markings_left[i].x) / 2.0;
//             last_valid_lane_markings_.markings_left[i].y = (last_valid_lane_markings_.markings_left[i].y + current_frame.markings_left[i].y) / 2.0;
//             last_valid_lane_markings_.markings_left[i].z = (last_valid_lane_markings_.markings_left[i].z + current_frame.markings_left[i].z) / 2.0;

//             last_valid_lane_markings_.markings_right[i].x = (last_valid_lane_markings_.markings_right[i].x + current_frame.markings_right[i].x) / 2.0;
//             last_valid_lane_markings_.markings_right[i].y = (last_valid_lane_markings_.markings_right[i].y + current_frame.markings_right[i].y) / 2.0;
//             last_valid_lane_markings_.markings_right[i].z = (last_valid_lane_markings_.markings_right[i].z + current_frame.markings_right[i].z) / 2.0;
//         }

//         for (size_t i = overlap_size; i < current_frame.markings_left.size(); ++i) {
//             last_valid_lane_markings_.markings_left.push_back(current_frame.markings_left[i]);
//             last_valid_lane_markings_.markings_right.push_back(current_frame.markings_right[i]);
//         }
//     }

//     std::vector<geometry_msgs::msg::PoseStamped> smoothPath(const std::vector<geometry_msgs::msg::PoseStamped> &raw_path, int window_size)
//     {
//         std::vector<geometry_msgs::msg::PoseStamped> smoothed_path;
//         int n = raw_path.size();
//         if (n == 0 || window_size < 1)
//             return smoothed_path;

//         for (int i = 0; i < n; ++i)
//         {
//             double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
//             int count = 0;

//             for (int j = std::max(0, i - window_size); j <= std::min(n - 1, i + window_size); ++j)
//             {
//                 sum_x += raw_path[j].pose.position.x;
//                 sum_y += raw_path[j].pose.position.y;
//                 sum_z += raw_path[j].pose.position.z;
//                 count++;
//             }

//             geometry_msgs::msg::PoseStamped smoothed_pose = raw_path[i];
//             smoothed_pose.pose.position.x = sum_x / count;
//             smoothed_pose.pose.position.y = sum_y / count;
//             smoothed_pose.pose.position.z = sum_z / count;

//             smoothed_path.push_back(smoothed_pose);
//         }
//         return smoothed_path;
//     }

//     lane_parameter_msg::msg::LaneMarkingProjected transformPoint(
//         const lane_parameter_msg::msg::LaneMarkingProjected &point,
//         const geometry_msgs::msg::TransformStamped &transform)
//     {
//         double tx = transform.transform.translation.x;
//         double ty = transform.transform.translation.y;
//         double tz = transform.transform.translation.z;

//         tf2::Quaternion q(
//             transform.transform.rotation.x,
//             transform.transform.rotation.y,
//             transform.transform.rotation.z,
//             transform.transform.rotation.w);
//         tf2::Matrix3x3 rotation_matrix(q);

//         lane_parameter_msg::msg::LaneMarkingProjected transformed_point;
//         transformed_point.x = rotation_matrix[0][0] * point.x + rotation_matrix[0][1] * point.y +
//                               rotation_matrix[0][2] * point.z + tx;
//         transformed_point.y = rotation_matrix[1][0] * point.x + rotation_matrix[1][1] * point.y +
//                               rotation_matrix[1][2] * point.z + ty;
//         transformed_point.z = rotation_matrix[2][0] * point.x + rotation_matrix[2][1] * point.y +
//                               rotation_matrix[2][2] * point.z + tz;

//         return transformed_point;
//     }

//     rclcpp::Subscription<lane_parameter_msg::msg::LaneMarkingProjectedArrayBoth>::SharedPtr lane_markings_subscription_;
//     rclcpp::Publisher<lane_parameter_msg::msg::LaneMarkingProjectedArrayBoth>::SharedPtr lane_markings_publisher_;
//     rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
//     std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
//     std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

//     lane_parameter_msg::msg::LaneMarkingProjectedArrayBoth last_valid_lane_markings_;
// };

// int main(int argc, char *argv[])
// {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<LaneTransformerNode>());
//     rclcpp::shutdown();
//     return 0;
// }

// // #include <rclcpp/rclcpp.hpp>
// // #include <nav_msgs/msg/path.hpp>
// // #include <geometry_msgs/msg/point.hpp>
// // #include <geometry_msgs/msg/transform_stamped.hpp>
// // #include <tf2_ros/buffer.h>
// // #include <tf2_ros/transform_listener.h>
// // #include <tf2/LinearMath/Quaternion.h>
// // #include <tf2/LinearMath/Matrix3x3.h>
// // #include "lane_parameter_msg/msg/lane_marking_projected_array_both.hpp"
// // #include "lane_parameter_msg/msg/lane_marking_projected.hpp"
// // #include <vector>
// // #include <deque>
// // #include <cmath>

// // class LaneTransformerNode : public rclcpp::Node {
// // public:
// //     LaneTransformerNode() : Node("lane_transformer_node") {
// //         lane_markings_subscription_ = this->create_subscription<lane_parameter_msg::msg::LaneMarkingProjectedArrayBoth>(
// //             "/detection/lane/lane_markings_projected", 10,
// //             std::bind(&LaneTransformerNode::laneMarkingsCallback, this, std::placeholders::_1));

// //         lane_markings_publisher_ = this->create_publisher<lane_parameter_msg::msg::LaneMarkingProjectedArrayBoth>(
// //             "/lane_markings_in_odom", 10);

// //         path_publisher_ = this->create_publisher<nav_msgs::msg::Path>(
// //             "/reference_path", 10);

// //         tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
// //         tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

// //         RCLCPP_INFO(this->get_logger(), "Lane Transformer Node Initialized.");
// //     }

// // private:
// //     std::deque<nav_msgs::msg::Path> history_paths_;
// //     const size_t max_history_size_ = 10;

// //     void laneMarkingsCallback(const lane_parameter_msg::msg::LaneMarkingProjectedArrayBoth::SharedPtr msg) {
// //         geometry_msgs::msg::TransformStamped transform;
// //         try {
// //             transform = tf_buffer_->lookupTransform("odom", "base_link", tf2::TimePointZero);
// //         } catch (const tf2::TransformException &ex) {
// //             RCLCPP_WARN(this->get_logger(), "Failed to get transform: %s", ex.what());
// //             return;
// //         }

// //         lane_parameter_msg::msg::LaneMarkingProjectedArrayBoth transformed_msg;
// //         transformed_msg.header.stamp = this->get_clock()->now();
// //         transformed_msg.header.frame_id = "odom";

// //         nav_msgs::msg::Path path_msg;
// //         path_msg.header = transformed_msg.header;

// //         if (msg->markings_left.empty() || msg->markings_right.empty()) {
// //             if (!history_paths_.empty()) {
// //                 path_msg = history_paths_.back();
// //             }
// //         } else {
// //             size_t min_size = std::min(msg->markings_left.size(), msg->markings_right.size());
// //             for (size_t i = 0; i < min_size; ++i) {
// //                 auto left_transformed = transformPoint(msg->markings_left[i], transform);
// //                 auto right_transformed = transformPoint(msg->markings_right[i], transform);
                
// //                 geometry_msgs::msg::PoseStamped pose;
// //                 pose.header = path_msg.header;
// //                 pose.pose.position.x = (left_transformed.x + right_transformed.x) / 2.0;
// //                 pose.pose.position.y = (left_transformed.y + right_transformed.y) / 2.0;
// //                 pose.pose.position.z = (left_transformed.z + right_transformed.z) / 2.0;
// //                 pose.pose.orientation.w = 1.0;
// //                 path_msg.poses.push_back(pose);
// //             }
// //         }

// //         path_msg.poses = smoothPath(path_msg.poses, 5);

// //         if (history_paths_.size() >= max_history_size_) {
// //             history_paths_.pop_front();
// //         }
// //         history_paths_.push_back(path_msg);

// //         lane_markings_publisher_->publish(transformed_msg);
// //         path_publisher_->publish(path_msg);
// //     }

// //     std::vector<geometry_msgs::msg::PoseStamped> smoothPath(const std::vector<geometry_msgs::msg::PoseStamped> &raw_path, int window_size) {
// //         std::vector<geometry_msgs::msg::PoseStamped> smoothed_path;
// //         int n = raw_path.size();
// //         if (n == 0 || window_size < 1) return smoothed_path;

// //         for (int i = 0; i < n; ++i) {
// //             double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
// //             int count = 0;
// //             for (int j = std::max(0, i - window_size); j <= std::min(n - 1, i + window_size); ++j) {
// //                 sum_x += raw_path[j].pose.position.x;
// //                 sum_y += raw_path[j].pose.position.y;
// //                 sum_z += raw_path[j].pose.position.z;
// //                 count++;
// //             }
// //             geometry_msgs::msg::PoseStamped smoothed_pose = raw_path[i];
// //             smoothed_pose.pose.position.x = sum_x / count;
// //             smoothed_pose.pose.position.y = sum_y / count;
// //             smoothed_pose.pose.position.z = sum_z / count;
// //             smoothed_path.push_back(smoothed_pose);
// //         }
// //         return smoothed_path;
// //     }

// //     lane_parameter_msg::msg::LaneMarkingProjected transformPoint(
// //         const lane_parameter_msg::msg::LaneMarkingProjected &point,
// //         const geometry_msgs::msg::TransformStamped &transform) {
// //         double tx = transform.transform.translation.x;
// //         double ty = transform.transform.translation.y;
// //         double tz = transform.transform.translation.z;

// //         tf2::Quaternion q(
// //             transform.transform.rotation.x,
// //             transform.transform.rotation.y,
// //             transform.transform.rotation.z,
// //             transform.transform.rotation.w);
// //         tf2::Matrix3x3 rotation_matrix(q);

// //         lane_parameter_msg::msg::LaneMarkingProjected transformed_point;
// //         transformed_point.x = rotation_matrix[0][0] * point.x + rotation_matrix[0][1] * point.y + rotation_matrix[0][2] * point.z + tx;
// //         transformed_point.y = rotation_matrix[1][0] * point.x + rotation_matrix[1][1] * point.y + rotation_matrix[1][2] * point.z + ty;
// //         transformed_point.z = rotation_matrix[2][0] * point.x + rotation_matrix[2][1] * point.y + rotation_matrix[2][2] * point.z + tz;
// //         return transformed_point;
// //     }

// //     rclcpp::Subscription<lane_parameter_msg::msg::LaneMarkingProjectedArrayBoth>::SharedPtr lane_markings_subscription_;
// //     rclcpp::Publisher<lane_parameter_msg::msg::LaneMarkingProjectedArrayBoth>::SharedPtr lane_markings_publisher_;
// //     rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
// //     std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
// //     std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
// // };

// // int main(int argc, char *argv[]) {
// //     rclcpp::init(argc, argv);
// //     rclcpp::spin(std::make_shared<LaneTransformerNode>());
// //     rclcpp::shutdown();
// //     return 0;
// // }
