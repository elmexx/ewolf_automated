#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "vehicle_dynamic_msg/msg/vehicle_dynamic.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <cmath>

class VehicleDynamicToOdometry : public rclcpp::Node
{
public:
    VehicleDynamicToOdometry()
        : Node("vehicle_dynamic_to_odometry"),
          last_time_(0, 0, RCL_ROS_TIME)
    {
        subscription_ = this->create_subscription<vehicle_dynamic_msg::msg::VehicleDynamic>(
            "/vehicle_dynamic_data", 10,
            std::bind(&VehicleDynamicToOdometry::callback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odometry/vehicle", 10);

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

private:
    void callback(const vehicle_dynamic_msg::msg::VehicleDynamic::SharedPtr msg)
    {

        rclcpp::Time current_time = msg->header.stamp;

        double dt = 0.0;
        if (last_time_.nanoseconds() > 0)
        {
            dt = (current_time - last_time_).seconds();
        }
        last_time_ = current_time;

        if (dt <= 0.0)
        {
            RCLCPP_WARN(this->get_logger(), "Invalid dt detected, skipping this callback.");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Received Stamp: sec=%d, nanosec=%d", 
                    msg->header.stamp.sec, msg->header.stamp.nanosec);

        double wheel_speed_fl = msg->wheel_speed_fl / 3.6;
        double wheel_speed_fr = msg->wheel_speed_fr / 3.6;
        double wheel_speed_rl = msg->wheel_speed_rl / 3.6;
        double wheel_speed_rr = msg->wheel_speed_rr / 3.6;

        double car_speed = (wheel_speed_fl + wheel_speed_fr + wheel_speed_rl + wheel_speed_rr) / 4.0;

        double front_wheel_track = 1.490; 
        double rear_wheel_track = 1.490; 

        double yaw_rate_front = (wheel_speed_fr - wheel_speed_fl) / front_wheel_track;
        double yaw_rate_rear = (wheel_speed_rr - wheel_speed_rl) / rear_wheel_track;
        double yaw_rate_calculated = (yaw_rate_front + yaw_rate_rear) / 2.0;

        auto odometry_msg = nav_msgs::msg::Odometry();
        odometry_msg.header.stamp = msg->header.stamp;
        odometry_msg.header.frame_id = "odom_vehicle";
        odometry_msg.child_frame_id = "base_link_vehicle";

        static double x = 0.0, y = 0.0, theta = 0.0;

        x += car_speed * dt * cos(theta);
        y += car_speed * dt * sin(theta);

        theta += yaw_rate_calculated * dt;

        odometry_msg.pose.pose.position.x = x;
        odometry_msg.pose.pose.position.y = y;
        odometry_msg.pose.pose.position.z = 0.0;

        odometry_msg.pose.pose.orientation.z = sin(theta / 2.0);
        odometry_msg.pose.pose.orientation.w = cos(theta / 2.0);

        odometry_msg.twist.twist.linear.x = car_speed; 
        odometry_msg.twist.twist.angular.z = yaw_rate_calculated; 

        set_covariance(odometry_msg.pose.covariance, {0.1, 0, 0, 0, 0, 0,
                                                      0, 0.1, 0, 0, 0, 0,
                                                      0, 0, 1.0, 0, 0, 0,
                                                      0, 0, 0, 1.0, 0, 0,
                                                      0, 0, 0, 0, 1.0, 0,
                                                      0, 0, 0, 0, 0, 0.1});

        set_covariance(odometry_msg.twist.covariance, {0.1, 0, 0, 0, 0, 0,
                                                       0, 0.1, 0, 0, 0, 0,
                                                       0, 0, 1.0, 0, 0, 0,
                                                       0, 0, 0, 1.0, 0, 0,
                                                       0, 0, 0, 0, 1.0, 0,
                                                       0, 0, 0, 0, 0, 0.05});

        publisher_->publish(odometry_msg);

        publish_tf(msg->header.stamp, x, y, theta);
    }

    void set_covariance(std::array<double, 36> &covariance, const std::vector<double> &values)
    {
        std::copy(values.begin(), values.end(), covariance.begin());
    }

    void publish_tf(const rclcpp::Time &stamp, double x, double y, double theta)
    {

        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = stamp;
        transform.header.frame_id = "odom_vehicle";
        transform.child_frame_id = "base_link_vehicle";

        transform.transform.translation.x = x;
        transform.transform.translation.y = y;
        transform.transform.translation.z = 0.0;

        transform.transform.rotation.x = 0.0;
        transform.transform.rotation.y = 0.0;
        transform.transform.rotation.z = sin(theta / 2.0);
        transform.transform.rotation.w = cos(theta / 2.0);

        tf_broadcaster_->sendTransform(transform);
    }

    rclcpp::Subscription<vehicle_dynamic_msg::msg::VehicleDynamic>::SharedPtr subscription_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_; 

    rclcpp::Time last_time_; 
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VehicleDynamicToOdometry>());
    rclcpp::shutdown();
    return 0;
}
