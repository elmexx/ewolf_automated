#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cmath>
#include <deque>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class GpsImuFusion : public rclcpp::Node
{
public:
    GpsImuFusion()
        : Node("gps_imu_fusion_node"),
          ref_lat_(0.0), ref_lon_(0.0), ref_set_(false),
          last_x_(0.0), last_y_(0.0), initial_theta_(0.0), theta_(0.0),
          gps_weight_(0.0), imu_weight_(1.0), last_imu_time_(0.0)
    {
        this->declare_parameter("gps_weight", gps_weight_);
        this->declare_parameter("imu_weight", imu_weight_);

        gps_sub_.subscribe(this, "/fix");
        imu_sub_.subscribe(this, "/imu_enu");

        sync_ = std::make_shared<Sync>(SyncPolicy(10), gps_sub_, imu_sub_);
        sync_->registerCallback(std::bind(&GpsImuFusion::gps_imu_callback, this, std::placeholders::_1, std::placeholders::_2));

        odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odometry/gps", 10);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

private:
    void gps_imu_callback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr gps_msg,
                          const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg)
    {

        this->get_parameter("gps_weight", gps_weight_);
        this->get_parameter("imu_weight", imu_weight_);

        if (gps_msg->status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX)
        {
            RCLCPP_WARN(this->get_logger(), "No valid GPS fix.");
            return;
        }

        if (!ref_set_)
        {
            ref_lat_ = gps_msg->latitude;
            ref_lon_ = gps_msg->longitude;
            ref_alt_ = gps_msg->altitude;
            ref_set_ = true;
            RCLCPP_INFO(this->get_logger(), "Reference point set: lat=%f, lon=%f, alt=%f", ref_lat_, ref_lon_, ref_alt_);
            return;
        }

        double x, y, z;
        gps_to_local(gps_msg->latitude, gps_msg->longitude, gps_msg->altitude, x, y, z);

        // if (initial_points_.size() < max_initial_points_)
        // {
        //     initial_points_.emplace_back(x, y);
        //     if (initial_points_.size() == max_initial_points_)
        //     {
        //         calculate_initial_theta();
        //         RCLCPP_INFO(this->get_logger(), "Initial theta calculated: %f radians", initial_theta_);
        //     }
        //     return;
        // }

        double dx = x - last_x_;
        double dy = y - last_y_;
        // double gps_theta = atan2(dy, dx);

        double imu_dt = 0.0;
        // if (last_imu_time_.nanoseconds() > 0)
        // {
        //     rclcpp::Time current_imu_time(imu_msg->header.stamp); 
        //     imu_dt = (current_imu_time.nanoseconds() - last_imu_time_.nanoseconds()) / 1e9;
        // }
        // last_imu_time_ = rclcpp::Time(imu_msg->header.stamp);

        rclcpp::Time current_imu_time(imu_msg->header.stamp);
        if (last_imu_time_.nanoseconds() > 0)
        {
            rclcpp::Duration duration = current_imu_time - last_imu_time_;
            imu_dt = duration.nanoseconds() / 1e9; 
        }
        last_imu_time_ = current_imu_time;

        if (imu_dt <= 0.0)
        {
            RCLCPP_WARN(this->get_logger(), "Invalid IMU time %f interval detected, skipping IMU update.", imu_dt);
            return;
        }

        double imu_theta = theta_ + imu_msg->angular_velocity.z * imu_dt;

        // double fused_theta = gps_weight_ * gps_theta + imu_weight_ * imu_theta;

        last_x_ = x;
        last_y_ = y;
        // double corrected_x = x * cos(-imu_theta) - y * sin(-imu_theta);
        // double corrected_y = x * sin(-imu_theta) + y * cos(-imu_theta);

        auto odom_msg = nav_msgs::msg::Odometry();
        odom_msg.header.stamp = gps_msg->header.stamp;
        odom_msg.header.frame_id = "odom_gps";
        odom_msg.child_frame_id = "base_link_gps";

        odom_msg.pose.pose.position.x = x;
        odom_msg.pose.pose.position.y = y;
        odom_msg.pose.pose.position.z = z;

        tf2::Quaternion quaternion;
        quaternion.setRPY(0, 0, imu_theta);
        odom_msg.pose.pose.orientation = tf2::toMsg(quaternion);

        const auto &cov = gps_msg->position_covariance;

        set_covariance(odom_msg.pose.covariance, {cov[0], cov[1], cov[2], 0, 0, 0,
                                                  cov[3], cov[4], cov[5], 0, 0, 0,
                                                  cov[6], cov[7], cov[8], 0, 0, 0,
                                                  0, 0, 0, 100.0, 0, 0,
                                                  0, 0, 0, 0, 100.0, 0,
                                                  0, 0, 0, 0, 0, 100.0});

        odometry_publisher_->publish(odom_msg);

        publish_dynamic_transform(gps_msg->header.stamp, x, y, z, imu_theta);
    }

    void calculate_initial_theta()
    {
        if (initial_points_.size() < 2)
        {
            RCLCPP_WARN(this->get_logger(), "Not enough points to calculate initial theta.");
            return;
        }
        auto front_point = initial_points_.front();
        auto x1 = front_point.first;
        auto y1 = front_point.second;

        auto back_point = initial_points_.back();
        auto x2 = back_point.first;
        auto y2 = back_point.second;

        initial_theta_ = atan2(y2 - y1, x2 - x1);
    }

    void gps_to_local(double lat, double lon, double alt, double &x, double &y, double &z)
    {
        const double earth_radius = 6378137.0;

        double lat_rad = lat * M_PI / 180.0;
        double lon_rad = lon * M_PI / 180.0;
        double ref_lat_rad = ref_lat_ * M_PI / 180.0;
        double ref_lon_rad = ref_lon_ * M_PI / 180.0;

        double delta_lat = lat_rad - ref_lat_rad;
        double delta_lon = lon_rad - ref_lon_rad;

        double radius_at_lat = earth_radius * cos(ref_lat_rad);

        x = radius_at_lat * delta_lon;
        y = earth_radius * delta_lat;
        z = alt - ref_alt_;
    }

    void set_covariance(std::array<double, 36> &covariance, const std::vector<double> &values)
    {
        std::copy(values.begin(), values.end(), covariance.begin());
    }

    void publish_dynamic_transform(const rclcpp::Time &stamp, double x, double y, double z, double theta)
    {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = stamp;
        transform.header.frame_id = "odom_gps";
        transform.child_frame_id = "base_link_gps";

        transform.transform.translation.x = x;
        transform.transform.translation.y = y;
        transform.transform.translation.z = z;

        tf2::Quaternion quaternion;
        quaternion.setRPY(0, 0, theta);
        transform.transform.rotation = tf2::toMsg(quaternion);

        tf_broadcaster_->sendTransform(transform);
    }

    message_filters::Subscriber<sensor_msgs::msg::NavSatFix> gps_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Imu> imu_sub_;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::NavSatFix, sensor_msgs::msg::Imu> SyncPolicy;
    typedef message_filters::Synchronizer<SyncPolicy> Sync;
    std::shared_ptr<Sync> sync_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    double ref_lat_, ref_lon_, ref_alt_;
    bool ref_set_;
    double last_x_, last_y_, initial_theta_, theta_;

    double gps_weight_, imu_weight_;
    rclcpp::Time last_imu_time_;
    std::deque<std::pair<double, double>> initial_points_;
    const size_t max_initial_points_ = 10;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GpsImuFusion>());
    rclcpp::shutdown();
    return 0;
}

// #include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/nav_sat_fix.hpp>
// #include <nav_msgs/msg/odometry.hpp>
// #include <tf2_ros/transform_broadcaster.h>
// #include <geometry_msgs/msg/transform_stamped.hpp>
// #include <cmath>
// #include <deque>
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// class GpsToOdometry : public rclcpp::Node
// {
// public:
//     GpsToOdometry()
//         : Node("gps_to_odometry_node"),
//           ref_lat_(0.0), ref_lon_(0.0), ref_set_(false),
//           last_x_(0.0), last_y_(0.0), initial_theta_(0.0),
//           cumulative_distance_(0.0)
//     {
//         gps_subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
//             "/fix", 10, std::bind(&GpsToOdometry::gps_callback, this, std::placeholders::_1));
//         odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odometry/gps", 10);
//         tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
//     }

// private:
//     void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
//     {
//         if (msg->status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX)
//         {
//             RCLCPP_WARN(this->get_logger(), "No valid GPS fix.");
//             return;
//         }

//         if (!ref_set_)
//         {
//             ref_lat_ = msg->latitude;
//             ref_lon_ = msg->longitude;
//             ref_alt_ = msg->altitude;
//             ref_set_ = true;
//             RCLCPP_INFO(this->get_logger(), "Reference point set: lat=%f, lon=%f, alt=%f", ref_lat_, ref_lon_, ref_alt_);
//             return;
//         }

//         // RCLCPP_INFO(this->get_logger(), "Received Stamp: sec=%d, nanosec=%d", 
//         //         msg->header.stamp.sec, msg->header.stamp.nanosec);

//         double x, y, z;
//         gps_to_local(msg->latitude, msg->longitude, msg->altitude, x, y, z);

//         if (!initial_points_.empty())
//         {
//             double dx = x - initial_points_.back().first;
//             double dy = y - initial_points_.back().second;

//             if (sqrt(dx * dx + dy * dy) > min_point_distance_)
//             {
//                 cumulative_distance_ += sqrt(dx * dx + dy * dy);
//                 initial_points_.emplace_back(x, y);
//             }
//         }
//         else
//         {
//             initial_points_.emplace_back(x, y);
//         }

//         if (cumulative_distance_ >= min_distance_threshold_ && initial_theta_ == 0.0)
//         {
//             calculate_initial_theta();
//             RCLCPP_INFO(this->get_logger(), "Initial theta calculated: %f radians", initial_theta_);
//         }

//         double dx = x - last_x_;
//         double dy = y - last_y_;
//         double dynamic_theta = atan2(dy, dx);
//         RCLCPP_INFO(this->get_logger(), "%f, %f, %f, %f", x, y, last_x_, last_y_);

//         double corrected_x = x * cos(-initial_theta_) - y * sin(-initial_theta_);
//         double corrected_y = x * sin(-initial_theta_) + y * cos(-initial_theta_);
//         dynamic_theta -= initial_theta_;
        
//         last_x_ = x;
//         last_y_ = y;

//         auto odom_msg = nav_msgs::msg::Odometry();
//         odom_msg.header.stamp = msg->header.stamp;
//         odom_msg.header.frame_id = "odom_gps";
//         odom_msg.child_frame_id = "base_link_gps";

//         odom_msg.pose.pose.position.x = corrected_x;
//         odom_msg.pose.pose.position.y = corrected_y;
//         odom_msg.pose.pose.position.z = z;

//         tf2::Quaternion quaternion;
//         quaternion.setRPY(0, 0, dynamic_theta);
//         odom_msg.pose.pose.orientation = tf2::toMsg(quaternion);

//         const auto& cov = msg->position_covariance;

//         set_covariance(odom_msg.pose.covariance, {cov[0], cov[1], cov[2], 0, 0, 0,
//                                                   cov[3], cov[4], cov[5], 0, 0, 0,
//                                                   cov[6], cov[7], cov[8], 0, 0, 0,
//                                                   0, 0, 0, 100.0, 0, 0,
//                                                   0, 0, 0, 0, 100.0, 0,
//                                                   0, 0, 0, 0, 0, 100.0});

//         odometry_publisher_->publish(odom_msg);
//         publish_dynamic_transform(msg->header.stamp, corrected_x, corrected_y, z, dynamic_theta);
//         //publish_dynamic_transform(msg->header.stamp, x, y, z, dynamic_theta);
//     }

//     void set_covariance(std::array<double, 36> &covariance, const std::vector<double> &values)
//     {
//         std::copy(values.begin(), values.end(), covariance.begin());
//     }

//     void calculate_initial_theta()
//     {
//         if (initial_points_.size() < 2)
//         {
//             RCLCPP_WARN(this->get_logger(), "Not enough points to calculate initial theta.");
//             return;
//         }

//         auto front_point = initial_points_.front();
//         auto x1 = front_point.first;
//         auto y1 = front_point.second;

//         auto back_point = initial_points_.back();
//         auto x2 = back_point.first;
//         auto y2 = back_point.second;

//         initial_theta_ = atan2(y2 - y1, x2 - x1);
//     }

//     void gps_to_local(double lat, double lon, double alt, double &x, double &y, double &z)
//     {
//         const double earth_radius = 6378137.0; 

//         double lat_rad = lat * M_PI / 180.0;
//         double lon_rad = lon * M_PI / 180.0;
//         double ref_lat_rad = ref_lat_ * M_PI / 180.0;
//         double ref_lon_rad = ref_lon_ * M_PI / 180.0;

//         double delta_lat = lat_rad - ref_lat_rad;
//         double delta_lon = lon_rad - ref_lon_rad;

//         double radius_at_lat = earth_radius * cos(ref_lat_rad);

//         x = radius_at_lat * delta_lon;   
//         y = earth_radius * delta_lat;   

//         z = alt - ref_alt_;            
//     }

//     void publish_dynamic_transform(const rclcpp::Time &stamp, double x, double y, double z, double theta)
//     {
//         geometry_msgs::msg::TransformStamped transform;
//         transform.header.stamp = stamp;
//         transform.header.frame_id = "odom_gps";
//         transform.child_frame_id = "base_link_gps";

//         transform.transform.translation.x = x;
//         transform.transform.translation.y = y;
//         transform.transform.translation.z = z;

//         tf2::Quaternion quaternion;
//         quaternion.setRPY(0, 0, theta); 
//         transform.transform.rotation = tf2::toMsg(quaternion);

//         tf_broadcaster_->sendTransform(transform);
//     }

//     rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_subscription_;
//     rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
//     std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

//     double ref_lat_, ref_lon_, ref_alt_; 
//     bool ref_set_;                       
//     double last_x_, last_y_, initial_theta_;
//     double cumulative_distance_;

//     std::deque<std::pair<double, double>> initial_points_;
//     const size_t max_initial_points_ = 10; // Use 10 points for initial theta calculation
//     const double min_distance_threshold_ = 10.0; 
//     const double min_point_distance_ = 1.0; 
// };

// int main(int argc, char *argv[])
// {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<GpsToOdometry>());
//     rclcpp::shutdown();
//     return 0;
// }
