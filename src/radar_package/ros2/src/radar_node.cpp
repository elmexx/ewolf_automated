#include <cmath>
#include <cstring>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include "mcr1.h"

namespace mk = mobilitykit::radar;

class Radar : public rclcpp::Node {
   public:
    Radar() : Node("radar_node") {
        std::string socketName = declare_parameter("canbus_name", "can0");
        uint8_t locationId = declare_parameter("sensor_location", 0);

        mSensorData = std::make_shared<mk::MCR1::Data>();
        mSensor = std::make_shared<mk::MCR1>(socketName, locationId, true);

        mPositionPublisher =
            create_publisher<geometry_msgs::msg::PoseStamped>("radar_position", 10);

        mInfraDetectionPublisher =
            create_publisher<sensor_msgs::msg::PointCloud2>("radar_detection_pcl_infra", 10);

        mNonInfraDetectionPublisher =
            create_publisher<sensor_msgs::msg::PointCloud2>("radar_detection_pcl_non_infra", 10);

        mTimer = create_wall_timer(
            std::chrono::milliseconds(1),
            std::bind(&Radar::publish_latest, this));
    }

    ~Radar() = default;

   private:
    std::shared_ptr<mk::MCR1> mSensor;
    std::shared_ptr<mk::MCR1::Data> mSensorData;
    rclcpp::TimerBase::SharedPtr mTimer;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr mPositionPublisher;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr mInfraDetectionPublisher;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr mNonInfraDetectionPublisher;

    struct pcl2point {
        float x;            // longitudinal position in radar frame [m]
        float y;            // lateral position in radar frame [m]
        float z;            // height, currently 0 [m]
        float doppler;      // radial relative velocity [m/s], NaN for infra detections
        float range;        // CoGRange [m]
        float azimuth;      // azimuth angle [deg]
        float azimuth_std;  // standard deviation of azimuth [deg]
    };

    static constexpr const char* FRAME_ID = "radar_detections";

    float degreeToRad(float angleInDegrees) const {
        return angleInDegrees * static_cast<float>(M_PI / 180.0);
    }

    pcl2point detectionValuesToPoint(
        float range,
        float azimuth,
        float azimuthStd,
        float doppler) const
    {
        pcl2point p{};
        p.x = std::cos(degreeToRad(azimuth)) * range;
        p.y = std::sin(degreeToRad(azimuth)) * range;
        p.z = 0.0f;
        p.doppler = doppler;
        p.range = range;
        p.azimuth = azimuth;
        p.azimuth_std = azimuthStd;
        return p;
    }

    geometry_msgs::msg::PoseStamped getPose(
        const std::shared_ptr<mk::MCR1::Data>& data) 
    {
        geometry_msgs::msg::PoseStamped position;
        position.header.frame_id = FRAME_ID;
        position.header.stamp = get_clock()->now();

        position.pose.position.x = data->sensorPosition.xOffsetMeters;
        position.pose.position.y = data->sensorPosition.yOffsetMeters;
        position.pose.position.z = data->sensorPosition.zOffsetMeters;

        const float yawRad = degreeToRad(data->sensorPosition.azimuthAngleDegrees);
        position.pose.orientation.w = std::cos(yawRad / 2.0f);
        position.pose.orientation.x = 0.0;
        position.pose.orientation.y = 0.0;
        position.pose.orientation.z = std::sin(yawRad / 2.0f);

        return position;
    }

    sensor_msgs::msg::PointCloud2 generateMessage(
        const std::vector<pcl2point>& points) 
    {
        sensor_msgs::msg::PointCloud2 cloud;

        cloud.header.frame_id = FRAME_ID;
        cloud.header.stamp = get_clock()->now();

        cloud.height = 1;
        cloud.width = static_cast<uint32_t>(points.size());

        cloud.is_bigendian = false;
        cloud.is_dense = true;

        cloud.point_step = sizeof(pcl2point);
        cloud.row_step = cloud.width * cloud.point_step;

        cloud.fields.resize(7);

        cloud.fields[0].name = "x";
        cloud.fields[0].offset = offsetof(pcl2point, x);
        cloud.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
        cloud.fields[0].count = 1;

        cloud.fields[1].name = "y";
        cloud.fields[1].offset = offsetof(pcl2point, y);
        cloud.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
        cloud.fields[1].count = 1;

        cloud.fields[2].name = "z";
        cloud.fields[2].offset = offsetof(pcl2point, z);
        cloud.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
        cloud.fields[2].count = 1;

        cloud.fields[3].name = "doppler";
        cloud.fields[3].offset = offsetof(pcl2point, doppler);
        cloud.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
        cloud.fields[3].count = 1;

        cloud.fields[4].name = "range";
        cloud.fields[4].offset = offsetof(pcl2point, range);
        cloud.fields[4].datatype = sensor_msgs::msg::PointField::FLOAT32;
        cloud.fields[4].count = 1;

        cloud.fields[5].name = "azimuth";
        cloud.fields[5].offset = offsetof(pcl2point, azimuth);
        cloud.fields[5].datatype = sensor_msgs::msg::PointField::FLOAT32;
        cloud.fields[5].count = 1;

        cloud.fields[6].name = "azimuth_std";
        cloud.fields[6].offset = offsetof(pcl2point, azimuth_std);
        cloud.fields[6].datatype = sensor_msgs::msg::PointField::FLOAT32;
        cloud.fields[6].count = 1;

        cloud.data.resize(points.size() * sizeof(pcl2point));
        if (!points.empty()) {
            std::memcpy(cloud.data.data(), points.data(), points.size() * sizeof(pcl2point));
        }

        return cloud;
    }

    void publish_latest() {
        if (!mSensor->run(mSensorData)) {
            return;
        }

        mPositionPublisher->publish(getPose(mSensorData));

        {
            std::vector<pcl2point> detectedPoints;
            detectedPoints.reserve(mSensorData->infraDetections.size());

            const float nanValue = std::numeric_limits<float>::quiet_NaN();

            for (const mk::MCR1::Infrastructure& detection : mSensorData->infraDetections) {
                detectedPoints.push_back(
                    detectionValuesToPoint(
                        detection.CoGRange,
                        detection.Azimuth,
                        detection.StandardDeviationAzimuth,
                        nanValue));
            }

            if (!detectedPoints.empty()) {
                mInfraDetectionPublisher->publish(generateMessage(detectedPoints));
            }
        }

        {
            std::vector<pcl2point> detectedPoints;
            detectedPoints.reserve(mSensorData->nonInfraDetections.size());

            for (const mk::MCR1::NonInfrastructure& detection : mSensorData->nonInfraDetections) {
                detectedPoints.push_back(
                    detectionValuesToPoint(
                        detection.CoGRange,
                        detection.Azimuth,
                        detection.StandardDeviationAzimuth,
                        detection.Doppler));
            }

            if (!detectedPoints.empty()) {
                mNonInfraDetectionPublisher->publish(generateMessage(detectedPoints));
            }
        }
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Radar>());
    rclcpp::shutdown();
    return 0;
}