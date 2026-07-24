/**
 * @copyright(c) 2023 This program is the confidential and proprietary product of Valeo
 *                    Schalter und Sensoren GmbH (DSW).
 *                    All rights reserved.
 *
 *                    VALEO Schalter und Sensoren GmbH (DSW) will
 *                    take no responsibility for any improper behavior of the software. In case of
 *                    equipping test vehicles with the sensor kit, VALEO Schalter und Sensoren
 *                    GmbH takes no liability on the behavior of the test vehicles equipped with
 *                    the sensor kit or any damage caused within or outside to material and
 *                    people.
 *
 * @author David Peter <david.peter@valeo.com>
 *
 * @date May 2023
 *
 */

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "mcr1.h"

namespace mk = mobilitykit::radar;

class Radar : public rclcpp::Node {
   public:
    Radar() : Node("radar_node") {
        std::string socketName = declare_parameter("canbus_name", "can0");
        uint8_t locationId = declare_parameter("sensor_location", 0);
        mSensorData = std::make_shared<mk::MCR1::Data>();
        mSensor = std::make_shared<mk::MCR1>(socketName, locationId, true);
        mPositionPublisher = create_publisher<geometry_msgs::msg::PoseStamped>("radar_position", 10);
        mInfraDetectionPublisher = create_publisher<sensor_msgs::msg::PointCloud2>("radar_detection_pcl_infra", 10);
        mNonInfraDetectionPublisher =
            create_publisher<sensor_msgs::msg::PointCloud2>("radar_detection_pcl_non_infra", 10);
        mTimer = create_wall_timer(std::chrono::milliseconds(1), std::bind(&Radar::publish_latest, this));
    }

    ~Radar(){};

   private:
    std::shared_ptr<mk::MCR1> mSensor;
    std::shared_ptr<mk::MCR1::Data> mSensorData;
    rclcpp::TimerBase::SharedPtr mTimer;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr mPositionPublisher;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr mInfraDetectionPublisher;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr mNonInfraDetectionPublisher;

    struct pcl2point {
        float x;
        float y;
        float z;
        float doppler;
    };

    float degreeToRad(float angleInDegrees) { return angleInDegrees * (M_PI / 180.f); }

    pcl2point detectionValuesToPoint(float distance, float azimuth, float doppler = 0.) {
        pcl2point p;
        p.x = cosf(degreeToRad(azimuth)) * distance;
        p.y = sinf(degreeToRad(azimuth)) * distance;
        p.z = 0.f;
        p.doppler = doppler;
        return p;
    }

    geometry_msgs::msg::PoseStamped getPose(std::shared_ptr<mk::MCR1::Data> data) {
        geometry_msgs::msg::PoseStamped position;
        position.header.frame_id = "radar_detections";
        position.header.stamp = get_clock()->now();
        position.pose.position.x = data->sensorPosition.xOffsetMeters;
        position.pose.position.y = data->sensorPosition.yOffsetMeters;
        position.pose.position.z = data->sensorPosition.zOffsetMeters;
        position.pose.orientation.w = cos((degreeToRad(data->sensorPosition.azimuthAngleDegrees)) / 2);
        position.pose.orientation.x = 0.;
        position.pose.orientation.y = 0.;
        position.pose.orientation.z = sin((degreeToRad(data->sensorPosition.azimuthAngleDegrees)) / 2);
        return position;
    }

    sensor_msgs::msg::PointCloud2 generateMessage(std::vector<pcl2point> points) {
        sensor_msgs::msg::PointCloud2 cloud;
        cloud.is_bigendian = false;
        cloud.is_dense = true;
        cloud.header.frame_id = "radar_detections";
        cloud.header.stamp = get_clock()->now();
        cloud.height = 1;
        cloud.width = points.size();
        cloud.point_step = sizeof(pcl2point);
        // cloud.row_step = cloud.width;
        cloud.row_step = cloud.point_step * cloud.width;

        // cloud.fields.resize(points.size() * 4);
        cloud.fields.resize(4);
        cloud.fields[0].name = "x";
        cloud.fields[0].offset = 0;
        cloud.fields[1].name = "y";
        cloud.fields[1].offset = 4;
        cloud.fields[2].name = "z";
        cloud.fields[2].offset = 8;
        cloud.fields[3].name = "doppler";
        cloud.fields[3].offset = 12;
        for (auto& field : cloud.fields) {
            field.count = 1;
            field.datatype = sensor_msgs::msg::PointField::FLOAT32;
        }
        cloud.data.resize(points.size() * sizeof(pcl2point));
        memmove(cloud.data.data(), points.data(), points.size() * sizeof(pcl2point));

        return cloud;
    }

    void publish_latest() {
        if (!mSensor->run(mSensorData)) {
            return;
        }

        mPositionPublisher->publish(getPose(mSensorData));

        std::vector<pcl2point> detectedPoints;
        for (const mk::MCR1::Infrastructure& detection : mSensorData->infraDetections) {
            detectedPoints.push_back(detectionValuesToPoint(detection.CoGRange, detection.Azimuth));
        }
        if (!detectedPoints.empty()) {
            mInfraDetectionPublisher->publish(generateMessage(detectedPoints));
        }
        detectedPoints.clear();

        for (const mk::MCR1::NonInfrastructure& detection : mSensorData->nonInfraDetections) {
            detectedPoints.push_back(detectionValuesToPoint(detection.CoGRange, detection.Azimuth, detection.Doppler));
        }
        if (!detectedPoints.empty()) {
            mNonInfraDetectionPublisher->publish(generateMessage(detectedPoints));
        }
        detectedPoints.clear();
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Radar>());
    rclcpp::shutdown();
    return 0;
}
