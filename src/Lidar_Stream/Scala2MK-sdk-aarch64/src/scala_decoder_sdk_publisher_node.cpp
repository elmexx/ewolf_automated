#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>

#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <scala_decoder_sdk/scala_decoder_parsing_sdk.h>
#include <scala_decoder_sdk/scala_point_conversion_sdk.h>
#include <scala_decoder_sdk/scala_socket_ethernet_sdk.h>

class ScalaSDKPublisher : public rclcpp::Node
{
public:
    ScalaSDKPublisher() : Node("scala_decoder_sdk_publisher")
    {
        try
        {
            parseParameters();

            mScalaSocketEthernet = std::make_shared<scala_decoder_sdk::ScalaSocketEthernet>(
                mArguments.type,
                mArguments.portHost,
                mArguments.ipHost,
                mArguments.ipMulticast);

            if (!mScalaSocketEthernet)
            {
                throw std::runtime_error("Could not allocate memory for Decoder Socket.");
            }

            std::thread socketThread(
                &scala_decoder_sdk::ScalaSocketEthernet::listenOnPort,
                mScalaSocketEthernet.get(),
                mArguments.portHost,
                mArguments.ipHost,
                0,
                nullptr,
                mArguments.ipMulticast);

            mSocketThread = std::move(socketThread);

            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            if (!mScalaSocketEthernet->isRunning())
            {
                throw std::runtime_error(
                    "ScalaSocketEthernet::listenOnPort is not running. Check IP configuration.");
            }

            mPublisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                mTopicName,
                rclcpp::SensorDataQoS());

            mTimer = this->create_wall_timer(
                std::chrono::milliseconds(10),
                std::bind(&ScalaSDKPublisher::timerCallback, this));

            RCLCPP_INFO(this->get_logger(),
                        "Started publisher. topic=%s frame_id=%s host_ip=%s multicast_ip=%s port=%u",
                        mTopicName.c_str(),
                        mFrameId.c_str(),
                        mArguments.ipHost.c_str(),
                        mArguments.ipMulticast.c_str(),
                        mArguments.portHost);
        }
        catch (const std::exception & error)
        {
            RCLCPP_ERROR(this->get_logger(), "%s", error.what());
        }
    }

    ~ScalaSDKPublisher()
    {
        try
        {
            if (mScalaSocketEthernet)
            {
                mScalaSocketEthernet->stopListening(0.5);
            }
            if (mSocketThread.joinable())
            {
                mSocketThread.join();
            }
        }
        catch (const std::exception & error)
        {
            RCLCPP_ERROR(this->get_logger(), "%s", error.what());
        }
    }

private:
    void timerCallback()
    {
        try
        {
            if (!mScalaSocketEthernet)
            {
                return;
            }

            auto package = mScalaSocketEthernet->getNewestCompleteScalaPackage(true);
            if (!package)
            {
                return;
            }

            auto pcl_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
            pcl_cloud->points.reserve(package->points.size());

            for (const auto & scanPoint : package->points)
            {
                scala_decoder_sdk::cartesianPoint pointCar =
                    scala_decoder_sdk::getCartesianPoint(scanPoint);

                pcl::PointXYZI point;
                point.x = pointCar.x;
                point.y = pointCar.y;
                point.z = pointCar.z;
                point.intensity = scanPoint.epw;

                pcl_cloud->points.push_back(point);
            }

            pcl_cloud->width = static_cast<std::uint32_t>(pcl_cloud->points.size());
            pcl_cloud->height = 1;
            pcl_cloud->is_dense = false;

            sensor_msgs::msg::PointCloud2 message;
            pcl::toROSMsg(*pcl_cloud, message);

            message.header.stamp = this->now();
            message.header.frame_id = mFrameId;

            mPublisher->publish(message);

            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Publishing cloud: topic=%s frame=%s points=%zu",
                mTopicName.c_str(),
                mFrameId.c_str(),
                pcl_cloud->points.size());
        }
        catch (const std::exception & error)
        {
            RCLCPP_ERROR_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "%s",
                error.what());
        }
    }

    void parseParameters()
    {
        scala_decoder_sdk::ListenOnPortArgs def{};

        this->declare_parameter("HostPort", static_cast<int32_t>(def.portHost));
        this->declare_parameter("HostIP", def.ipHost);
        this->declare_parameter("MulticastIP", def.ipMulticast);
        this->declare_parameter("PointCloudType", scala_decoder_sdk::typeToString(def.type));

        this->declare_parameter<std::string>("topic_name", "/scala_decoder_sdk_points");
        this->declare_parameter<std::string>("frame_id", "scala_decoder_sdk_lidar");

        rclcpp::Parameter port = this->get_parameter("HostPort");
        rclcpp::Parameter hostIP = this->get_parameter("HostIP");
        rclcpp::Parameter multicastIP = this->get_parameter("MulticastIP");
        rclcpp::Parameter type = this->get_parameter("PointCloudType");

        mTopicName = this->get_parameter("topic_name").as_string();
        mFrameId = this->get_parameter("frame_id").as_string();

        mArguments.portHost = static_cast<uint16_t>(port.as_int());
        mArguments.ipHost = hostIP.as_string();
        mArguments.ipMulticast = multicastIP.as_string();
        mArguments.type = scala_decoder_sdk::stringToType(type.as_string());

        if (!scala_decoder_sdk::isValidIP(mArguments.ipHost.c_str()))
        {
            throw std::runtime_error("HostIP is invalid: " + mArguments.ipHost);
        }

        if (("Unicast" != mArguments.ipMulticast) &&
            ("unicast" != mArguments.ipMulticast) &&
            !scala_decoder_sdk::isValidIP(mArguments.ipMulticast.c_str()))
        {
            throw std::runtime_error("MulticastIP is invalid: " + mArguments.ipMulticast);
        }

        if (("Unicast" == mArguments.ipMulticast) ||
            ("unicast" == mArguments.ipMulticast))
        {
            mArguments.ipMulticast.clear();
        }

        RCLCPP_INFO(this->get_logger(),
                    "Parameters: HostIP=%s HostPort=%u MulticastIP=%s topic_name=%s frame_id=%s type=%s",
                    mArguments.ipHost.c_str(),
                    mArguments.portHost,
                    mArguments.ipMulticast.c_str(),
                    mTopicName.c_str(),
                    mFrameId.c_str(),
                    scala_decoder_sdk::typeToString(mArguments.type).c_str());
    }

private:
    rclcpp::TimerBase::SharedPtr mTimer;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr mPublisher;

    scala_decoder_sdk::ListenOnPortArgs mArguments;
    std::shared_ptr<scala_decoder_sdk::ScalaSocketEthernet> mScalaSocketEthernet;
    std::thread mSocketThread;

    std::string mTopicName;
    std::string mFrameId;
};

int main(int argc, char * argv[])
{
    try
    {
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<ScalaSDKPublisher>());
        rclcpp::shutdown();
    }
    catch (const std::exception & error)
    {
        std::cerr << error.what() << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}