/**
 * @copyright(c) 2020 This program is the confidential and proprietary product of Valeo
 *                    Schalter und Sensoren GmbH (Driving Assistance Research).
 *                    All rights reserved.
 *
 *                    VALEO Schalter und Sensoren GmbH (Driving Assistance Research) will
 *                    take no responsibility for any improper behavior of the software. In case of
 *                    equipping test vehicles with the sensor kit, VALEO Schalter und Sensoren
 *                    GmbH takes no liability on the behavior of the test vehicles equipped with
 *                    the sensor kit or any damage caused within or outside to material and
 *                    people.
 *
 * @brief ROS2 node to test source code with connected Scala1 or Scala2
 *
 * @author Patrick Reichel <patrick.reichel@valeo.com>
 *
 * @date September 2021
 *
 * @file
 */

#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <sstream> // std::stringstream
#include <string>

#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <scala_decoder_sdk/scala_decoder_parsing_sdk.h>
#include <scala_decoder_sdk/scala_point_conversion_sdk.h>
#include <scala_decoder_sdk/scala_socket_ethernet_sdk.h>

class ScalaSDKPublisher : public rclcpp::Node {
public:
    /**
     * @brief initialize needed objects
     */
    ScalaSDKPublisher() : Node("scala_decoder_sdk_publisher_3") {
        try {
            parseParameters();
            mScalaSocketEthernet = std::make_shared<scala_decoder_sdk::ScalaSocketEthernet>(mArguments.type,
                                                                                            mArguments.portHost,
                                                                                            mArguments.ipHost,
                                                                                            mArguments.ipMulticast);
            if (!mScalaSocketEthernet) {
                throw std::runtime_error("\nERROR: Could not allocate memory for Decoder Socket!\n");
            }
            // Start socket thread with ethernet configuration
            std::thread socketThread(&scala_decoder_sdk::ScalaSocketEthernet::listenOnPort,
                                     mScalaSocketEthernet.get(),
                                     mArguments.portHost,
                                     mArguments.ipHost,
                                     0,
                                     nullptr,
                                     mArguments.ipMulticast);
            mSocketThread = std::move(socketThread);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            if (!mScalaSocketEthernet->isRunning()) {
                throw std::runtime_error("\nERROR::ScalaSocketEthernet::listenOnPort is not running, "
                                         "check ip configuration.\n");
            }
            mPublisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("/scala_decoder_sdk_points_3", 25);
            // Scala Lidar has frame rate of 25 hz -> Use 10 ms here to be sure, that buffer is checked often enough
            // for a new package to reduce the occurence of wrong sorted scans
            mTimer = this->create_wall_timer(std::chrono::milliseconds(10),
                                             std::bind(&ScalaSDKPublisher::timer_callback, this));
        } catch (const std::runtime_error& error) {
            RCLCPP_ERROR_ONCE(this->get_logger(), error.what());
        }
    }

    /**
     * @brief reset needed objects
     */
    ~ScalaSDKPublisher() {
        // Stop the thread
        try {
            if (mScalaSocketEthernet) {
                mScalaSocketEthernet->stopListening(0.5);
            }
            if (mSocketThread.joinable()) {
                mSocketThread.join();
            }
        } catch (const std::runtime_error& error) {
            RCLCPP_ERROR_ONCE(this->get_logger(), error.what());
        }
    }

private:
    /**
     * @brief timerCallback function for publish messages
     */
    void timer_callback() {
        try {
            if (!mScalaSocketEthernet) {
                return;
            }
            auto package = mScalaSocketEthernet->getNewestCompleteScalaPackage(true);
            if (package) {
                std::stringstream logMsg;
                logMsg << "\nScan number:                  " << package->scanNumber <<
                          "\nScan timestamp:               " << package->scanTime <<
                          "\nNumber of Points in the scan: " << package->points.size() << std::endl;
                RCLCPP_INFO(this->get_logger(), logMsg.str().c_str());
                // Note: Using library functions, the Scala 'package' will now be copied into a pcl::PointXYZI,
                // only to be converted again into a ROS 'Pointcloud2'. This is safe but sub-optimal concerning runtime.
                // Feel free to do your own pointer arithmetic to create Pointcloud2 directly.
                pcl::PointCloud<pcl::PointXYZI>::Ptr pclVecFrame =
                        boost::make_shared<pcl::PointCloud<pcl::PointXYZI> >();
                if (pclVecFrame) {
                    for (const auto scanPoint : package->points) {
                        scala_decoder_sdk::cartesianPoint pointCar = scala_decoder_sdk::getCartesianPoint(scanPoint);
                        pcl::PointXYZI point;
                        point.x = pointCar.x; // x
                        point.y = pointCar.y; // y
                        point.z = pointCar.z; // z
                        point.intensity = scanPoint.epw; // epw
                        pclVecFrame->push_back(point);
                    }
                    pclVecFrame->header.frame_id = "scala_decoder_sdk_lidar";
                    auto message = sensor_msgs::msg::PointCloud2();
                    pcl::toROSMsg(*pclVecFrame, message);
                    mPublisher->publish(message);
                }
            }
        } catch (const std::runtime_error& error) {
            RCLCPP_ERROR(this->get_logger(), error.what());
        }
    }

    /**
     * @brief parseParameters declare and get ROS2 parameters
     */
    void parseParameters() {
        scala_decoder_sdk::ListenOnPortArgs def{};
        this->declare_parameter("HostPort", static_cast<int32_t>(def.portHost));
        this->declare_parameter("HostIP", def.ipHost);
        this->declare_parameter("MulticastIP", def.ipMulticast);
        this->declare_parameter("PointCloudType", scala_decoder_sdk::typeToString(def.type));
        rclcpp::Parameter port = this->get_parameter("HostPort");
        rclcpp::Parameter hostIP = this->get_parameter("HostIP");
        rclcpp::Parameter multicastIP = this->get_parameter("MulticastIP");
        rclcpp::Parameter type = this->get_parameter("PointCloudType");
        mArguments.portHost = static_cast<uint16_t>(port.as_int());
        mArguments.ipHost = hostIP.as_string();
        mArguments.ipMulticast = multicastIP.as_string();
        mArguments.type = scala_decoder_sdk::stringToType(type.as_string());
        std::stringstream logMsg;
        logMsg << "\nParameters:" <<
                  "\n\tHostIP:         " << mArguments.ipHost <<
                  "\n\tHostPort:       " << mArguments.portHost <<
                  "\n\tMulticastIP:    " << mArguments.ipMulticast <<
                  "\n\tPointCloudType: " << scala_decoder_sdk::typeToString(mArguments.type) <<
                  std::endl;
        RCLCPP_INFO_ONCE(this->get_logger(), logMsg.str().c_str());
        if (!scala_decoder_sdk::isValidIP(mArguments.ipHost.c_str())) {
            throw std::runtime_error("\nERROR HostIP: " + mArguments.ipHost + " is not valid.\n");
        }
        if (("Unicast" != mArguments.ipMulticast) && ("unicast" != mArguments.ipMulticast) &&
            !scala_decoder_sdk::isValidIP(mArguments.ipMulticast.c_str())) {
            throw std::runtime_error("\nERROR MulticastIP: " + mArguments.ipMulticast + " is not valid.\n");
        }
        if (("Unicast" == mArguments.ipMulticast) || ("unicast" == mArguments.ipMulticast)) {
            (mArguments.ipMulticast).clear();
        }
    }

    rclcpp::TimerBase::SharedPtr mTimer; //!< timer object for getting point clouds
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr mPublisher; //!< publish object
    scala_decoder_sdk::ListenOnPortArgs mArguments; //!< scala_decoder_sdk arguments
    std::shared_ptr<scala_decoder_sdk::ScalaSocketEthernet> mScalaSocketEthernet; //!< Scala SDK socket
    std::thread mSocketThread; //!< Scala SDK socket thread
};

int main(int argc, char * argv[]) {
    try {
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<ScalaSDKPublisher>());
        rclcpp::shutdown();
    } catch (const std::runtime_error& error) {
        std::cerr << "\n" << error.what() << std::endl;
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
