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
 * @brief Command-line parsing
 *
 * @param [optional] Integer value defining how many scala package fragments should be read.
 *                   0 -> unlimited number. Default 1024.
 * @param [optional] Integer value defining the host port to which the Scala device is sending.
 * @param [optional] String defining the IP address of the host. Default "192.168.1.123".
 * @param [optional] Integer value defining the Scala port to which the host will send timstamps. Default 2017.
 * @param [optional] String defining the IP address of the Scala sensor. Default "192.168.1.52".
 * @param [optional] String that describes the expected Mutlicast / Unicast behavior of the Scala sensor.
 *                   Can be "Unicast", or a valid IP address for Multicast. Default "224.111.111.111".
 * @param [optional] String that describes the Point Cloud type of the Scala Sensor.
 *                   Can be "unspecified", "Scala2HI", "Scala2LO", "Scala2Single", "Scala1" or "Scala2Double".
 *                   Default "Scala2Double".
 *
 * @author Patrick Reichel <patrick.reichel@valeo.com>
 *
 * @date July 2020
 *
 * @file
 */

#ifndef SCALA_DECODER_x_SCALA_DECODER_SDK_x_SCALA_DECODER_PARSING_SDK
#define SCALA_DECODER_x_SCALA_DECODER_SDK_x_SCALA_DECODER_PARSING_SDK

#include <iostream> // std::cout
#include <memory> // std::unique_ptr
#include <string> // std::string

#include "scala_socket_ethernet_sdk.h"

namespace scala_decoder_sdk {

//! Positional argmunents for scala_decoder_sdk w. defaults:
struct ListenOnPortArgs {
    /// @cond
    uint32_t numPacketsToRead{1024};
    uint16_t portHost{22017};
    uint16_t portScala{2017};
    std::string ipHost = "192.168.1.123";
    std::string ipScala = "192.168.1.52";
    std::string ipMulticast = "224.111.111.111";
    PointCloudType type{PointCloudType::Scala2Double};
    /// @endcond
};

/**
 * @brief Parse positional arguments of the scala_decoder_sdk_app and reports on output *channel
 * @param [in] argc 1 + # of positional args
 * @param [in] argv Positional args
 * @param [in] channel Output channel, such as &std::cout
 * @return Parsed ListenOnPortArgs, if parsing fails pointer is NULL
 */
std::unique_ptr<ListenOnPortArgs> parsePositionalArgs(const int argc, char** argv, std::ostream* channel = &std::cout);

/**
 * @brief Display an explanation of the expected positional arguments to *channel
 * @param [in] myName Name of the executable, typically derived from argv[0]
 * @param [in] channel Output channel, such as &std::cout
 */
void explainUsage(const std::string& myName, std::ostream* channel);

/**
 * @brief Check if the argument represents a valid IP address
 * @details 0's are excluded as they are in general not allowed for all bytes.
 *          Example: Netmask 192.168.0.0 -> IP 192.168.0.0 is not allowed, but 192.168.0.1 is allowed.
 *          255's are excluded because such IP's may represent a broadcast address.
 * @param[in] address Address to be checked
 * @return True if 'address' == aaa.bbb.ccc.ddd with 1 <= aaa, bbb, ccc, ddd <= 254
 */
bool isValidIP(const char* address);

/**
 * @brief Print a summary of the parsed, or default, ListenOnPort arguments to *channel
 * @param [in] args ListenOnPort arguments
 * @param [in] channel Output channel, such as &std::cout
 */
void summarizePositionalArgs(const ListenOnPortArgs& args, std::ostream* channel);

/**
 * @brief Converts scala_decoder_sdk::PointCloudType to string name
 * @param [in] type PointCloudType enum class input
 * @return std::string type name
 */
std::string typeToString(PointCloudType type);

/**
 * @brief Converts string name to scala_decoder_sdk::PointCloudType
 * @param [in] name String from PointCloudType value.
 *                  Allowed: "Scala2HI", "Scala2LO", "Scala2Double", "Scala2Single", "Scala1" or "unspecified"
 * @return PointCloudType enum class
 */
PointCloudType stringToType(const std::string& name);

} // namespace scala_decoder_sdk

#endif // SCALA_DECODER_x_SCALA_DECODER_SDK_x_SCALA_DECODER_PARSING_SDK
