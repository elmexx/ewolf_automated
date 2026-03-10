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
 * @brief Scala point conversion
 *
 * @author Patrick Reichel <patrick.reichel@valeo.com>
 *
 * @date July 2020
 *
 * @file
 */

#ifndef SCALA_DECODER_x_SCALA_DECODER_SDK_x_SCALA_POINT_CONVERSION_SDK
#define SCALA_DECODER_x_SCALA_DECODER_SDK_x_SCALA_POINT_CONVERSION_SDK

#include <cmath> // std::cos, std::sin

#include "scala_socket_ethernet_sdk.h"

namespace scala_decoder_sdk {

struct cartesianPoint {
    float x{0.0f}; //!< x coordinate in m
    float y{0.0f}; //!< y coordinate in m
    float z{0.0f}; //!< z coordinate in m
    cartesianPoint() = default;
    cartesianPoint(float x, float y, float z) : x(x), y(y), z(z) {}
};

inline cartesianPoint getCartesianPoint(const ScalaPoint& point) {
    return (cartesianPoint(point.distance * std::cos(point.phi) * std::cos(point.theta),
                           point.distance * std::sin(point.phi) * std::cos(point.theta),
                           point.distance * std::sin(point.theta)));
}

} // namespace scala_decoder_sdk

#endif // SCALA_DECODER_x_SCALA_DECODER_SDK_x_SCALA_POINT_CONVERSION_SDK
