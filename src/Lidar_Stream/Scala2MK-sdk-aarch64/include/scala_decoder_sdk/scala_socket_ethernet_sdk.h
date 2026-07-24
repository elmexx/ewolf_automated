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
 * @brief Socket SDK to Scala1/Scala2 device to connect to device via ethernet.
 *        The functions in class 'ScalaSocketEthernet' are mutually thread safe, but not self-thread-safe. That is:
 *        All functions in the class 'ScalaSocketEthernet' are thread safe regarding asynchronous operation if every
 *        function of the class 'ScalaSocketEthernet' is only used in one thread.
 *        There is no guarantee that the operations are thread safe if any one function of the class
 *        'ScalaSocketEthernet' is used in two or more separated threads.
 *
 * @author Patrick Reichel <patrick.reichel@valeo.com>
 *
 * @date July 2020
 *
 * @file
 */

#ifndef SCALA_DECODER_x_SCALA_DECODER_SDK_x_SCALA_SOCKET_ETHERNET_SDK
#define SCALA_DECODER_x_SCALA_DECODER_SDK_x_SCALA_SOCKET_ETHERNET_SDK

#include <atomic>
#include <cstdint>
#include <fstream>
#include <functional>
#include <memory>
#include <mutex>
#include <ostream>
#include <sstream>
#include <vector>

namespace scala_decoder_sdk {

typedef char byte_t; //!< Definition of byte type

//! Use this flag casted in uint8 in combination with bitwise 'AND' and current scanPointflag to check active flags
enum class ScanPointFlag {
    TransparentPoint = 1,
    ClutterOrRain = 2 * TransparentPoint,
    Ground = 2 * ClutterOrRain,
    Dirt = 2 * Ground,
    HighThreshold = 2 * Dirt,
    Noise = 2 * HighThreshold,
    NearRange = 2 * Noise,
    Marker = 2 * NearRange,
}; // enum class ScanPointFlag

//! Mirror side
enum class MirrorSide {
    undetermined = 0,
    up = 1 ,  //!< Deflecting the laser upwards (in the direction of positive z)
    down =2 //!< Deflecting the laser downwards (in the direction of negative z)
}; // enum class MirrorSide

//! Struct describing a single Scala point
struct ScalaPoint {
    uint8_t scanPointflag{0u}; //!< Active point flags
    uint8_t echo{0u}; //!< Echo id, Echo number: 0 = farthest. Range [0, 2]
    uint8_t layer{0u}; //!< Layer id, Layer number 0 = lowest. Range Scala1 [0, 3], Range Scala2 [0, 15]
    float distance{0.0f}; //!< Distance sensor--scanpoint [m]
    float theta{0.0f}; //!< Polar angle [rad]
    float phi{0.0f}; //!< Azimuthal angle [rad]
    float epw{0.0f}; //!< Echo pulse width [m]
    ScalaPoint() = default;
    ScalaPoint(uint8_t scanPointflag, uint8_t echo, uint8_t layer, float theta, float phi, float distance,
               float epw) : scanPointflag(scanPointflag), echo(echo), layer(layer), distance(distance), theta(theta),
                            phi(phi), epw(epw) {}
}; // struct ScalaPoint

//! Struct describing a Scala point cloud
struct ScalaPointCloud {
    uint16_t scanNumber{0u}; //!< scan number
    MirrorSide mirrorSide{MirrorSide::up}; //!< mirror side of the Scala scan
    uint64_t scanTime{0u}; //!< Scan time in nanoseconds
    float mountYawAngle{0.0f}; //!< Flashed yaw angle [rad]
    float mountPitchAngle{0.0f}; //!< Flashed pitch [rad]
    float mountRollAngle{0.0f}; //!< Flashed roll [rad]
    float mountPositionX{0.0f}; //!< Flashed x position in m
    float mountPositionY{0.0f}; //!< Flashed y angle in m
    float mountPositionZ{0.0f}; //!< Flashed z angle in m
    std::vector<ScalaPoint> points{}; //!< List of points
}; // struct ScalaPointCloud

//!< Different supported Point Cloud types
enum class PointCloudType {
    unspecified = 0, //!< Attempt an autodetect of the point cloud type
    Scala2HI = 1, //!< 'HI' threshold point cloud part of Scala Gen2 'Double Point Cloud'
    Scala2LO = 2, //!< 'LO' threshold point cloud part of Scala Gen2 'Double Point Cloud'
    Scala2Single = 3, //!< Scala Gen2 'Single Point Cloud'
    Scala1 = 4, //!< Scala Gen1 point cloud
    Scala2Double = 5, //!< Complete Scala Gen2 'Double Point Cloud'
    numValues = 6 //!< Number of possible values, excluding itself
}; // enum class PointCloudType

//! Ethernet (UDP) comm. class for Scala sensor
class ScalaSocketEthernet {
public:
    //! Deleted default constructor
    ScalaSocketEthernet() = delete;

    /**
     * @brief Constructor, throws error if it fails. Does nothing if 'PointCloudType::unspecified' is used and
     *        portHost or ipHost are the default values. Sets the internally 'mIsInitialized' to true if socket
     *        implementation initialization succeded.
     * @param[in, out] type PointCloudType of the used Scala Sensor or 'PointCloudType::unspecified' for auto detect
     * @param[in] portHost Optional host port for autodetect of point cloud type
     * @param[in] ipHost Optional host IP for autodetect of point cloud type
     * @param[in] ipMulticast Optional multicast group IP in which to expect incoming Scala traffic
     * @param[in] autodetectPatienceSeconds Optional avail. time for (optional) point cloud type autodetect.
     * @throw std::runtime_error
     */
    ScalaSocketEthernet(PointCloudType& type, const uint16_t portHost = 0u, const std::string& ipHost = "",
                        const std::string& ipMulticast = "", const uint32_t autodetectPatienceSeconds = 3u);

    //! Definded as empty destructor in .cpp
    ~ScalaSocketEthernet();

    //! Deleted copy constructor
    ScalaSocketEthernet(const ScalaSocketEthernet&) = delete;

    //! Setter break execution, sets break execution in listenOnPort (if e.g. listenOnPort running in a thread)
    void setBreakExecution(bool breakExecution = true);

    //! Getter break execution
    bool getBreakExecution();

    /**
     * @brief Function listening on specified port. If 'mIsInitialized' is false the function tries to autodetect
     *        the 'PointCloudType'. Sets the internally 'mIsInitialized' to true if autodetection and the socket
     *        implementation initialization succeded.
     * @param[in] portNumber Port number the scala is sending to
     * @param[in] hostname IP of the host
     * @param[in] numberOfPackagesToProcess Number of scala packages to process (0 -> infinite)
     * @param[in] outputstream Pointer to output stream
     * @param[in] ipMulticast Multicast group, empty string for unicast mode
     */
    void listenOnPort(const uint16_t portNumber,
                      const std::string& hostname,
                      const uint64_t numberOfPackagesToProcess,
                      std::ostream* outputstream = nullptr,
                      const std::string& ipMulticast = "");

    /**
     * @brief Stops the running listenOnPort function, throws error if it fails
     * @param[in] patience Waiting patience [sec]
     * @throw std::runtime_error
     */
    void stopListening(const double patience = 0.5);

    //! Getter running state of listenOnPort
    bool isRunning();

    /**
     * @brief Returns a pointer on the newest complete package in the buffer
     * @param[in] popElement Pop element afterwards from buffer
     * @return Pointer on the newest complete package, nullptr if package is invalid
     */
    std::shared_ptr<const ScalaPointCloud> getNewestCompleteScalaPackage(const bool popElement = false);

    /**
     * @brief Adds a byte stream to the internal buffer - replay method, works without listenOnPort socket generation.
     *        Can be used in combination with getNewestCompleteScalaPackage.
     *        If 'mIsInitialized' is false the function tries to autodetect
     *        the 'PointCloudType'. Sets the internally 'mIsInitialized' to true if autodetection and the socket
     *        implementation initialization succeded.
     * @param[in] byteStream Vector containing the raw byte stream
     */
    void addByteStream(std::shared_ptr<const std::vector<byte_t> > byteStream);

private:
    std::atomic<bool> mIsInitialized{false}; //!< Is socket implementation initialized
    class ImplAutoDetection;
    std::unique_ptr<ImplAutoDetection> mAutoDetection;
    class ImplScalaSocketEthernet;
    std::unique_ptr<ImplScalaSocketEthernet> mSocket;
}; // class ScalaSocketEthernet

} // namespace scala_decoder_sdk

#endif // SCALA_DECODER_x_SCALA_DECODER_SDK_x_SCALA_SOCKET_ETHERNET_SDK
