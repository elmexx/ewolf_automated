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

#ifndef MOBILITYKIT_RADAR_MCR1_H
#define MOBILITYKIT_RADAR_MCR1_H

#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace mobilitykit {
namespace radar {

/// @brief Represents the corner radar sensor
class MCR1 {
   public:
    /// @brief Stores data of one infrastructure detection
    struct Infrastructure {
        float CoGRange;
        float StandardDeviationAzimuth;
        float Azimuth;
    };

    /// @brief Stores data of one non-infrastructure detection
    struct NonInfrastructure {
        float CoGRange;
        float Doppler;
        float StandardDeviationAzimuth;
        float Azimuth;
    };

    struct Position {
        float xOffsetMeters;
        float yOffsetMeters;
        float zOffsetMeters;
        float azimuthAngleDegrees;
    };

    /// @brief Stores data of current detection frame, including the position of the radar sensor
    struct Data {
        Position sensorPosition;
        std::vector<Infrastructure> infraDetections;
        std::vector<NonInfrastructure> nonInfraDetections;
    };

    /// @brief Standard constructor for the corner radar class. Sensor has to be initialized afterwards.
    MCR1(std::string socketName, uint8_t sensorLocationId, bool activeSync);

    /// @brief Decode sensor messages
    /// @return data object holding detections of the latest complete sensor frame
    bool run(std::shared_ptr<Data> data);

    ~MCR1();

   private:
    struct CanMsg {
        uint16_t id;
        uint8_t flag;
        std::vector<uint8_t> data;
    };

    struct Header {
        struct {
            uint32_t infrastructure, nonInfrastructure;
        } expectedNumberOfDetections;
        uint32_t frameId;
        uint32_t timestamp;
    };

    struct Footer {
        uint32_t frameId;
    };

    void canSocketSetup(std::string name);
    void cyclicSync();
    void handleSensorSyncMessage(std::vector<uint8_t> received);
    void insertSensorPosition(std::shared_ptr<Data> data, MCR1::Position pos);
    bool analyzeMessages(std::shared_ptr<Data> data);
    Header decodeHeader(std::vector<uint8_t> raw);
    Footer decodeFooter(std::vector<uint8_t> raw);
    Position decodePosition(std::vector<uint8_t> raw);
    Infrastructure decodeInfrastructure(std::vector<uint8_t> raw);
    NonInfrastructure decodeNonInfrastructure(std::vector<uint8_t> raw);

    bool mRun = false;
    uint8_t mSensorId;
    int16_t mCanSocket;
    std::thread mSyncThread;
    std::vector<CanMsg> mMessages;
};

}  // namespace radar
}  // namespace mobilitykit

#endif  // MOBILITYKIT_RADAR_MCR1_H
