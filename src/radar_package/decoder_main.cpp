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

#include <iomanip>
#include <iostream>
#include <memory>

#include "radar_decoder/mcr1.h"

using namespace mobilitykit::radar;

int main(int argc, char* argv[]) {
    std::string socketName = "can0";
    if (argc > 1) {
        socketName = argv[1];
    }

    uint8_t sensorId = 0;
    if (argc > 2) {
        sensorId = std::atoi(argv[2]);
    }

    MCR1 sensor(socketName, sensorId, true);

    auto sensorData = std::make_shared<MCR1::Data>();
    while (true) {
        if (!sensor.run(sensorData)) {
            continue;
        };
        unsigned displayedDetections = 5;
        unsigned detectionNumber = 0;
        std::cout << "-------- Non infrastructure detections (" << std::setw(3) << sensorData->nonInfraDetections.size()
                  << ") --------\n";
        for (const MCR1::NonInfrastructure detection : sensorData->nonInfraDetections) {
            ++detectionNumber;
            if (detectionNumber > displayedDetections) {
                std::cout << std::setw(2) << detectionNumber << " ...\n";
                break;
            } else {
                std::cout << std::setw(2) << detectionNumber << " Distance " << std::fixed << std::setw(7)
                          << std::setfill(' ') << std::setprecision(3) << detection.CoGRange << " m angle "
                          << std::setw(6) << std::setprecision(1) << detection.Azimuth << "° ("
                          << detection.StandardDeviationAzimuth << "°) doppler " << detection.Doppler << " m/s\n";
            }
        }
        ++detectionNumber;
        for (; detectionNumber <= displayedDetections + 1; ++detectionNumber) {
            std::cout << std::setw(2) << detectionNumber << "\n";
        }
        std::cout << "-----------------------------------------------------\n\n";

        std::cout << "---------- Infrastructure detections (" << std::setw(3) << sensorData->infraDetections.size()
                  << ") ----------\n";
        displayedDetections = 30;
        detectionNumber = 0;
        for (const MCR1::Infrastructure detection : sensorData->infraDetections) {
            ++detectionNumber;
            if (detectionNumber > displayedDetections) {
                std::cout << std::setw(2) << detectionNumber << " ...\n";
                break;
            } else {
                std::cout << std::setw(2) << detectionNumber << " Distance " << std::fixed << std::setw(7)
                          << std::setfill(' ') << std::setprecision(3) << detection.CoGRange << " m angle "
                          << std::setw(6) << std::setprecision(1) << detection.Azimuth << "° ("
                          << detection.StandardDeviationAzimuth << "°)\n";
            }
        }
        ++detectionNumber;
        for (; detectionNumber <= displayedDetections + 1; ++detectionNumber) {
            std::cout << std::setw(2) << detectionNumber << "\n";
        }
        std::cout << "-----------------------------------------------------\n\n";

        std::cout << "Sensor position: X " << sensorData->sensorPosition.xOffsetMeters << " | Y "
                  << sensorData->sensorPosition.yOffsetMeters << " | Angle "
                  << sensorData->sensorPosition.azimuthAngleDegrees << "\n\n\n\n";

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}
