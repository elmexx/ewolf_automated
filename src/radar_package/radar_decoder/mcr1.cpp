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

#include "mcr1.h"

#include <arpa/inet.h>
#include <linux/can/raw.h>
#include <linux/if.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <exception>
#include <iostream>
#include <memory>
#include <vector>

namespace mobilitykit {
namespace radar {

const uint8_t FOOTER_MSG_SIZE = 4;
const uint8_t FRAME_ID_LOCATION = 63;
const uint8_t GENERAL_MSG_SIZE = 64;
const uint8_t TIMESYNC_MSG_TYPE_ID = 4;
const uint8_t INFRA_DETECTION_SIZE = 7;
const uint8_t NON_INFRA_DETECTION_SIZE = 11;
const uint8_t TIMESYNC_MASTER = 0x08;
const uint8_t TIMESYNC_CMD = 0x11;
const uint8_t TIMESYNC_TRANSMIT_DELAY_REQUEST = 0x20;
const uint8_t TIMESYNC_TRANSMIT_DELAY_RESPONSE = 0x21;
const uint8_t TIMESYNC_TRANSMIT_DELAY_INFO = 0x22;
const uint32_t PARAMETERS = 0x110;
const uint32_t HEADER = 0x120;
const uint32_t NON_INFRA_DETECTION = 0x130;
const uint32_t INFRA_DETECTION = 0x140;
const uint32_t FOOTER = 0x150;
const uint32_t TIMESYNC_SLAVE = 0x090;

MCR1::MCR1(std::string socketName, uint8_t sensorLocationId, bool activeSync = false) {
    mMessages.clear();
    mSensorId = std::min(sensorLocationId, uint8_t(4));

    canSocketSetup(socketName);

    mRun = activeSync;
    if (activeSync) {
        mSyncThread = std::thread(&MCR1::cyclicSync, this);
    }
}

void MCR1::canSocketSetup(std::string name) {
    mCanSocket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (mCanSocket < 0) {
        throw std::runtime_error("Cannot create socket");
    }

    ifreq ifr;
    strncpy(ifr.ifr_ifrn.ifrn_name, name.c_str(), sizeof(ifr.ifr_ifrn.ifrn_name));
    ioctl(mCanSocket, SIOCGIFINDEX, &ifr);
    sockaddr_can address;
    memset(&address, 0, sizeof(address));
    address.can_family = AF_CAN;
    address.can_ifindex = ifr.ifr_ifru.ifru_ivalue;

    uint32_t canFd = 1;
    setsockopt(mCanSocket, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &canFd, sizeof(canFd));
    if (bind(mCanSocket, (sockaddr*)&address, sizeof(address)) < 0) {
        throw std::runtime_error("Cannot bind to socket");
    }
}

void MCR1::handleSensorSyncMessage(std::vector<uint8_t> received) {
    canfd_frame CanMsg;
    CanMsg.len = 8;
    CanMsg.can_id = TIMESYNC_MASTER;

    if (received[TIMESYNC_MSG_TYPE_ID] == TIMESYNC_TRANSMIT_DELAY_REQUEST) {
        std::copy(received.begin(), received.end(), CanMsg.data);
        CanMsg.data[TIMESYNC_MSG_TYPE_ID] = TIMESYNC_TRANSMIT_DELAY_RESPONSE;

        if (write(mCanSocket, &CanMsg, sizeof(CanMsg)) < 0) {
            std::cerr << "Failed to answer sync message\n";
        }
    }
}

template <class T>
T rawToInt(const T* valueToDecode) {
    switch (sizeof(T)) {
        case sizeof(uint16_t):
            return ntohs(*valueToDecode);

        case sizeof(uint32_t):
            return ntohl(*valueToDecode);

        default:
            std::cerr << "Cannot decode value with size " << sizeof(T) << "\n";
            return 0.f;
    }
}

void MCR1::cyclicSync() {
    canfd_frame CanMsg;
    CanMsg.data[4] = TIMESYNC_CMD;
    CanMsg.data[5] = mSensorId;
    CanMsg.data[6] = 0x00;
    CanMsg.data[7] = 0x00;
    CanMsg.len = 8;
    CanMsg.can_id = TIMESYNC_MASTER;

    while (mRun) {
        uint64_t time =
            std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch())
                .count();

        CanMsg.data[0] = time >> 24;
        CanMsg.data[1] = time >> 16;
        CanMsg.data[2] = time >> 8;
        CanMsg.data[3] = time;

        if (write(mCanSocket, &CanMsg, sizeof(CanMsg)) < 0) {
            std::cerr << "Failed to send sync command\n";
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
}

MCR1::Header MCR1::decodeHeader(std::vector<uint8_t> raw) {
    Header h;
    h.expectedNumberOfDetections.infrastructure = raw[35];
    h.expectedNumberOfDetections.nonInfrastructure = raw[34];
    h.frameId = rawToInt(reinterpret_cast<uint32_t*>(&raw[0]));
    h.timestamp = rawToInt(reinterpret_cast<uint32_t*>(&raw[4]));
    return h;
}

MCR1::Footer MCR1::decodeFooter(std::vector<uint8_t> raw) {
    Footer f;
    f.frameId = rawToInt(reinterpret_cast<uint32_t*>(&raw[0]));
    return f;
}

MCR1::Infrastructure MCR1::decodeInfrastructure(std::vector<uint8_t> raw) {
    Infrastructure inf;
    inf.CoGRange = rawToInt(reinterpret_cast<uint16_t*>(&raw[0])) / 256.f;
    inf.StandardDeviationAzimuth = raw[3] / 10.f;
    raw[6] &= 0xf0;
    inf.Azimuth = rawToInt(reinterpret_cast<int16_t*>(&raw[5])) / 160.f;
    return inf;
}

MCR1::NonInfrastructure MCR1::decodeNonInfrastructure(std::vector<uint8_t> raw) {
    NonInfrastructure nInf;
    nInf.CoGRange = rawToInt(reinterpret_cast<uint16_t*>(&raw[0])) / 256.f;
    nInf.Doppler = rawToInt(reinterpret_cast<int16_t*>(&raw[2])) / 256.f;
    nInf.StandardDeviationAzimuth = raw[8] / 10.f;
    raw[10] &= 0xf0;
    nInf.Azimuth = rawToInt(reinterpret_cast<int16_t*>(&raw[9])) / 160.f;
    return nInf;
}

MCR1::Position MCR1::decodePosition(std::vector<uint8_t> raw) {
    Position pos;
    pos.xOffsetMeters = rawToInt(reinterpret_cast<int16_t*>(&raw[25])) / 1000.f;
    pos.yOffsetMeters = rawToInt(reinterpret_cast<int16_t*>(&raw[27])) / 1000.f;
    pos.zOffsetMeters = raw[29] / 100.f;
    pos.azimuthAngleDegrees = rawToInt(reinterpret_cast<int16_t*>(&raw[30])) / 10.f;
    return pos;
}

bool MCR1::analyzeMessages(std::shared_ptr<Data> data) {
    uint8_t expectedInfraFrameId = 0;
    uint8_t expectedNonInfraFrameId = 0;
    Header header;

    for (auto msg : mMessages) {
        if (msg.data.size() < FOOTER_MSG_SIZE) {
            return false;
        }
        if (msg.id == FOOTER + mSensorId) {
            std::sort(data->infraDetections.begin(), data->infraDetections.end(),
                      [](const Infrastructure& a, const Infrastructure& b) { return a.CoGRange < b.CoGRange; });
            std::sort(data->nonInfraDetections.begin(), data->nonInfraDetections.end(),
                      [](const NonInfrastructure& a, const NonInfrastructure& b) { return a.CoGRange < b.CoGRange; });
            return (header.frameId == decodeFooter(msg.data).frameId) &&
                   (data->infraDetections.size() == header.expectedNumberOfDetections.infrastructure) &&
                   (data->nonInfraDetections.size() == header.expectedNumberOfDetections.nonInfrastructure);
        }
        if (msg.data.size() < GENERAL_MSG_SIZE) {
            return false;
        }
        if (msg.id == HEADER + mSensorId) {
            header = decodeHeader(msg.data);
        } else if (msg.id == INFRA_DETECTION + mSensorId) {
            if (msg.data.size() != GENERAL_MSG_SIZE) {
                return false;
            }
            if (msg.data[FRAME_ID_LOCATION] != expectedInfraFrameId) {
                std::cerr << "Received unexpected infrastructure detection frame\n";
                return false;
            }
            for (auto record = msg.data.cbegin(); record < msg.data.cend() - INFRA_DETECTION_SIZE;
                 record += INFRA_DETECTION_SIZE) {
                if (data->infraDetections.size() < header.expectedNumberOfDetections.infrastructure) {
                    data->infraDetections.push_back(
                        decodeInfrastructure(std::vector<uint8_t>{record, record + INFRA_DETECTION_SIZE}));
                }
            }
            ++expectedInfraFrameId;
        } else if (msg.id == NON_INFRA_DETECTION + mSensorId) {
            if (msg.data[FRAME_ID_LOCATION] != expectedNonInfraFrameId) {
                std::cerr << "Received unexpected non-infrastructure detection frame\n";
                return false;
            }
            for (auto record = msg.data.cbegin(); record < msg.data.cend() - NON_INFRA_DETECTION_SIZE;
                 record += NON_INFRA_DETECTION_SIZE) {
                if (data->nonInfraDetections.size() < header.expectedNumberOfDetections.nonInfrastructure) {
                    data->nonInfraDetections.push_back(
                        decodeNonInfrastructure(std::vector<uint8_t>{record, record + NON_INFRA_DETECTION_SIZE}));
                }
            }
            ++expectedNonInfraFrameId;
        }
    }
    return false;
}

void MCR1::insertSensorPosition(std::shared_ptr<Data> data, MCR1::Position pos) {
    data->sensorPosition.xOffsetMeters = pos.xOffsetMeters;
    data->sensorPosition.yOffsetMeters = pos.yOffsetMeters;
    data->sensorPosition.zOffsetMeters = pos.zOffsetMeters;
    data->sensorPosition.azimuthAngleDegrees = pos.azimuthAngleDegrees;
}

bool MCR1::run(std::shared_ptr<Data> data) {
    canfd_frame readFrame;
    if (read(mCanSocket, &readFrame, sizeof(readFrame)) < 0) {
        std::cerr << "Failed to read frame. Is the interface up?\n";
        return false;
    }

    if (readFrame.can_id == TIMESYNC_SLAVE + mSensorId) {
        handleSensorSyncMessage(std::vector<uint8_t>(readFrame.data, readFrame.data + readFrame.len));
        return false;
    }

    mMessages.push_back({static_cast<uint16_t>(readFrame.can_id), readFrame.flags,
                         std::vector<uint8_t>(readFrame.data, readFrame.data + readFrame.len)});

    if (mMessages.back().id == PARAMETERS + mSensorId) {
        insertSensorPosition(data, decodePosition(mMessages.back().data));
        return false;
    }

    if (mMessages.back().id == FOOTER + mSensorId) {
        data->infraDetections.clear();
        data->nonInfraDetections.clear();
        if (!analyzeMessages(data)) {
            mMessages.clear();
            return false;
        };
        mMessages.clear();
        return true;
    }

    return false;
}

MCR1::~MCR1() {
    mRun = false;
    if (mSyncThread.joinable()) {
        mSyncThread.join();
    }
}

}  // namespace radar
}  // namespace mobilitykit
