#include "msp/bitaflught_msp.hpp"

#include <memory>
#include <termios.h>
#include <chrono>


namespace msp {
    BitaflughtMsp::BitaflughtMsp(const char *dev, speed_t baud_rate, cc_t timeout) : stream_(*dev, baud_rate, timeout) {
        timeout_ = timeout;
        uint8_t * buffer_;
    }

    void BitaflughtMsp::send(std::uint8_t messageID, void *payload, std::uint8_t size) {
        auto message = std::make_unique<uint8_t[]>(size + 6);
        message[0] = '$';
        message[1] = 'M';
        message[2] = '<';
        message[3] = size;
        message[4] = messageID;
        uint8_t checksum = size ^ messageID;
        auto * payloadPtr = (uint8_t*)payload;

        for (uint8_t i = 0; i < size; ++i) {
            uint8_t b = *(payloadPtr++);
            checksum ^= b;
            message[i+5] = b;
        }
        message[size+5] = checksum;
        stream_.write(message.get(), size+6);
    }

    void BitaflughtMsp::error(std::uint8_t messageID, void *payload, std::uint8_t size) {
        auto message = std::make_unique<uint8_t[]>(size + 6);
        message[0] = '$';
        message[1] = 'M';
        message[2] = '!';
        message[3] = size;
        message[4] = messageID;
        uint8_t checksum = size ^ messageID;
        auto * payloadPtr = (uint8_t*)payload;

        for (uint8_t i = 0; i < size; ++i) {
            uint8_t b = *(payloadPtr++);
            checksum ^= b;
            message[i+5] = b;
        }
        message[size+5] = checksum;
        stream_.write(message.get(), size+6);
    }

    void BitaflughtMsp::response(std::uint8_t messageID, void *payload, std::uint8_t size) {
        auto message = std::make_unique<uint8_t[]>(size + 6);
        message[0] = '$';
        message[1] = 'M';
        message[2] = '>';
        message[3] = size;
        message[4] = messageID;
        uint8_t checksum = size ^ messageID;
        auto * payloadPtr = (uint8_t*)payload;

        for (uint8_t i = 0; i < size; ++i) {
            uint8_t b = *(payloadPtr++);
            checksum ^= b;
            message[i+5] = b;
        }
        message[size+5] = checksum;
        stream_.write(message.get(), size+6);
    }

    bool BitaflughtMsp::recv(std::uint8_t *messageID, void *payload, std::uint8_t maxSize,
          std::uint8_t *recvSize) {
        while (true) {
            ...
        }
    }

    size_t BitaflughtMsp::reset() {
        stream_.flush();
        while (stream_.available() > 0)
            return stream_.read(buffer_, 255);
    }

    bool BitaflughtMsp::waitFor(std::uint8_t messageID, void *payload, std::uint8_t maxSize,
               std::uint8_t *recvSize) {
        uint8_t recvMessageID;
        uint8_t recvSizeValue;
        const auto t0 = std::chrono::steady_clock::now();

        while (std::chrono::steady_clock::now() - t0 < timeout_)
            if (recv(&recvMessageID, payload, maxSize, (recvSize ? recvSize : &recvSizeValue)) && messageID == recvMessageID)
                return true;
        return false;
    }

    bool BitaflughtMsp::command(std::uint8_t messageID, void *payload, std::uint8_t size, bool waitACK) {
        send(messageID, payload, size);

        if (waitACK)
            return waitFor(messageID, NULL, 0);
        return true;
    }
    bool BitaflughtMsp::getActiveModes(std::uint32_t *activeModes) {

    }

    static const uint8_t BOXIDS[30] PROGMEM = {
        0,  //  0: MSP_MODE_ARM
        1,  //  1: MSP_MODE_ANGLE
        2,  //  2: MSP_MODE_HORIZON
        3,  //  3: MSP_MODE_NAVALTHOLD (cleanflight BARO)
        5,  //  4: MSP_MODE_MAG
        6,  //  5: MSP_MODE_HEADFREE
        7,  //  6: MSP_MODE_HEADADJ
        8,  //  7: MSP_MODE_CAMSTAB
        10, //  8: MSP_MODE_NAVRTH (cleanflight GPSHOME)
        11, //  9: MSP_MODE_NAVPOSHOLD (cleanflight GPSHOLD)
        12, // 10: MSP_MODE_PASSTHRU
        13, // 11: MSP_MODE_BEEPERON
        15, // 12: MSP_MODE_LEDLOW
        16, // 13: MSP_MODE_LLIGHTS
        19, // 14: MSP_MODE_OSD
        20, // 15: MSP_MODE_TELEMETRY
        21, // 16: MSP_MODE_GTUNE
        22, // 17: MSP_MODE_SONAR
        26, // 18: MSP_MODE_BLACKBOX
        27, // 19: MSP_MODE_FAILSAFE
        28, // 20: MSP_MODE_NAVWP (cleanflight AIRMODE)
        29, // 21: MSP_MODE_AIRMODE (cleanflight DISABLE3DSWITCH)
        30, // 22: MSP_MODE_HOMERESET (cleanflight FPVANGLEMIX)
        31, // 23: MSP_MODE_GCSNAV (cleanflight BLACKBOXERASE)
        32, // 24: MSP_MODE_HEADINGLOCK
        33, // 25: MSP_MODE_SURFACE
        34, // 26: MSP_MODE_FLAPERON
        35, // 27: MSP_MODE_TURNASSIST
        36, // 28: MSP_MODE_NAVLAUNCH
        37, // 29: MSP_MODE_AUTOTRIM
      };



}
