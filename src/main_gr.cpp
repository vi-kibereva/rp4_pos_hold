#include "msp/bitaflught_msp.hpp"

#include <cstdint>
#include <cstdio>
#include <exception>
#include <clocale>
#include <thread>
#include <chrono>

// MSP command for motor control
constexpr uint8_t MSP_SET_MOTOR = 214;

// Spin motors a little (bench test)
bool spinMotors(msp::BitaflughtMsp &msp) {
    // Example: 4 motors, spin motor 1 to 1200, others minimal 1000
    uint16_t motorValues[4] = {1000, 100, 100, 100};
    uint8_t payload[8];
    for (int i = 0; i < 4; i++) {
        payload[2*i]   = motorValues[i] & 0xFF;
        payload[2*i+1] = motorValues[i] >> 8;
    }

    uint8_t recv_size = 0;
    if (!msp.request(MSP_SET_MOTOR, payload,
                     static_cast<uint8_t>(sizeof(payload)), &recv_size)) {
        std::fprintf(stderr, "MSP_SET_MOTOR failed\n");
        return false;
    }

    std::printf("Motors command sent (bench test)\n");
    return true;
}

int main(int argc, char **argv) {
    setlocale(LC_ALL, "UA"); // Українська локалізація консолі
    if (argc < 2) {
        std::fprintf(stderr, "Usage: %s /dev/ttyUSB0\n", argv[0]);
        return 2;
    }

    const char *port = argv[1];

    try {
        msp::BitaflughtMsp msp(port, B115200, 10);

        // Example: get MSP API version
        constexpr uint8_t MSP_API_VERSION = 0x01;
        std::uint8_t payload[3]; // to receive response
        std::uint8_t recv_size = 0;

        if (!msp.request(MSP_API_VERSION, payload, 3, &recv_size)) {
            std::fprintf(stderr, "No/invalid response to MSP_API_VERSION\n");
            return 1;
        }

        const uint8_t mspProtocol = payload[0];
        const uint8_t apiMajor = payload[1];
        const uint8_t apiMinor = payload[2];
        std::printf("MSP protocol: %u, API: %u.%u\n", mspProtocol, apiMajor,
                    apiMinor);

        // Example: MSP_ATTITUDE
        std::uint8_t payload2[6];
        constexpr uint8_t MSP_ATTITUDE = 108;
        if (msp.request(MSP_ATTITUDE, payload2,
                        static_cast<uint8_t>(sizeof(payload2)), &recv_size)) {
            if (recv_size >= 6) {
                const auto roll_tenths =
                    static_cast<int16_t>(payload2[0] | (payload2[1] << 8));
                const auto pitch_tenths =
                    static_cast<int16_t>(payload2[2] | (payload2[3] << 8));
                const auto yaw_tenths =
                    static_cast<int16_t>(payload2[4] | (payload2[5] << 8));
                std::printf("Attitude ~ roll=%.1f° pitch=%.1f° yaw=%.1f°\n",
                            roll_tenths / 10.0, pitch_tenths / 10.0, yaw_tenths
                            / 10.0);
            } else {
                std::fprintf(stderr, "MSP_ATTITUDE payload size %u (expected >= 6)\n",
                             recv_size);
            }
        } else {
            std::fprintf(stderr, "MSP_ATTITUDE failed or not supported\n");
        }

        // --- New: spin motors a little ---
        if (!spinMotors(msp)) {
            std::fprintf(stderr, "Motor spin failed\n");
        }

        // Wait 2 seconds so you can observe motor spin
        std::this_thread::sleep_for(std::chrono::seconds(2));

        // Stop motors (set all to minimal 1000)
        uint16_t motorValuesStop[4] = {0, 0, 0, 0};
        uint8_t payloadStop[8];
        for (int i = 0; i < 4; i++) {
            payloadStop[2*i]   = motorValuesStop[i] & 0xFF;
            payloadStop[2*i+1] = motorValuesStop[i] >> 8;
        }
        msp.request(MSP_SET_MOTOR, payloadStop, sizeof(payloadStop), &recv_size);

        return 0;

    } catch (const std::exception &ex) {
        std::fprintf(stderr, "Error: %s\n", ex.what());
        return 1;
    }
}
