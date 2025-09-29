#include "msp/bitaflught_msp.hpp"

#include <cstdint>
#include <cstdio>
#include <exception>
#include <clocale>

int main(int argc, char **argv) {
  setlocale(LC_ALL, "UA"); // Українська локалізація консолі
  if (argc < 2) {
    std::fprintf(stderr, "Usage: %s /dev/ttyUSB0\n", argv[0]);
    return 2;
  }

  const char *port = argv[1];

  try {
    // Construct MSP client
    msp::BitaflughtMsp msp(port, B115200, 10);

    // --- Example 1: MSP_API_VERSION ---
    constexpr uint8_t MSP_API_VERSION = 0x01;
    std::uint8_t payload[3]; // to receive response
    std::uint8_t recv_size = 0;

    if (!msp.request(MSP_API_VERSION, payload, 3, &recv_size)) {
      std::fprintf(stderr, "No/invalid response to MSP_API_VERSION\n");
      return 1;
    }

    if (recv_size < 3) {
      std::fprintf(stderr, "MSP_API_VERSION payload too small: %u\n",
                   recv_size);
      return 1;
    }

    const uint8_t mspProtocol = payload[0];
    const uint8_t apiMajor = payload[1];
    const uint8_t apiMinor = payload[2];
    std::printf("MSP protocol: %u, API: %u.%u\n", mspProtocol, apiMajor,
                apiMinor);
    //
    // // --- Example 2: MSP_ATTITUDE ---
    // constexpr uint8_t MSP_ATTITUDE = 109;
    // payload.resize(6); // attitude returns 6 bytes
    // if (msp.request(MSP_ATTITUDE, payload.data(),
    //                 static_cast<uint8_t>(payload.size()), &recv_size)) {
    //   if (recv_size >= 6) {
    //     const int16_t roll_tenths =
    //         static_cast<int16_t>(payload[0] | (payload[1] << 8));
    //     const int16_t pitch_tenths =
    //         static_cast<int16_t>(payload[2] | (payload[3] << 8));
    //     const int16_t yaw_tenths =
    //         static_cast<int16_t>(payload[4] | (payload[5] << 8));
    //     std::printf("Attitude ~ roll=%.1f° pitch=%.1f° yaw=%.1f°\n",
    //                 roll_tenths / 10.0, pitch_tenths / 10.0, yaw_tenths
    //                 / 10.0);
    //   } else {
    //     std::fprintf(stderr, "MSP_ATTITUDE payload size %u (expected >=
    //     6)\n",
    //                  recv_size);
    //   }
    // } else {
    //   std::fprintf(stderr, "MSP_ATTITUDE failed or not supported\n");
    // }
    //
    // return 0;

  } catch (const std::exception &ex) {
    std::fprintf(stderr, "Error: %s\n", ex.what());
    return 1;
  }
}
