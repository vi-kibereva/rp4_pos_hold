#include "msp/msp.hpp"

#include <clocale>
#include <cstdint>
#include <cstdio>
#include <exception>

int main(int argc, char **argv) {
  if (argc < 2) {
    std::fprintf(stderr, "Usage: %s /dev/ttyUSB0\n", argv[0]);
    return 2;
  }

  const char *port = argv[1];

  try {
    // Construct MSP client
    msp::Msp msp(port, B115200, 10);

    // --- Example: MSP_STATUS ---
    auto status = msp.status();
    std::printf("Status ~ cycle_time=%u us, i2c_errors=%u, sensors=0x%04X, "
                "pid_profile=%u, system_load=%u%%\n",
                status.cycle_time, status.i2c_errors, status.sensor_flags,
                status.pid_profile, status.system_load);

    // --- Example: MSP_RC ---
    auto rc = msp.rc();
    std::printf("RC ~ %u channels: ", rc.channel_count);
    for (std::uint8_t i = 0; i < rc.channel_count; i++) {
      std::printf("%u ", rc.channels[i]);
    }
    std::printf("\n");

    // --- Example: MSP_ATTITUDE ---
    auto attitude = msp.attitude();
    std::printf("Attitude ~ roll=%.1f° pitch=%.1f° yaw=%.1f°\n",
                attitude.roll_tenths / 10.0, attitude.pitch_tenths / 10.0,
                attitude.yaw_tenths / 10.0);

    // --- Example: MSP_ALTITUDE ---
    auto altitude = msp.altitude();
    std::printf("Altitude: %d cm, Vario: %d cm/s\n", altitude.altitude,
                altitude.vario);

    return 0;

  } catch (const std::exception &ex) {
    std::fprintf(stderr, "Error: %s\n", ex.what());
    return 1;
  }
}
