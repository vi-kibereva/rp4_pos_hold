#include "msp/msp.hpp"

#include <exception>
#include <iostream>

int main(int argc, char **argv) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " /dev/ttyUSB0\n";
    return 2;
  }

  const char *port = argv[1];

  try {
    // Construct MSP client
    msp::Msp msp(port, B115200, 10);

    // --- Example: MSP_STATUS ---
    std::cout << msp.status() << '\n';

    // --- Example: MSP_RC ---
    std::cout << msp.rc() << '\n';

    // --- Example: MSP_ATTITUDE ---
    std::cout << msp.attitude() << '\n';

    // --- Example: MSP_ALTITUDE ---
    std::cout << msp.altitude() << '\n';

    // --- Example: MSP_SET_RAW_RC (commented out for safety) ---
    // Uncomment to send RC values: roll, pitch, throttle, yaw, aux1-4
    // msp::SetRawRcData rc_data(1500, 1500, 1000, 1500, 1000, 1000, 1000,
    // 1000); std::cout << "Sending: " << rc_data << '\n';
    // msp.setRawRc(rc_data);
    // std::cout << "RC values sent successfully\n";

    return 0;

  } catch (const std::exception &ex) {
    std::cerr << "Error: " << ex.what() << '\n';
    return 1;
  }
}
