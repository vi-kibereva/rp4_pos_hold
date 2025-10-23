#include "msp/msp.hpp"

#include <exception>
#include <iostream>
#include <unistd.h>
#include <chrono>
#include <thread>

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

    auto start = std::chrono::steady_clock::now();

    // --- Example: MSP_SET_RAW_RC (commented out for safety) ---
    msp::SetRawRcData rc_data(1500, 1500, 1000, 1500, 1900, 1000, 1700, 1000);
    std::cout << "Sending: " << rc_data << '\n';
    while (true) {
      msp.setRawRc(rc_data);
      auto now = std::chrono::steady_clock::now();
      if (duration_cast<std::chrono::seconds>(now - start).count() >= 3)
        break;
    }
    std::cout << "RC values sent successfully\n";

    sleep(1);

    std::cout << msp.rc() << '\n';

    return 0;

  } catch (const std::exception &ex) {
    std::cerr << "Error: " << ex.what() << '\n';
    return 1;
  }
}
