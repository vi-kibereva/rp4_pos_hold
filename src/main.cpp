#include "msp/bitaflught_msp.hpp"
#include "msp/serial_stream.hpp"
#include <memory>

int main() {
  auto bmsp = std::make_unique<msp::BitaflughtMsp>(msp::DEFAULT_SERIAL_DEVICE);

  constexpr uint8_t MSP_API_VERSION = 0x01;

  return 0;
}
