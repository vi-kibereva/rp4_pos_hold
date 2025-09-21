#include <cstdint>
#include <cstring>

#include "msp/bitaflught_msp.hpp"
#include "msp/serial_stream.hpp"

namespace msp {

BitaflughtMsp::BitaflughtMsp(const char *dev, speed_t baud_rate, cc_t timeout)
    : stream_(dev, baud_rate, timeout) {}

void BitaflughtMsp::send(CommandType command_type, std::uint8_t command_id,
                         const void *payload, std::uint8_t size) {
  uint8_t frame[5 + 255 + 1];
  frame[0] = '$';
  frame[1] = 'M';
  frame[2] = msp::to_underlying(command_type);
  frame[3] = size;
  frame[4] = command_id;

  const uint8_t *p = static_cast<const uint8_t *>(payload);
  uint8_t chk = static_cast<uint8_t>(size ^ command_id);

  for (uint8_t i = 0; i < size; ++i) {
    uint8_t b = p[i];
    frame[5 + i] = b;
    chk ^= b;
  }

  frame[5 + size] = chk;

  const size_t total = 5 + size + 1;
  stream_.write(frame, total);
}

} // namespace msp
