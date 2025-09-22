#include <cstdint>
#include <cstring>
#include <iostream>
#include <termios.h>

#include "msp/bitaflught_msp.hpp"
#include "msp/serial_stream.hpp"

namespace msp {

BitaflughtMsp::BitaflughtMsp(const char *dev, speed_t baud_rate, cc_t timeout)
    : stream_(dev, baud_rate, timeout) {}

BitaflughtMsp::~BitaflughtMsp() = default;

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

bool BitaflughtMsp::recv(std::uint8_t *command_id, void *payload,
                         std::uint8_t max_size, std::uint8_t *recv_size) {
  uint8_t buffer[6 + 255];
  while (true) {
    size_t size = stream_.read(buffer, sizeof(buffer));
    if (size == 0) {
      return false;
    }
    if (buffer[0] == '$' && buffer[1] == 'M' && buffer[2] == '<') {
      *recv_size = buffer[3];
      std::cout << "*recv_size: " << static_cast<int>(*recv_size) << std::endl;

      for (int i = 0; i < 10; ++i) {
        std::printf("%02X%s", buffer[i], (i == 9 ? "\n" : " "));
      }

      *command_id = buffer[4];

      uint8_t checksumCalc = *recv_size ^ *command_id;

      uint8_t *payload_ptr = static_cast<uint8_t *>(payload);

      for (int i = 0; i < *recv_size; ++i) {
        uint8_t b = buffer[i + 5];
        checksumCalc ^= b;
        *(payload_ptr++) = b;
      }
      for (std::uint8_t j = *recv_size + 5; j < max_size; ++j) {
        *(payload_ptr++) = 0;
      }
      uint8_t checksum = buffer[*recv_size + 5];
      if (checksumCalc == checksum) {
        return true;
      } else {
        std::cout << "Invalid checksum (calc): "
                  << static_cast<int>(checksumCalc)
                  << "; (received): " << static_cast<int>(checksum)
                  << std::endl;
      }
    } else {
      std::cout << "shit" << std::endl;
    }
  }

  std::cout << "Huy " << buffer << std::endl;
}

bool BitaflughtMsp::request(std::uint8_t command_id, void *payload,
                            std::uint8_t max_size, std::uint8_t *recv_size) {
  send(CommandType::Request, command_id, NULL, 0);
  return waitFor(command_id, payload, max_size, recv_size);
}

void BitaflughtMsp::reset() { stream_.flush(); }

bool BitaflughtMsp::waitFor(std::uint8_t command_id, void *payload,
                            std::uint8_t max_size, std::uint8_t *recv_size) {
  std::uint8_t rx_id = 0;
  std::uint8_t scratch_len = 0;
  std::uint8_t *out_len = recv_size ? recv_size : &scratch_len;

  for (;;) {
    if (!recv(&rx_id, payload, max_size, out_len)) {
      if (recv_size)
        *recv_size = 0;
      return false;
    }

    if (rx_id == command_id) {
      return true;
    }
  }
}

bool BitaflughtMsp::command(std::uint8_t command_id, void *payload,
                            std::uint8_t size, bool wait_ACK) {
  send(CommandType::Request, command_id, payload, size);

  if (wait_ACK)
    return waitFor(command_id, NULL, 0);
  return true;
}
bool BitaflughtMsp::getActiveModes(std::uint32_t *active_modes) {
  (void)active_modes;

  return true;
}

} // namespace msp
