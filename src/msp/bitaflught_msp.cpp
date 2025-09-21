#include <chrono>
#include <cstdint>
#include <cstring>
#include <termios.h>

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

bool BitaflughtMsp::recv(std::uint8_t *command_id, void *payload,
                         std::uint8_t max_size, std::uint8_t *recv_size) {
  uint8_t buffer[255+6];
  while (true) {
    size_t size = stream_.read(buffer, max_size);
    if (size == 0) {
      return false;
    }
    if (buffer[0] == '$' && buffer[1] == 'M' && buffer[2] == '<') {
      *recv_size = buffer[3];

      *command_id = buffer[4];

      uint8_t checksumCalc = *recv_size ^ *command_id;

      uint8_t * payload_ptr = static_cast<uint8_t *>(payload);

      for (int i = 0; i<*recv_size; ++i) {
        uint8_t b = buffer[i+5];
        checksumCalc ^= b;
        *(payload_ptr++) = b;
      }
      for (std::uint8_t j = *recv_size + 5; j < max_size; ++j) {
        *(payload_ptr++) = 0;
      }
      uint8_t checksum = buffer[size-1];
      if (checksumCalc == checksum) {
        return true;
      }

    }
  }
}

bool BitaflughtMsp::request(std::uint8_t command_id, void *payload,
                            std::uint8_t max_size, std::uint8_t *recv_size) {
  send(CommandType::Request, command_id, NULL, 0);
  return waitFor(command_id, payload, max_size, recv_size);
}

void BitaflughtMsp::reset() { stream_.flush(); }

  bool BitaflughtMsp::waitFor(std::uint8_t command_id,
                              void*        payload,
                              std::uint8_t max_size,
                              std::uint8_t* recv_size) {
  std::uint8_t rx_id = 0;
  std::uint8_t scratch_len = 0;
  std::uint8_t* out_len = recv_size ? recv_size : &scratch_len;

  for (;;) {
    if (!recv(&rx_id, payload, max_size, out_len)) {
      if (recv_size) *recv_size = 0;
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

static constexpr uint8_t BOXIDS[30] = {
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

} // namespace msp
