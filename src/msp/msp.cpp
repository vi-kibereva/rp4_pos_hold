#include "msp/msp.hpp"

namespace msp {

Msp::Msp(const char *dev, speed_t baud_rate, cc_t timeout)
    : bitaflught_msp_(dev, baud_rate, timeout) {}

AttitudeData Msp::attitude() {
  constexpr std::uint8_t EXPECTED_SIZE = 6;
  std::uint8_t payload[EXPECTED_SIZE];
  std::uint8_t recv_size = 0;

  if (!bitaflught_msp_.request(MSP_ATTITUDE, payload, EXPECTED_SIZE,
                               &recv_size)) {
    throw std::runtime_error("MSP_ATTITUDE request failed or timed out");
  }

  return AttitudeData(recv_size, payload);
}

StatusData Msp::status() {
  constexpr std::uint8_t EXPECTED_SIZE = 13;
  std::uint8_t payload[32]; // MSP_STATUS can be longer with extended flags
  std::uint8_t recv_size = 0;

  if (!bitaflught_msp_.request(MSP_STATUS, payload, sizeof(payload),
                               &recv_size)) {
    throw std::runtime_error("MSP_STATUS request failed or timed out");
  }

  return StatusData(recv_size, payload);
}

RcData Msp::rc() {
  std::uint8_t payload[MAX_RC_CHANNELS * 2]; // 2 bytes per channel
  std::uint8_t recv_size = 0;

  if (!bitaflught_msp_.request(MSP_RC, payload, sizeof(payload),
                               &recv_size)) {
    throw std::runtime_error("MSP_RC request failed or timed out");
  }

  return RcData(recv_size, payload);
}

AltitudeData Msp::altitude() {
  constexpr std::uint8_t EXPECTED_SIZE = 6;
  std::uint8_t payload[EXPECTED_SIZE];
  std::uint8_t recv_size = 0;

  if (!bitaflught_msp_.request(MSP_ALTITUDE, payload, EXPECTED_SIZE,
                               &recv_size)) {
    throw std::runtime_error("MSP_ALTITUDE request failed or timed out");
  }

  return AltitudeData(recv_size, payload);
}

} // namespace msp
