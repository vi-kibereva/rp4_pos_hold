#ifndef MSP_HPP
#define MSP_HPP

#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <string>
#include <termios.h>

#include "bitaflught_msp.hpp"
#include "box_ids.hpp"

namespace msp {

/**
 * @brief MSP command identifiers for Betaflight/Cleanflight protocol.
 *
 * Each constant represents a specific message type in the MSP v1 protocol.
 * Used as command_id in send/request operations.
 */
enum MspCommand : std::uint8_t {
  MSP_API_VERSION = 1,
  MSP_STATUS = 101,
  MSP_RC = 105,
  MSP_ATTITUDE = 108,
  MSP_ALTITUDE = 109,
  MSP_SET_RAW_RC = 200,
};

constexpr std::uint8_t MAX_RC_CHANNELS = 18;

/**
 * @brief RC channel data from MSP_RC.
 *
 * This structure holds the parsed response from an MSP_RC request
 * (command 105). Contains RC channel values, typically in range [1000, 2000].
 */
struct RcData {
  std::uint8_t channel_count;              ///< Number of channels received.
  std::uint16_t channels[MAX_RC_CHANNELS]; ///< RC channel values.

  RcData(std::uint8_t recv_size, std::uint8_t *payload) {
    if (recv_size < 2 || recv_size % 2 != 0) {
      throw std::runtime_error("MSP_RC payload size " +
                               std::to_string(recv_size) +
                               " invalid (expected even number >= 2)\n");
    }

    channel_count = recv_size / 2;
    if (channel_count > MAX_RC_CHANNELS) {
      channel_count = MAX_RC_CHANNELS;
    }

    for (std::uint8_t i = 0; i < channel_count; i++) {
      channels[i] =
          static_cast<uint16_t>(payload[i * 2] | (payload[i * 2 + 1] << 8));
    }
  }

  friend std::ostream &operator<<(std::ostream &os, const RcData &rc) {
    os << "RC ~ " << static_cast<int>(rc.channel_count) << " channels: ";
    for (std::uint8_t i = 0; i < rc.channel_count; i++) {
      os << rc.channels[i];
      if (i < rc.channel_count - 1) {
        os << " ";
      }
    }
    return os;
  }
};

/**
 * @brief RC channel data to send via MSP_SET_RAW_RC.
 *
 * This structure holds RC channel values to be sent to the flight controller
 * (command 200). Used to override RC input, typically for autonomous flight.
 */
struct Channels {
  std::uint16_t roll;
  std::uint16_t pitch;
  std::uint16_t throttle;
  std::uint16_t yaw;
  std::uint16_t aux1;
  std::uint16_t aux2;
  std::uint16_t aux3;
  std::uint16_t aux4;
};

struct SetRawRcData {
  Channels channels;

  SetRawRcData() : channels{0, 0, 0, 0, 0, 0, 0, 0} {}

  SetRawRcData(const Channels &ch) : channels(ch) {}

  SetRawRcData(std::uint16_t roll, std::uint16_t pitch, std::uint16_t throttle,
               std::uint16_t yaw, std::uint16_t aux1 = 1000,
               std::uint16_t aux2 = 1000, std::uint16_t aux3 = 1000,
               std::uint16_t aux4 = 1000)
      : channels{roll, pitch, throttle, yaw, aux1, aux2, aux3, aux4} {}

  friend std::ostream &operator<<(std::ostream &os, const SetRawRcData &rc) {
    os << "SET_RAW_RC ~ roll=" << rc.channels.roll
       << ", pitch=" << rc.channels.pitch
       << ", throttle=" << rc.channels.throttle << ", yaw=" << rc.channels.yaw
       << ", aux1=" << rc.channels.aux1 << ", aux2=" << rc.channels.aux2
       << ", aux3=" << rc.channels.aux3 << ", aux4=" << rc.channels.aux4;
    return os;
  }
};

/**
 * @brief Flight controller status data from MSP_STATUS.
 *
 * This structure holds the parsed response from an MSP_STATUS request
 * (command 101). Contains cycle time, error counters, sensor flags, and system
 * load.
 */
struct StatusData {
  std::uint16_t cycle_time; ///< Task delta time in microseconds.
  std::uint16_t i2c_errors; ///< I2C error counter.
  std::uint16_t
      sensor_flags; ///< Sensor presence flags (ACC, BARO, MAG, GPS, etc.).
  std::uint32_t flight_mode_flags; ///< Flight mode flags (first 32 bits).
  std::uint8_t pid_profile;        ///< Current PID profile index.
  std::uint16_t system_load;       ///< Average system load percentage.

  StatusData(std::uint8_t recv_size, std::uint8_t *payload) {
    if (recv_size >= 13) {
      cycle_time = static_cast<uint16_t>(payload[0] | (payload[1] << 8));
      i2c_errors = static_cast<uint16_t>(payload[2] | (payload[3] << 8));
      sensor_flags = static_cast<uint16_t>(payload[4] | (payload[5] << 8));
      flight_mode_flags =
          static_cast<uint32_t>(payload[6] | (payload[7] << 8) |
                                (payload[8] << 16) | (payload[9] << 24));
      pid_profile = payload[10];
      system_load = static_cast<uint16_t>(payload[11] | (payload[12] << 8));
    } else {
      throw std::runtime_error("MSP_STATUS payload size " +
                               std::to_string(recv_size) +
                               " (expected >= 13)\n");
    }
  }

  friend std::ostream &operator<<(std::ostream &os, const StatusData &status) {
    os << "Status ~ cycle_time=" << status.cycle_time
       << " us, i2c_errors=" << status.i2c_errors << ", sensors=0x" << std::hex
       << status.sensor_flags << std::dec
       << ", pid_profile=" << static_cast<int>(status.pid_profile)
       << ", system_load=" << status.system_load << "%\n";

    os << "Active flight modes: ";
    bool first = true;
    for (int i = 0; i < 32; i++) {
      if (status.flight_mode_flags & (1u << i)) {
        if (!first) {
          os << ", ";
        }
        os << getBoxName(static_cast<BoxId>(i));
        first = false;
      }
    }
    if (first) {
      os << "none";
    }
    return os;
  }
};

/**
 * @brief Altitude and vertical velocity data from MSP_ALTITUDE.
 *
 * This structure holds the parsed response from an MSP_ALTITUDE request
 * (command 109). All units follow the Betaflight convention.
 */
struct AltitudeData {
  std::int32_t altitude; ///< Estimated altitude in centimeters.
  std::int16_t vario;    ///< Vertical velocity (variometer) in cm/s.

  AltitudeData(std::uint8_t recv_size, std::uint8_t *payload) {
    if (recv_size >= 6) {
      altitude = static_cast<int32_t>(payload[0] | (payload[1] << 8) |
                                      (payload[2] << 16) | (payload[3] << 24));
      vario = static_cast<int16_t>(payload[4] | (payload[5] << 8));
    } else {
      throw std::runtime_error("MSP_ALTITUDE payload size " +
                               std::to_string(recv_size) +
                               " (expected >= 6)\n");
    }
  }

  friend std::ostream &operator<<(std::ostream &os,
                                  const AltitudeData &altitude) {
    os << "Altitude: " << altitude.altitude << " cm, Vario: " << altitude.vario
       << " cm/s";
    return os;
  }
};

struct AttitudeData {
  std::int16_t roll_tenths;
  std::int16_t pitch_tenths;
  std::int16_t yaw_tenths;

  AttitudeData(std::uint8_t recv_size, std::uint8_t *payload) {
    if (recv_size >= 6) {
      roll_tenths = static_cast<int16_t>(payload[0] | (payload[1] << 8));
      pitch_tenths = static_cast<int16_t>(payload[2] | (payload[3] << 8));
      yaw_tenths = static_cast<int16_t>(payload[4] | (payload[5] << 8));
    } else {
      throw std::runtime_error("MSP_ATTITUDE payload size " +
                               std::to_string(recv_size) +
                               " (expected >= 6)\n");
    }
  }

  friend std::ostream &operator<<(std::ostream &os,
                                  const AttitudeData &attitude) {
    os << "Attitude ~ roll=" << (attitude.roll_tenths / 10.0)
       << "° pitch=" << (attitude.pitch_tenths / 10.0)
       << "° yaw=" << (attitude.yaw_tenths / 10.0) << "°";
    return os;
  }
};

/**
 * @brief High-level MSP client providing typed command methods.
 *
 * Wraps BitaflughtMsp to expose domain-specific MSP commands (altitude,
 * attitude, etc.) with structured return types. Each method sends the
 * corresponding MSP request and parses the binary response into a convenient
 * C++ struct.
 *
 * Operations use the underlying serial stream configured at construction time.
 * Methods throw std::runtime_error when the flight controller does not respond
 * or returns invalid data.
 */
class Msp {
public:
  Msp() = delete;

  /**
   * @brief Construct an MSP client bound to a POSIX serial device.
   *
   * Opens and configures the underlying serial stream (raw 8N1) and keeps it
   * ready for MSP I/O.
   *
   * @param dev       Serial device path (e.g., "/dev/ttyUSB0", "/dev/serial0").
   * @param baud_rate Termios baud constant (e.g., B115200).
   * @param timeout   Read timeout (VTIME, in deciseconds).
   *
   * @throws std::system_error if the serial stream cannot be opened/configured.
   */
  explicit Msp(const char *dev = DEFAULT_SERIAL_DEVICE,
               speed_t baud_rate = DEFAULT_BAUD_RATE,
               cc_t timeout = DEFAULT_TIMEOUT);

  /**
   * @brief Request flight controller status information.
   *
   * Behavior:
   * - Sends MSP_STATUS (command 101) request with no payload.
   * - Waits for a response containing cycle time, error counters, sensor flags,
   *   flight mode flags, PID profile, and system load.
   * - Parses the response into a StatusData struct.
   * - Throws std::runtime_error if the request times out, the response is
   * invalid, or the payload size is incorrect.
   *
   * @return StatusData on success.
   */
  [[nodiscard]] StatusData status();

  /**
   * @brief Request RC channel values from the flight controller.
   *
   * Behavior:
   * - Sends MSP_RC (command 105) request with no payload.
   * - Waits for a response containing RC channel values (uint16_t per channel).
   * - Parses the response into an RcData struct.
   * - Throws std::runtime_error if the request times out, the response is
   * invalid, or the payload size is incorrect.
   *
   * @return RcData on success.
   *
   * @note RC channel values are typically in the range [1000, 2000].
   */
  [[nodiscard]] RcData rc();

  /**
   * @brief Request altitude and vertical velocity from the flight controller.
   *
   * Behavior:
   * - Sends MSP_ALTITUDE (command 109) request with no payload.
   * - Waits for a 6-byte response:
   *   - bytes [0..3]: int32_t altitude (cm, little-endian)
   *   - bytes [4..5]: int16_t vario (cm/s, little-endian)
   * - Parses the response into an AltitudeData struct.
   * - Throws std::runtime_error if the request times out, the response is
   * invalid, or the payload size is incorrect.
   *
   * @return AltitudeData on success.
   *
   * @note Altitude is relative to the flight controller's initial position
   *       (typically armed at ground level). Vario is positive when ascending.
   */
  [[nodiscard]] AltitudeData altitude();

  [[nodiscard]] AttitudeData attitude();

  /**
   * @brief Send RC channel values to the flight controller.
   *
   * Behavior:
   * - Sends MSP_SET_RAW_RC (command 200) with RC channel values as payload.
   * - This overrides the RC input from the physical receiver.
   * - Typically used for autonomous flight or MSP-based RC control.
   * - Throws std::runtime_error if the send fails.
   *
   * @param data SetRawRcData containing channel count and channel values.
   *
   * @note Requires the flight controller to be compiled with USE_RX_MSP.
   * @note The MSPOVERRIDE flight mode may need to be active.
   */
  void setRawRc(const SetRawRcData &data);

private:
  BitaflughtMsp bitaflught_msp_;
};

} // namespace msp

#endif // !MSP_HPP
