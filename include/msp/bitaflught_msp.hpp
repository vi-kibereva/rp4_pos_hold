#ifndef BITALUGHT_MSP_HPP
#define BITALUGHT_MSP_HPP

#include <cstdint>

#include <sys/termios.h>

#include "serial_stream.hpp"

namespace msp {

class BitaflughtMsp {
public:
  BitaflughtMsp() = delete;

  /**
   * @brief Construct an MSP client bound to a POSIX serial device.
   *
   * Opens and configures the underlying serial stream (raw 8N1) and keeps it
   * ready for MSP I/O.
   *
   * @param dev       Serial device path (e.g., "/dev/ttyUSB0").
   * @param baud_rate Termios baud constant (e.g., B115200).
   * @param timeout   Read timeout (VTIME, in deciseconds).
   *
   * @throws std::system_error if the serial stream cannot be opened/configured.
   */
  explicit BitaflughtMsp(const char *dev = DEFAULT_SERIAL_DEVICE,
                         speed_t baud_rate = DEFAULT_BAUD_RATE,
                         cc_t timeout = DEFAULT_TIMEOUT);

  ~BitaflughtMsp();

  /**
   * @brief Send a single MSP v1 command frame.
   *
   * Builds and writes a frame with layout:
   * `'$' 'M' '<' <size> <command_id> <payload[0..size-1]> <checksum>`
   *
   * - `<size>` is the payload length (0–255).
   * - `<checksum>` is the XOR of `<size> ^ <command_id> ^ each payload byte`.
   * - The function copies @p payload bytes into an internal buffer and performs
   *   a blocking write to the serial stream.
   *
   * @param command_id MSP command identifier.
   * @param payload    Pointer to payload bytes (may be nullptr when @p size ==
   * 0).
   * @param size       Number of payload bytes (0–255).
   *
   * @pre @p size <= 255.
   * @post Entire frame is written on success.
   *
   * @throws std::system_error if the underlying write fails.
   *
   * @note This sends an MSP **request**\command frame (direction '<'). For
   *       responses ('>') and error frames ('!'), see the receiver logic.
   */
  void send(std::uint8_t command_id, const void *payload, std::uint8_t size);

  void error(std::uint8_t command_id, void *payload, std::uint8_t size);
  void response(std::uint8_t command_id, void *payload, std::uint8_t size);
  bool recv(std::uint8_t *command_id, void *payload, std::uint8_t maxSize,
            std::uint8_t *recvSize);
  bool waitFor(std::uint8_t command_id, void *payload, std::uint8_t maxSize,
               std::uint8_t *recvSize = nullptr);
  bool request(std::uint8_t command_id, void *payload, std::uint8_t maxSize,
               std::uint8_t *recvSize = nullptr);
  bool command(std::uint8_t command_id, void *payload, std::uint8_t size,
               bool waitACK = true);

  void reset();
  bool getActiveModes(std::uint32_t *activeModes);

private:
  SerialStream stream_;
};

} // namespace msp

#endif // !BITALUGHT_MSP_HPP
