#ifndef BITALUGHT_MSP_HPP
#define BITALUGHT_MSP_HPP

#include <cstdint>

#include <sys/termios.h>

#include "serial_stream.hpp"

namespace msp {

class BitaflughtMsp {
public:
  BitaflughtMsp() = delete;
  explicit BitaflughtMsp(const char *dev = DEFAULT_SERIAL_DEVICE,
                         speed_t baud_rate = DEFAULT_BAUD_RATE,
                         cc_t timeout = DEFAULT_TIMEOUT);

  ~BitaflughtMsp();

  void send(std::uint8_t messageID, void *payload, std::uint8_t size);
  void error(std::uint8_t messageID, void *payload, std::uint8_t size);
  void response(std::uint8_t messageID, void *payload, std::uint8_t size);
  bool recv(std::uint8_t *messageID, void *payload, std::uint8_t maxSize,
            std::uint8_t *recvSize);
  bool waitFor(std::uint8_t messageID, void *payload, std::uint8_t maxSize,
               std::uint8_t *recvSize = nullptr);
  bool request(std::uint8_t messageID, void *payload, std::uint8_t maxSize,
               std::uint8_t *recvSize = nullptr);
  bool command(std::uint8_t messageID, void *payload, std::uint8_t size,
               bool waitACK = true);

  size_t reset();
  bool getActiveModes(std::uint32_t *activeModes);

private:
  SerialStream stream_;
    cc_t timeout_;
    uint8_t * buffer_;
};

} // namespace msp

#endif // !BITALUGHT_MSP_HPP
