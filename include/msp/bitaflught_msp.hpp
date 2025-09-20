#ifndef BITALUGHT_MSP_HPP
#define BITALUGHT_MSP_HPP

#include <cstdint>

#include "serial_stream.hpp"

namespace msp {

class BitaflughtMsp {
public:
  BitaflughtMsp() = delete;
  BitaflughtMsp(uint32_t timeout = 500);

  ~BitaflughtMsp();

  void send(uint8_t messageID, void *payload, uint8_t size);
  void error(uint8_t messageID, void *payload, uint8_t size);
  void response(uint8_t messageID, void *payload, uint8_t size);
  bool recv(uint8_t *messageID, void *payload, uint8_t maxSize,
            uint8_t *recvSize);
  bool waitFor(uint8_t messageID, void *payload, uint8_t maxSize,
               uint8_t *recvSize = nullptr);
  bool request(uint8_t messageID, void *payload, uint8_t maxSize,
               uint8_t *recvSize = nullptr);
  bool command(uint8_t messageID, void *payload, uint8_t size,
               bool waitACK = true);

  void reset();
  bool getActiveModes(uint32_t *activeModes);

private:
  SerialStream stream_;
  uint32_t timeout_;
};

} // namespace msp

#endif // !BITALUGHT_MSP_HPP
