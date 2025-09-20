#ifndef SERIAL_STREAM_HPP
#define SERIAL_STREAM_HPP

#include <cstddef>
#include <cstdint>

namespace msp {

static constexpr uint32_t DEFAULT_BAUDRATE = 115200;

class SerialStream {
public:
  SerialStream();
  SerialStream(uint32_t baudrate);

  ~SerialStream();

  uint8_t read();
  size_t write(uint8_t data);
  void flush();
  uint8_t avaliable();
};

} // namespace msp

#endif // !SERIAL_STREAM_HPP
