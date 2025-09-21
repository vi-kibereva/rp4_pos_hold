#ifndef SERIAL_STREAM_HPP
#define SERIAL_STREAM_HPP

#include <cstddef>
#include <cstdint>

// for Linux
#include <termios.h>

namespace msp {

static constexpr uint32_t DEFAULT_BAUD_RATE = 115200;
static constexpr char DEFAULT_SERIAL_DEVICE[] = "/dev/serial0";
static constexpr cc_t DEFAULT_TIMEOUT = 2; // 0.2 seconds

class SerialStream {
public:
  SerialStream() = delete;

  /**
   * @brief Open and configure a POSIX serial device in raw 8N1 mode.
   *
   * Behavior:
   * - Opens @p dev with O_RDWR | O_NOCTTY | O_CLOEXEC.
   * - Disables input special handling & XON/XOFF; disables output
   * post-processing.
   * - Sets 8 data bits, no parity, 1 stop bit; disables RTS/CTS; enables
   * CLOCAL/CREAD.
   * - Non-canonical mode; no echo/signals/extensions (raw, binary-clean I/O).
   * - Read control: VMIN=1; VTIME=@p timeout (deciseconds).
   *   * timeout==0 → block until ≥1 byte;
   *   * timeout>0  → block for first byte up to timeout, then inter-byte
   * timeout.
   * - Sets both input/output baud to @p baud_rate; flushes I/O with
   * tcflush(TCIOFLUSH).
   *
   * @param dev       Serial device path (e.g., "/dev/ttyUSB0", "/dev/serial0").
   * @param baud_rate Termios baud constant (e.g., B115200).
   * @param timeout   VTIME in deciseconds (see behavior above).
   *
   * @throws std::system_error with original errno on failure to open or
   * configure the device.
   */
  explicit SerialStream(const char *dev = DEFAULT_SERIAL_DEVICE,
                        speed_t baud_rate = DEFAULT_BAUD_RATE,
                        cc_t timeout = DEFAULT_TIMEOUT);

  ~SerialStream();

  uint8_t read();
  size_t write(uint8_t data);
  void flush();
  uint8_t avaliable();

private:
  int serial_fd_;
};

} // namespace msp

#endif // !SERIAL_STREAM_HPP
