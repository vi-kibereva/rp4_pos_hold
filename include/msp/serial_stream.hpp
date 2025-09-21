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

/**
 * @brief RAII wrapper for a POSIX serial port.
 *
 * Opens a device path and configures it for raw 8N1 (non-canonical, no
 * echo/flow control), providing binary-clean read/write and a tcdrain()-based
 * flush. Operations throw std::system_error on failure (preserving errno). On
 * destruction, pending output is drained and the file descriptor is closed;
 * destructor errors are ignored.
 */
class SerialStream {
public:
  SerialStream() = delete;

  /**
   * @brief Open and configure a POSIX serial device in raw 8N1 mode.
   *
   * Behavior:
   * - Opens @p dev with O_RDWR | O_NOCTTY | O_CLOEXEC.
   * - Disables input special handling; disables output post-processing.
   * - Sets 8 data bits, no parity, 1 stop bit;
   * - Non-canonical mode; raw I/O.
   * - Read control: VMIN=1; VTIME=@p timeout (deciseconds).
   *   * timeout==0 -> block until >= 1 byte;
   *   * timeout>0  -> block until >= 1 byte or timeout;
   * timeout.
   * - Sets both I/O baud to @p baud_rate; flushes I/O with tcflush(TCIOFLUSH).
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

  /**
   * @brief Flush pending output and close the serial file descriptor.
   *
   * Error handling:
   * - All errors from ::tcdrain() and ::close() are ignored.
   *
   * @post serial_fd_ == -1.
   */
  ~SerialStream() noexcept;

  /**
   * @brief Read bytes from the serial device.
   *
   * Behavior:
   * - Attempts to read up to @p size bytes into @p buffer via ::read().
   * - Retries on EINTR.
   * - Treats EAGAIN/EWOULDBLOCK as “no data” and returns 0.
   * - May return fewer than @p size bytes depending on VMIN/VTIME and driver.
   *
   * @param buffer Destination buffer (valid for @p size bytes).
   * @param size   Maximum number of bytes to read.
   * @return Number of bytes actually read (0 = no data/timeout/EOF).
   * @throws std::system_error on other read errors (preserves errno).
   */
  [[nodiscard]] size_t read(uint8_t *buffer, size_t size);

  /**
   * @brief Write a buffer to the serial device (blocking until all bytes are
   * sent).
   *
   * Behavior:
   * - Repeatedly calls ::write() until @p size bytes are written.
   * - Retries on EINTR.
   *
   * @param data Pointer to bytes to send.
   * @param size Number of bytes to send.
   * @return Number of bytes written (equals @p size on success).
   * @throws std::system_error on write errors other than EINTR (preserves
   * errno).
   * @note If the FD is non-blocking, consider handling EAGAIN/EWOULDBLOCK
   *       (e.g., with poll/select) to avoid busy looping or premature failure.
   */
  size_t write(uint8_t *data, size_t size);

  /**
   * @brief Block until all pending output is transmitted.
   *
   * Behavior:
   * - Calls ::tcdrain() on the underlying file descriptor.
   *
   * @throws std::system_error if ::tcdrain() fails (preserves errno).
   */
  void flush();

  /**
   * @brief Return the number of bytes currently readable without blocking.
   * Behavior:
   * - Queries the kernel RX queue via ::ioctl(FIONREAD) and returns the byte
   * count.
   * - The value may change before a subsequent read; use poll/select for
   * readiness.
   *
   * @return Bytes available to read immediately (0 if none).
   * @throws std::system_error if ::ioctl(FIONREAD, ...) fails (preserves
   * errno).
   *
   * @note This does not consume data and is independent of blocking mode.
   */
  size_t available();

private:
  int serial_fd_;
};

} // namespace msp

#endif // !SERIAL_STREAM_HPP
