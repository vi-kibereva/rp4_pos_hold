#ifndef SERIAL_STREAMBUF_HPP
#define SERIAL_STREAMBUF_HPP

#include <streambuf>
#include <vector>

namespace msp {

/**
 * @brief A POSIX FD-backed stream buffer that adapts a serial device
 *        (e.g., /dev/serial0, /dev/ttyUSB0) to C++ iostreams.
 *
 * Behavior highlights:
 *  - Input uses an internal get area with a small putback window so peek/unget
 * work.
 *  - Reads block using poll() up to timeout_ms (−1 = block forever, 0 =
 * non-blocking).
 *  - Writes push directly to the FD; xsputn() performs bulk writes.
 *  - sync() drains TX with tcdrain().
 *
 * Note: Definitions should live in a corresponding .cpp file.
 */
class serial_streambuf : public std::streambuf {
public:
  /**
   * @param fd          Existing POSIX file descriptor, or −1 if not yet set.
   * @param timeout_ms  Read/write wait timeout in milliseconds.
   *                    −1 = block forever; 0 = non-blocking; >0 = finite
   * timeout.
   * @param buf_sz      Size of the main input buffer (excludes putback window).
   *
   * Initializes internal buffers and sets the get area to an empty state.
   * Does not take ownership of configuring termios; do that externally.
   */
  explicit serial_streambuf(int fd = -1, int timeout_ms = -1,
                            size_t buf_sz = 4096);

  /// Set the underlying file descriptor (no close performed by this class).
  void set_fd(int fd);

  /// Get the current file descriptor (−1 if none).
  int fd() const;

  /// Set the poll timeout in milliseconds (see constructor for semantics).
  void set_timeout(int timeout_ms);

  /// Get the current timeout in milliseconds.
  int timeout() const;

protected:
  /**
   * @brief Refill the get area from the FD when it’s empty.
   *
   * Implementation notes (in .cpp):
   *  - If buffer still has data, return current char.
   *  - If fd < 0, return EOF.
   *  - Move up to putback_ chars into the putback window.
   *  - poll(POLLIN) with timeout_ms_, then read() into buffer.
   *  - Reset get pointers via setg() and return next char or EOF on failure.
   */
  int_type underflow() override;

  /**
   * @brief Write a single character to the FD when put area overflows
   *        (or for unbuffered writes).
   *
   * Implementation notes:
   *  - If ch is EOF, report success (not_eof) without writing.
   *  - Otherwise write() one byte; on success return ch, else EOF.
   */
  int_type overflow(int_type ch) override;

  /**
   * @brief Optimized bulk write to the FD.
   *
   * Implementation notes:
   *  - Loop until all n bytes written or error.
   *  - Handle EINTR; on EAGAIN/EWOULDBLOCK, poll(POLLOUT) up to timeout_ms_.
   *  - Return the number of bytes successfully written.
   */
  std::streamsize xsputn(const char *s, std::streamsize n) override;

  /**
   * @brief Flush pending output; drain TX.
   *
   * Implementation notes:
   *  - Call tcdrain(fd_) if fd_ is valid.
   *  - Return 0 on success (as per std::streambuf contract).
   */
  int sync() override;

private:
  // --- State configured at construction / via setters ---
  int fd_ = -1;         ///< POSIX file descriptor for the serial device.
  int timeout_ms_ = -1; ///< −1: block forever; 0: non-blocking; >0: ms timeout.

  // --- Input buffering (get area) ---
  std::vector<char> inbuf_; ///< Main read buffer (size = buf_sz).
  size_t putback_ = 0;      ///< Size of the putback window (min(64, buf_sz/4)).
  std::vector<char> inbuf_effective_; ///< putback_ + inbuf_ contiguous storage
                                      ///< used by setg().
};

} // namespace msp

#endif // !SERIAL_STREAMBUF_HPP
