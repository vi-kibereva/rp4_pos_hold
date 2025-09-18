#ifndef SERIAL_IOSTREAM_HPP
#define SERIAL_IOSTREAM_HPP

#include <iostream>

#include "serial_streambuf.hpp"

namespace msp {

/**
 * @brief A convenient std::iostream that *owns* a serial_streambuf.
 *
 * Usage: construct, then use operator<< / operator>> like any iostream.
 * The underlying file descriptor and timeout are managed via thin
 * pass-throughs.
 *
 * Definitions live in the corresponding .cpp file.
 */
class serial_iostream : public std::iostream {
public:
  /**
   * @brief Default-construct an iostream with an internal, unopened buffer.
   *
   * Postcondition: rdbuf() points at the owned serial_streambuf.
   */
  serial_iostream();

  /**
   * @brief Construct and attach to an existing FD with an optional
   * timeout/buffer size.
   * @param fd          POSIX file descriptor (e.g., /dev/ttyUSB0 opened
   * elsewhere)
   * @param timeout_ms  −1 = block forever; 0 = non-blocking; >0 = millisecond
   * timeout
   * @param buf_sz      Size of the internal input buffer (excludes putback
   * window)
   */
  explicit serial_iostream(int fd, int timeout_ms = -1, size_t buf_sz = 4096);

  /// @return mutable pointer to the owned serial_streambuf
  serial_streambuf *rdbuf();

  /// @return const pointer to the owned serial_streambuf
  const serial_streambuf *rdbuf() const;

  /// Set the underlying file descriptor on the owned buffer.
  void set_fd(int fd);

  /// @return current file descriptor (−1 if none)
  int fd() const;

  /// Set the read/write poll timeout (see ctor for semantics).
  void set_timeout(int timeout_ms);

  /// @return current timeout in milliseconds
  int timeout() const;

private:
  serial_streambuf *buf_ptr(); // non-virtual helper to get typed rdbuf
  const serial_streambuf *buf_ptr() const; // const helper

  serial_streambuf &buf();             // reference to owned buffer
  const serial_streambuf &buf() const; // const reference

  serial_streambuf buf_; ///< Owned stream buffer instance
};

} // namespace msp

#endif
