#include <cerrno>
#include <cstddef>
#include <cstring>

#include <fcntl.h>
#include <iostream>
#include <ostream>
#include <sys/ioctl.h>
#include <sys/termios.h>
#include <termios.h>
#include <unistd.h>

#include "msp/serial_stream.hpp"
#include "utils.hpp"

namespace msp {

SerialStream::SerialStream(const char *dev, const speed_t baud_rate,
                           const cc_t timeout)
    : serial_fd_(::open(dev, O_RDWR | O_NOCTTY | O_CLOEXEC)) {
  if (serial_fd_ < 0) {
    auto e = errno;
    utils::throw_errno(e, "while trying to open serial device <", dev, ">");
  }

  // a helper function to close serial_fd_ on error
  auto fail = [&](auto &&...msg) -> void {
    const int e = errno;
    if (serial_fd_ >= 0) {
      ::close(serial_fd_);
    }

    /// std::forward<decltype(msg)>(msg)... preserves the original value
    /// category (lvalue/rvalue)
    utils::throw_errno(e, std::forward<decltype(msg)>(msg)...);
  };

  struct termios tty;
  if (::tcgetattr(serial_fd_, &tty) != 0)
    fail("Error calling tcgetattr");

  // --- Input flags ---
  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // diable software flow control
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR |
                   ICRNL); // disable special handling of input data

  // --- Output flags ---
  tty.c_oflag &= ~OPOST; // disable output post-processing
  tty.c_oflag &= ~ONLCR; // disable converision of newline to carriage return

  // --- Control flags ---
  tty.c_cflag &= ~PARENB; // disable parity
  tty.c_cflag &= ~CSTOPB; // 1 stop bit

  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8; // 8 data bits

  tty.c_cflag &= ~CRTSCTS; // disable RTS/CTS
  tty.c_cflag |= CLOCAL;   // ignore modem-specific control lines
  tty.c_cflag |= CREAD;    // allow read

  // --- Local flags ---
  tty.c_lflag &= ~ICANON; // disable canonical mode
  tty.c_lflag &= ~ECHO;   // disable echo
  tty.c_lflag &= ~ECHOE;  // disable erasing
  tty.c_lflag &= ~ECHONL; // disable new-line echo
  tty.c_lflag &= ~ISIG;   // disable signals
  tty.c_lflag &= ~IEXTEN; // disable DISCARD and LNEXT

  // --- Control characters ---
  tty.c_cc[VTIME] = timeout; // if > 0: blocking read with timeout,
                             // if == 0: return immeditely

  tty.c_cc[VMIN] = 1; // at least one byte

  // --- Baud rate ---
  if (::cfsetispeed(&tty, baud_rate) != 0)
    fail("Error setting input baud rate with cfsetispeed");
  if (::cfsetospeed(&tty, baud_rate) != 0)
    fail("Error setting output baud rate with cfsetospeed");

  if (::tcsetattr(serial_fd_, TCSANOW, &tty) != 0)
    fail("Error setting attributes with tcsetattr");

  if (::tcflush(serial_fd_, TCIOFLUSH)) // flush both input and output
    fail("Error error flushing input and output with tcflush");
}

SerialStream::~SerialStream() noexcept {
  if (serial_fd_ >= 0) {
    ::tcdrain(serial_fd_); // ensure all queued output is transmitted
    ::close(serial_fd_);

    serial_fd_ = -1;
  }
}

size_t SerialStream::read(std::uint8_t *buffer, size_t size) {
  ssize_t n = 0;
  for (;;) {
    n += ::read(serial_fd_, buffer+n, size-n);

    const int e = errno;

    if (n >= size)
      return static_cast<std::size_t>(n);

    switch (e) {
    case EINTR:
      continue; // retry if interrupted
    case EAGAIN:
      return 0; // timeouts/nonblocking as "no data"
    default:
      utils::throw_errno(e, "Error reading serial input with read");
    }
  }
}

size_t SerialStream::write(std::uint8_t *data, size_t size) {
  size_t sent = 0;
  while (sent < size) {
    ssize_t result = ::write(serial_fd_, data + sent, size - sent);
    const int e = errno;

    if (result >= 0) {
      sent += result;
      continue;
    }

    switch (e) {
    case EINTR:
      continue; // retry if interrupted
    default:
      utils::throw_errno(e, "Error reading serial input with read");
    }
  }

  return sent;
}

void SerialStream::flush() {
  if (::tcdrain(serial_fd_) != 0)
    utils::throw_errno(errno, "tcdrain failed");
}

size_t SerialStream::available() {
  int n;
  if (::ioctl(serial_fd_, FIONREAD, &n) != 0)
    utils::throw_errno(errno,
                       "Error getting available bytes for read with ioctl");

  return static_cast<size_t>(n);
}

} // namespace msp
