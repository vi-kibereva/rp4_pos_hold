#include <cerrno>
#include <cstring>

#include <fcntl.h>
#include <sys/termios.h>
#include <termios.h>
#include <unistd.h>

#include "msp/serial_stream.hpp"
#include "utils.hpp"

namespace msp {

SerialStream::SerialStream(const char *dev, const speed_t baud_rate,
                           const cc_t timeout)
    : serial_fd_(::open(dev, O_RDWR | O_NOCTTY | O_CLOEXEC)) {
  if (serial_fd_ < 0)
    utils::throw_errno("while trying to open serial device <", dev, ">");

  // a helper function to close serial_fd_ on error
  auto fail = [&](auto &&...msg) -> void {
    const int e = errno;
    if (serial_fd_ >= 0) {
      ::close(serial_fd_);
    }
    errno = e;

    /// std::forward<decltype(msg)>(msg)... preserves the original value
    /// category (lvalue/rvalue)
    utils::throw_errno(std::forward<decltype(msg)>(msg)...);
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

} // namespace msp
