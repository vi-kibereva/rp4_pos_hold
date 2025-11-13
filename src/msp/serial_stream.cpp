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

SerialStream::SerialStream(const char *dev, const speed_t baud_rate, const cc_t timeout) :
		serial_fd_(::open(dev, O_RDWR | O_NOCTTY | O_CLOEXEC)) {
	if (serial_fd_ < 0) {
		auto e = errno;
		utils::throw_errno(e, "while trying to open serial device <", dev, ">");
	}

	auto fail = [&](auto &&...msg) -> void {
		const int e = errno;
		if (serial_fd_ >= 0) {
			::close(serial_fd_);
		}
		
		utils::throw_errno(e, std::forward<decltype(msg)>(msg)...);
	};

	struct termios tty;
	if (::tcgetattr(serial_fd_, &tty) != 0)
		fail("Error calling tcgetattr");

	tty.c_iflag &= ~(IXON | IXOFF | IXANY);
	tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

	tty.c_oflag &= ~OPOST;
	tty.c_oflag &= ~ONLCR;

	tty.c_cflag &= ~PARENB;
	tty.c_cflag &= ~CSTOPB;

	tty.c_cflag &= ~CSIZE;
	tty.c_cflag |= CS8;

	tty.c_cflag &= ~CRTSCTS;
	tty.c_cflag |= CLOCAL;
	tty.c_cflag |= CREAD;

	tty.c_lflag &= ~ICANON;
	tty.c_lflag &= ~ECHO;
	tty.c_lflag &= ~ECHOE;
	tty.c_lflag &= ~ECHONL;
	tty.c_lflag &= ~ISIG;
	tty.c_lflag &= ~IEXTEN;

	tty.c_cc[VTIME] = timeout;

	tty.c_cc[VMIN] = 1;

	if (::cfsetispeed(&tty, baud_rate) != 0)
		fail("Error setting input baud rate with cfsetispeed");
	if (::cfsetospeed(&tty, baud_rate) != 0)
		fail("Error setting output baud rate with cfsetospeed");

	if (::tcsetattr(serial_fd_, TCSANOW, &tty) != 0)
		fail("Error setting attributes with tcsetattr");

	if (::tcflush(serial_fd_, TCIOFLUSH))
		fail("Error error flushing input and output with tcflush");
}

SerialStream::~SerialStream() noexcept {
	if (serial_fd_ >= 0) {
		::tcdrain(serial_fd_);
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
			continue;
		case EAGAIN:
			return 0;
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
			continue;
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

}
