// g++ -std=c++20 -O2 -Wall -Wextra msp_min.cpp -o msp_min
// Usage: ./msp_min /dev/ttyUSB0   (or /dev/ttyAMA0, /dev/ttyS0, etc.)

#include <array>
#include <cerrno>
#include <cinttypes>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <stdexcept>
#include <string>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
#include <vector>

static int open_serial_8N2(const char *path, int baud = B115200) {
  int fd = ::open(path, O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0)
    throw std::runtime_error(std::string("open: ") + std::strerror(errno));

  // Put into blocking after open; we’ll still use poll() for timeouts.
  int flags = fcntl(fd, F_GETFL, 0);
  fcntl(fd, F_SETFL, flags & ~O_NONBLOCK);

  termios tio{};
  if (tcgetattr(fd, &tio) < 0)
    throw std::runtime_error(std::string("tcgetattr: ") + std::strerror(errno));

  cfmakeraw(&tio);
  // 8 data bits, no parity, 2 stop bits
  tio.c_cflag &= ~(PARENB | CSTOPB | CSIZE | CRTSCTS);
  tio.c_cflag |= (CS8 | CLOCAL | CREAD | CSTOPB); // CSTOPB set => 2 stop bits
  // No software flow control
  tio.c_iflag &= ~(IXON | IXOFF | IXANY);

  cfsetispeed(&tio, baud);
  cfsetospeed(&tio, baud);

  if (tcsetattr(fd, TCSANOW, &tio) < 0)
    throw std::runtime_error(std::string("tcsetattr: ") + std::strerror(errno));
  tcflush(fd, TCIOFLUSH);
  return fd;
}

// Poll+read exactly n bytes (or timeout_ms)
static bool read_exact(int fd, uint8_t *buf, size_t n, int timeout_ms) {
  size_t got = 0;
  while (got < n) {
    pollfd pfd{.fd = fd, .events = POLLIN, .revents = 0};
    int pr = ::poll(&pfd, 1, timeout_ms);
    if (pr == 0)
      return false; // timeout
    if (pr < 0)
      throw std::runtime_error("poll failed");
    if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL))
      return false;

    ssize_t r = ::read(fd, buf + got, n - got);
    if (r <= 0)
      return false;
    got += static_cast<size_t>(r);
  }
  return true;
}

// -------- MSP v1 framing helpers --------
// Frame: "$M<" + size(1) + cmd(1) + payload[size] + chksum(1)
// checksum = XOR of size, cmd, and all payload bytes
static void msp_send_v1(int fd, uint8_t cmd, const uint8_t *payload,
                        uint8_t size) {
  std::array<uint8_t, 3> hdr{'$', 'M', '<'};
  uint8_t sum = size ^ cmd;
  std::vector<uint8_t> frame;
  frame.reserve(3 + 2 + size + 1);
  frame.insert(frame.end(), hdr.begin(), hdr.end());
  frame.push_back(size);
  frame.push_back(cmd);
  for (uint8_t i = 0; i < size; ++i) {
    frame.push_back(payload[i]);
    sum ^= payload[i];
  }
  frame.push_back(sum);

  ssize_t w = ::write(fd, frame.data(), frame.size());
  if (w != static_cast<ssize_t>(frame.size()))
    throw std::runtime_error("short write");
  // Optional: wait until bytes are actually transmitted
  tcdrain(fd);
}

// Returns true and fills respPayload if a valid response to 'cmd' arrives
static bool msp_read_response_v1(int fd, uint8_t expected_cmd,
                                 std::vector<uint8_t> &respPayload,
                                 int timeout_ms) {
  // We expect "$M>" then size, cmd, payload, checksum
  uint8_t hdr[3];
  // Hunt for '$'
  for (;;) {
    if (!read_exact(fd, hdr, 1, timeout_ms))
      return false;
    if (hdr[0] == '$')
      break;
  }
  // Read 'M' '>'
  if (!read_exact(fd, hdr + 1, 2, timeout_ms))
    return false;
  if (hdr[1] != 'M' || hdr[2] != '>')
    return false;

  // size + cmd
  uint8_t sz_cmd[2];
  if (!read_exact(fd, sz_cmd, 2, timeout_ms))
    return false;
  uint8_t size = sz_cmd[0];
  uint8_t cmd = sz_cmd[1];

  respPayload.resize(size);
  if (size && !read_exact(fd, respPayload.data(), size, timeout_ms))
    return false;

  uint8_t recv_sum = 0;
  if (!read_exact(fd, &recv_sum, 1, timeout_ms))
    return false;

  uint8_t calc = size ^ cmd;
  for (auto b : respPayload)
    calc ^= b;

  if (calc != recv_sum)
    return false;
  if (cmd != expected_cmd)
    return false; // out-of-order/other packet

  return true;
}

int main(int argc, char **argv) {
  if (argc < 2) {
    std::fprintf(stderr, "Usage: %s /dev/ttyUSB0\n", argv[0]);
    return 2;
  }
  const char *port = argv[1];

  try {
    int fd = open_serial_8N2(port, B115200);

    // Example 1: MSP_API_VERSION (0x01), empty payload
    constexpr uint8_t MSP_API_VERSION = 0x01;
    msp_send_v1(fd, MSP_API_VERSION, nullptr, 0);

    std::vector<uint8_t> payload;
    if (!msp_read_response_v1(fd, MSP_API_VERSION, payload,
                              /*timeout_ms=*/1000)) {
      std::fprintf(stderr, "No/invalid response to MSP_API_VERSION\n");
      return 1;
    }

    if (payload.size() < 3) {
      std::fprintf(stderr, "Unexpected payload size %zu\n", payload.size());
      return 1;
    }
    uint8_t mspProtocol = payload[0]; // MSP protocol version (v1 => 0x01)
    uint8_t apiMajor = payload[1];
    uint8_t apiMinor = payload[2];

    std::printf("MSP protocol: %u, API: %u.%u\n", mspProtocol, apiMajor,
                apiMinor);

    // Example 2: read gyro-based attitude (legacy MSP_ATTITUDE = 0x64, 6 bytes:
    // int16_t roll,pitch,yaw in 0.1deg)
    constexpr uint8_t MSP_ATTITUDE = 0x64;
    msp_send_v1(fd, MSP_ATTITUDE, nullptr, 0);
    if (msp_read_response_v1(fd, MSP_ATTITUDE, payload, 1000) &&
        payload.size() >= 6) {
      auto rd16 = [](const uint8_t *p) { return int16_t(p[0] | (p[1] << 8)); };
      int16_t roll_tenths = rd16(&payload[0]);
      int16_t pitch_tenths = rd16(&payload[2]);
      int16_t yaw_tenths = rd16(&payload[4]);
      std::printf("Attitude ~ roll=%.1f° pitch=%.1f° yaw=%.1f°\n",
                  roll_tenths / 10.0, pitch_tenths / 10.0, yaw_tenths / 10.0);
    } else {
      std::fprintf(stderr, "MSP_ATTITUDE failed or not supported\n");
    }

    ::close(fd);
    return 0;
  } catch (const std::exception &ex) {
    std::fprintf(stderr, "Error: %s\n", ex.what());
    return 1;
  }
}
