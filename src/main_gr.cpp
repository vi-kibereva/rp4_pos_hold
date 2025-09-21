// // msp_min.cpp
// // Minimal MSP v1 client with solid I/O and proper 8N1 serial config.
// // C++17-only, single-file, no deps.
//
// #include <array>
// #include <cerrno>
// #include <chrono>
// #include <cinttypes>
// #include <cstdint>
// #include <cstdio>
// #include <cstring>
// #include <exception>
// #include <fcntl.h>
// #include <poll.h>
// #include <stdexcept>
// #include <string>
// #include <string_view>
// #include <sys/ioctl.h>
// #include <sys/stat.h>
// #include <sys/types.h>
// #include <termios.h>
// #include <unistd.h>
// #include <vector>
//
// namespace util {
//
// struct unique_fd {
//   int fd{-1};
//   unique_fd() = default;
//   explicit unique_fd(int f) : fd(f) {}
//   unique_fd(const unique_fd &) = delete;
//   unique_fd &operator=(const unique_fd &) = delete;
//   unique_fd(unique_fd &&o) noexcept : fd(o.fd) { o.fd = -1; }
//   unique_fd &operator=(unique_fd &&o) noexcept {
//     if (this != &o) {
//       reset();
//       fd = o.fd;
//       o.fd = -1;
//     }
//     return *this;
//   }
//   ~unique_fd() { reset(); }
//   void reset(int f = -1) noexcept {
//     if (fd >= 0)
//       ::close(fd);
//     fd = f;
//   }
//   explicit operator bool() const noexcept { return fd >= 0; }
// };
//
// [[noreturn]] inline void throw_errno(const char *what) {
//   throw std::runtime_error(std::string(what) + ": " + std::strerror(errno));
// }
//
// inline int open_serial_8N1(const char *path, speed_t baud = B115200) {
//   int fd = ::open(path, O_RDWR | O_NOCTTY | O_CLOEXEC);
//   if (fd < 0)
//     throw_errno("open");
//
//   termios tio{};
//   if (::tcgetattr(fd, &tio) < 0) {
//     ::close(fd);
//     throw_errno("tcgetattr");
//   }
//
//   ::cfmakeraw(&tio);
//
//   // 8N1 (default MSP)
//   tio.c_cflag &= ~(PARENB | CSTOPB | CSIZE | CRTSCTS);
//   tio.c_cflag |= (CS8 | CLOCAL | CREAD); // 1 stop bit (no CSTOPB)
//
//   // Disable software flow control
//   tio.c_iflag &= ~(IXON | IXOFF | IXANY);
//
//   if (::cfsetispeed(&tio, baud) < 0) {
//     ::close(fd);
//     throw_errno("cfsetispeed");
//   }
//   if (::cfsetospeed(&tio, baud) < 0) {
//     ::close(fd);
//     throw_errno("cfsetospeed");
//   }
//
//   // Apply now
//   if (::tcsetattr(fd, TCSANOW, &tio) < 0) {
//     ::close(fd);
//     throw_errno("tcsetattr");
//   }
//   // Drop any stale bytes from both directions
//   ::tcflush(fd, TCIOFLUSH);
//
//   return fd;
// }
//
// // Poll with absolute deadline
// inline bool poll_until(int fd, short events,
//                        std::chrono::steady_clock::time_point deadline) {
//   for (;;) {
//     auto now = std::chrono::steady_clock::now();
//     if (now >= deadline)
//       return false;
//     auto ms =
//         std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now)
//             .count();
//     pollfd pfd{fd, events, 0};
//     int r = ::poll(&pfd, 1, static_cast<int>(ms));
//     if (r > 0) {
//       if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL))
//         return false;
//       return true;
//     }
//     if (r == 0)
//       return false; // timeout
//     if (errno == EINTR)
//       continue;
//     return false;
//   }
// }
//
// inline bool
// read_exact_deadline(int fd, uint8_t *buf, size_t n,
//                     std::chrono::steady_clock::time_point deadline) {
//   size_t got = 0;
//   while (got < n) {
//     if (!poll_until(fd, POLLIN, deadline))
//       return false;
//     ssize_t r = ::read(fd, buf + got, n - got);
//     if (r > 0) {
//       got += static_cast<size_t>(r);
//       continue;
//     }
//     if (r == 0)
//       return false; // EOF
//     if (errno == EINTR)
//       continue;
//     if (errno == EAGAIN || errno == EWOULDBLOCK)
//       continue;
//     return false;
//   }
//   return true;
// }
//
// inline bool write_all(int fd, const uint8_t *buf, size_t n) {
//   size_t sent = 0;
//   while (sent < n) {
//     ssize_t w = ::write(fd, buf + sent, n - sent);
//     if (w > 0) {
//       sent += static_cast<size_t>(w);
//       continue;
//     }
//     if (w < 0 && errno == EINTR)
//       continue;
//     return false;
//   }
//   return true;
// }
//
// inline int16_t le16(const uint8_t *p) {
//   return static_cast<int16_t>(static_cast<uint16_t>(p[0]) |
//                               (static_cast<uint16_t>(p[1]) << 8));
// }
//
// } // namespace util
//
// namespace msp {
//
// // MSP v1 constants
// constexpr uint8_t HDR_DOLLAR = '$';
// constexpr uint8_t HDR_M = 'M';
// constexpr uint8_t DIR_TO_FC = '<';
// constexpr uint8_t DIR_FROM_FC = '>';
// constexpr uint8_t DIR_ERROR = '!';
//
// struct FrameV1 {
//   uint8_t dir{};
//   uint8_t cmd{};
//   std::vector<uint8_t> payload;
//   bool checksum_ok{false};
//   bool is_error{false};
// };
//
// inline uint8_t checksum_v1(uint8_t size, uint8_t cmd, const uint8_t *payload) {
//   uint8_t sum = static_cast<uint8_t>(size ^ cmd);
//   for (uint8_t i = 0; i < size; ++i)
//     sum = static_cast<uint8_t>(sum ^ payload[i]);
//   return sum;
// }
//
// inline bool send_v1(int fd, uint8_t cmd, const uint8_t *payload, uint8_t size) {
//   // "$M<" + size + cmd + payload + chk
//   std::vector<uint8_t> buf;
//   buf.reserve(static_cast<size_t>(3 + 2 + size + 1));
//   buf.push_back(HDR_DOLLAR);
//   buf.push_back(HDR_M);
//   buf.push_back(DIR_TO_FC);
//   buf.push_back(size);
//   buf.push_back(cmd);
//   for (uint8_t i = 0; i < size; ++i)
//     buf.push_back(payload[i]);
//   buf.push_back(checksum_v1(size, cmd, payload));
//   if (!util::write_all(fd, buf.data(), buf.size()))
//     return false;
//   ::tcdrain(fd); // ensure bytes are out on the wire
//   return true;
// }
//
// // Read next *valid* frame (skips garbage and invalid checksums) before
// // deadline.
// inline bool read_next_frame_v1(int fd, FrameV1 &out,
//                                std::chrono::steady_clock::time_point deadline) {
//   // Hunt for '$'
//   uint8_t b = 0;
//   for (;;) {
//     if (!util::read_exact_deadline(fd, &b, 1, deadline))
//       return false;
//     if (b == HDR_DOLLAR)
//       break;
//     // else keep scanning
//   }
//
//   // Expect 'M' and dir
//   uint8_t md[2]{};
//   if (!util::read_exact_deadline(fd, md, 2, deadline))
//     return false;
//   if (md[0] != HDR_M || (md[1] != DIR_FROM_FC && md[1] != DIR_ERROR)) {
//     // Not a valid start; restart search from current stream position
//     return read_next_frame_v1(fd, out, deadline);
//   }
//   const uint8_t dir = md[1];
//
//   // size + cmd
//   uint8_t sz_cmd[2]{};
//   if (!util::read_exact_deadline(fd, sz_cmd, 2, deadline))
//     return false;
//   const uint8_t size = sz_cmd[0];
//   const uint8_t cmd = sz_cmd[1];
//
//   std::vector<uint8_t> payload;
//   payload.resize(size);
//   if (size > 0) {
//     if (!util::read_exact_deadline(fd, payload.data(), size, deadline))
//       return false;
//   }
//
//   uint8_t recv_sum = 0;
//   if (!util::read_exact_deadline(fd, &recv_sum, 1, deadline))
//     return false;
//
//   const uint8_t calc = checksum_v1(size, cmd, payload.data());
//   out.dir = dir;
//   out.cmd = cmd;
//   out.payload = std::move(payload);
//   out.checksum_ok = (calc == recv_sum);
//   out.is_error = (dir == DIR_ERROR);
//   if (!out.checksum_ok) {
//     // Bad frame — keep looking for the next good one.
//     return read_next_frame_v1(fd, out, deadline);
//   }
//   return true;
// }
//
// // Read until we get a response for expected_cmd (or timeout). Skips unrelated
// // frames.
// inline bool read_response_for(int fd, uint8_t expected_cmd,
//                               std::vector<uint8_t> &outPayload,
//                               std::chrono::milliseconds timeout) {
//   const auto deadline = std::chrono::steady_clock::now() + timeout;
//   FrameV1 f{};
//   while (util::poll_until(fd, POLLIN, deadline)) {
//     if (!read_next_frame_v1(fd, f, deadline))
//       return false;
//     if (f.is_error && f.cmd == expected_cmd)
//       return false;
//     if (!f.is_error && f.dir == DIR_FROM_FC && f.cmd == expected_cmd) {
//       outPayload = std::move(f.payload);
//       return true;
//     }
//     // else ignore and keep waiting within deadline
//   }
//   return false;
// }
//
// } // namespace msp
//
// int main(int argc, char **argv) {
//   if (argc < 2) {
//     std::fprintf(stderr, "Usage: %s /dev/ttyUSB0\n", argv[0]);
//     return 2;
//   }
//   const char *port = argv[1];
//
//   try {
//     util::unique_fd s(util::open_serial_8N1(port, B115200));
//
//     // --- Example 1: MSP_API_VERSION (0x01) ---
//     constexpr uint8_t MSP_API_VERSION = 0x01;
//     if (!msp::send_v1(s.fd, MSP_API_VERSION, nullptr, 0)) {
//       std::fprintf(stderr, "write MSP_API_VERSION failed\n");
//       return 1;
//     }
//
//     std::vector<uint8_t> payload;
//     if (!msp::read_response_for(s.fd, MSP_API_VERSION, payload,
//                                 std::chrono::milliseconds(1000))) {
//       std::fprintf(stderr, "No/invalid response to MSP_API_VERSION\n");
//       return 1;
//     }
//     if (payload.size() < 3) {
//       std::fprintf(stderr, "MSP_API_VERSION payload too small: %zu\n",
//                    payload.size());
//       return 1;
//     }
//     const uint8_t mspProtocol = payload[0];
//     const uint8_t apiMajor = payload[1];
//     const uint8_t apiMinor = payload[2];
//     std::printf("MSP protocol: %u, API: %u.%u\n", mspProtocol, apiMajor,
//                 apiMinor);
//
//     // --- Example 2: MSP_ATTITUDE ---
//     constexpr uint8_t MSP_ATTITUDE = 109;
//     if (!msp::send_v1(s.fd, MSP_ATTITUDE, nullptr, 0)) {
//       std::fprintf(stderr, "write MSP_ATTITUDE failed\n");
//       return 1;
//     }
//     if (msp::read_response_for(s.fd, MSP_ATTITUDE, payload,
//                                std::chrono::milliseconds(1000))) {
//       if (payload.size() >= 6) {
//         const int16_t roll_tenths = util::le16(&payload[0]);
//         const int16_t pitch_tenths = util::le16(&payload[2]);
//         const int16_t yaw_tenths = util::le16(&payload[4]);
//         std::printf("Attitude ~ roll=%.1f° pitch=%.1f° yaw=%.1f°\n",
//                     roll_tenths / 10.0, pitch_tenths / 10.0, yaw_tenths / 10.0);
//       } else {
//         std::fprintf(stderr, "MSP_ATTITUDE payload size %zu (expected >= 6)\n",
//                      payload.size());
//       }
//     } else {
//       std::fprintf(stderr, "MSP_ATTITUDE failed or not supported\n");
//     }
//
//     return 0;
//   } catch (const std::exception &ex) {
//     std::fprintf(stderr, "Error: %s\n", ex.what());
//     return 1;
//   }
// }

#include <cstdio>
#include <cstdint>
#include <vector>
#include "msp/bitaflught_msp.hpp"

int main(int argc, char **argv) {
    if (argc < 2) {
        std::fprintf(stderr, "Usage: %s /dev/ttyUSB0\n", argv[0]);
        return 2;
    }

    const char *port = argv[1];

    try {
        // Construct MSP client
        msp::BitaflughtMsp msp(port, B115200, 10);

        // --- Example 1: MSP_API_VERSION ---
        constexpr uint8_t MSP_API_VERSION = 0x01;
        std::vector<uint8_t> payload(3); // to receive response
        std::uint8_t recv_size = 0;

        if (!msp.request(MSP_API_VERSION, payload.data(), static_cast<uint8_t>(payload.size()), &recv_size)) {
            std::fprintf(stderr, "No/invalid response to MSP_API_VERSION\n");
            return 1;
        }

        if (recv_size < 3) {
            std::fprintf(stderr, "MSP_API_VERSION payload too small: %u\n", recv_size);
            return 1;
        }

        const uint8_t mspProtocol = payload[0];
        const uint8_t apiMajor = payload[1];
        const uint8_t apiMinor = payload[2];
        std::printf("MSP protocol: %u, API: %u.%u\n", mspProtocol, apiMajor, apiMinor);

        // --- Example 2: MSP_ATTITUDE ---
        constexpr uint8_t MSP_ATTITUDE = 109;
        payload.resize(6); // attitude returns 6 bytes
        if (msp.request(MSP_ATTITUDE, payload.data(), static_cast<uint8_t>(payload.size()), &recv_size)) {
            if (recv_size >= 6) {
                const int16_t roll_tenths = static_cast<int16_t>(payload[0] | (payload[1] << 8));
                const int16_t pitch_tenths = static_cast<int16_t>(payload[2] | (payload[3] << 8));
                const int16_t yaw_tenths = static_cast<int16_t>(payload[4] | (payload[5] << 8));
                std::printf("Attitude ~ roll=%.1f° pitch=%.1f° yaw=%.1f°\n",
                            roll_tenths / 10.0, pitch_tenths / 10.0, yaw_tenths / 10.0);
            } else {
                std::fprintf(stderr, "MSP_ATTITUDE payload size %u (expected >= 6)\n", recv_size);
            }
        } else {
            std::fprintf(stderr, "MSP_ATTITUDE failed or not supported\n");
        }

        return 0;

    } catch (const std::exception &ex) {
        std::fprintf(stderr, "Error: %s\n", ex.what());
        return 1;
    }
}

