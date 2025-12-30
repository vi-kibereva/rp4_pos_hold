#ifndef MSP_TYPES_HPP
#define MSP_TYPES_HPP
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <string>

#include "box_ids.hpp"
namespace msp {
/**
 * @brief MSP command identifiers for Betaflight/Cleanflight protocol.
 *
 * Each constant represents a specific message type in the MSP v1 protocol.
 * Used as command_id in send/request operations.
 */
enum MspCommand : std::uint8_t {
    MSP_API_VERSION = 1,
    MSP_STATUS = 101,
    MSP_RAW_IMU = 102,
    MSP_RC = 105,
    MSP_ATTITUDE = 108,
    MSP_ALTITUDE = 109,
    MSP_SET_RAW_RC = 200,
};

constexpr std::uint8_t MAX_RC_CHANNELS = 18;

/**
 * @brief RC channel data from MSP_RC.
 *
 * This structure holds the parsed response from an MSP_RC request
 * (command 105). Contains RC channel values, typically in range [1000, 2000].
 */
struct RcData {
    std::uint8_t channel_count;               ///< Number of channels received.
    std::uint16_t channels[MAX_RC_CHANNELS];  ///< RC channel values.

    RcData(std::uint8_t recv_size, std::uint8_t* payload) {
        if (recv_size < 2 || recv_size % 2 != 0) {
            throw std::runtime_error("MSP_RC payload size " + std::to_string(recv_size) +
                                     " invalid (expected even number >= 2)\n");
        }

        channel_count = recv_size / 2;
        if (channel_count > MAX_RC_CHANNELS) {
            channel_count = MAX_RC_CHANNELS;
        }

        for (std::uint8_t i = 0; i < channel_count; i++) {
            channels[i] = static_cast<uint16_t>(payload[i * 2] | (payload[i * 2 + 1] << 8));
        }
    }

    friend std::ostream& operator<<(std::ostream& os, const RcData& rc) {
        os << "RC ~ " << static_cast<int>(rc.channel_count) << " channels: ";
        for (std::uint8_t i = 0; i < rc.channel_count; i++) {
            os << rc.channels[i];
            if (i < rc.channel_count - 1) {
                os << " ";
            }
        }
        return os;
    }
};

/**
 * @brief RC channel data to send via MSP_SET_RAW_RC.
 *
 * This structure holds RC channel values to be sent to the flight controller
 * (command 200). Used to override RC input, typically for autonomous flight.
 */
struct Channels {
    std::uint16_t roll;
    std::uint16_t pitch;
    std::uint16_t throttle;
    std::uint16_t yaw;
    std::uint16_t aux1;
    std::uint16_t aux2;
    std::uint16_t aux3;
    std::uint16_t aux4;
};

struct SetRawRcData {
    Channels channels;

    SetRawRcData() : channels{0, 0, 0, 0, 0, 0, 0, 0} {}

    SetRawRcData(const Channels& ch) : channels(ch) {}

    SetRawRcData(std::uint16_t roll, std::uint16_t pitch, std::uint16_t throttle, std::uint16_t yaw,
                 std::uint16_t aux1 = 1000, std::uint16_t aux2 = 1000, std::uint16_t aux3 = 1000,
                 std::uint16_t aux4 = 1000)
        : channels{roll, pitch, throttle, yaw, aux1, aux2, aux3, aux4} {}

    friend std::ostream& operator<<(std::ostream& os, const SetRawRcData& rc) {
        os << "SET_RAW_RC ~ roll=" << rc.channels.roll << ", pitch=" << rc.channels.pitch
           << ", throttle=" << rc.channels.throttle << ", yaw=" << rc.channels.yaw
           << ", aux1=" << rc.channels.aux1 << ", aux2=" << rc.channels.aux2
           << ", aux3=" << rc.channels.aux3 << ", aux4=" << rc.channels.aux4;
        return os;
    }
};

/**
 * @brief Flight controller status data from MSP_STATUS.
 *
 * This structure holds the parsed response from an MSP_STATUS request
 * (command 101). Contains cycle time, error counters, sensor flags, and system
 * load.
 */
struct StatusData {
    std::uint16_t cycle_time;         ///< Task delta time in microseconds.
    std::uint16_t i2c_errors;         ///< I2C error counter.
    std::uint16_t sensor_flags;       ///< Sensor presence flags (ACC, BARO, MAG, GPS, etc.).
    std::uint32_t flight_mode_flags;  ///< Flight mode flags (first 32 bits).
    std::uint8_t pid_profile;         ///< Current PID profile index.
    std::uint16_t system_load;        ///< Average system load percentage.

    StatusData(std::uint8_t recv_size, std::uint8_t* payload) {
        if (recv_size >= 13) {
            cycle_time = static_cast<uint16_t>(payload[0] | (payload[1] << 8));
            i2c_errors = static_cast<uint16_t>(payload[2] | (payload[3] << 8));
            sensor_flags = static_cast<uint16_t>(payload[4] | (payload[5] << 8));
            flight_mode_flags = static_cast<uint32_t>(payload[6] | (payload[7] << 8) |
                                                      (payload[8] << 16) | (payload[9] << 24));
            pid_profile = payload[10];
            system_load = static_cast<uint16_t>(payload[11] | (payload[12] << 8));
        } else {
            throw std::runtime_error("MSP_STATUS payload size " + std::to_string(recv_size) +
                                     " (expected >= 13)\n");
        }
    }

    friend std::ostream& operator<<(std::ostream& os, const StatusData& status) {
        os << "Status ~ cycle_time=" << status.cycle_time << " us, i2c_errors=" << status.i2c_errors
           << ", sensors=0x" << std::hex << status.sensor_flags << std::dec
           << ", pid_profile=" << static_cast<int>(status.pid_profile)
           << ", system_load=" << status.system_load << "%\n";

        os << "Active flight modes: ";
        bool first = true;
        for (int i = 0; i < 32; i++) {
            if (status.flight_mode_flags & (1u << i)) {
                if (!first) {
                    os << ", ";
                }
                os << getBoxName(static_cast<BoxId>(i));
                first = false;
            }
        }
        if (first) {
            os << "none";
        }
        return os;
    }
};

/**
 * @brief Raw IMU sensor data from MSP_RAW_IMU.
 *
 * This structure holds the parsed response from an MSP_RAW_IMU request
 * (command 102). Contains raw accelerometer, gyroscope, and magnetometer
 * readings.
 */
struct RawImuData {
    std::int16_t acc_x;   ///< Accelerometer X-axis
    std::int16_t acc_y;   ///< Accelerometer Y-axis
    std::int16_t acc_z;   ///< Accelerometer Z-axis
    std::int16_t gyro_x;  ///< Gyroscope X-axis (deg/s)
    std::int16_t gyro_y;  ///< Gyroscope Y-axis (deg/s)
    std::int16_t gyro_z;  ///< Gyroscope Z-axis (deg/s)
    std::int16_t mag_x;   ///< Magnetometer X-axis
    std::int16_t mag_y;   ///< Magnetometer Y-axis
    std::int16_t mag_z;   ///< Magnetometer Z-axis

    RawImuData(std::uint8_t recv_size, std::uint8_t* payload) {
        if (recv_size < 18) {
            throw std::runtime_error("MSP_RAW_IMU payload size " + std::to_string(recv_size) +
                                     " (expected 18)\n");
        }

        acc_x = static_cast<int16_t>(payload[0] | (payload[1] << 8));
        acc_y = static_cast<int16_t>(payload[2] | (payload[3] << 8));
        acc_z = static_cast<int16_t>(payload[4] | (payload[5] << 8));
        gyro_x = static_cast<int16_t>(payload[6] | (payload[7] << 8));
        gyro_y = static_cast<int16_t>(payload[8] | (payload[9] << 8));
        gyro_z = static_cast<int16_t>(payload[10] | (payload[11] << 8));
        mag_x = static_cast<int16_t>(payload[12] | (payload[13] << 8));
        mag_y = static_cast<int16_t>(payload[14] | (payload[15] << 8));
        mag_z = static_cast<int16_t>(payload[16] | (payload[17] << 8));
    }

    friend std::ostream& operator<<(std::ostream& os, const RawImuData& imu) {
        os << "RAW_IMU ~ "
           << "acc[" << imu.acc_x << ", " << imu.acc_y << ", " << imu.acc_z << "], "
           << "gyro[" << imu.gyro_x << ", " << imu.gyro_y << ", " << imu.gyro_z << "], "
           << "mag[" << imu.mag_x << ", " << imu.mag_y << ", " << imu.mag_z << "]";
        return os;
    }
};

/**
 * @brief Altitude and vertical velocity data from MSP_ALTITUDE.
 *
 * This structure holds the parsed response from an MSP_ALTITUDE request
 * (command 109). All units follow the Betaflight convention.
 */
struct AltitudeData {
    std::int32_t altitude;  ///< Estimated altitude in centimeters.
    std::int16_t vario;     ///< Vertical velocity (variometer) in cm/s.

    AltitudeData(std::uint8_t recv_size, std::uint8_t* payload) {
        if (recv_size >= 6) {
            altitude = static_cast<int32_t>(payload[0] | (payload[1] << 8) | (payload[2] << 16) |
                                            (payload[3] << 24));
            vario = static_cast<int16_t>(payload[4] | (payload[5] << 8));
        } else {
            throw std::runtime_error("MSP_ALTITUDE payload size " + std::to_string(recv_size) +
                                     " (expected >= 6)\n");
        }
    }

    friend std::ostream& operator<<(std::ostream& os, const AltitudeData& altitude) {
        os << "Altitude: " << altitude.altitude << " cm, Vario: " << altitude.vario << " cm/s";
        return os;
    }
};

struct AttitudeData {
    std::int16_t roll_tenths;
    std::int16_t pitch_tenths;
    std::int16_t yaw_tenths;

    AttitudeData(std::uint8_t recv_size, std::uint8_t* payload) {
        if (recv_size >= 6) {
            roll_tenths = static_cast<int16_t>(payload[0] | (payload[1] << 8));
            pitch_tenths = static_cast<int16_t>(payload[2] | (payload[3] << 8));
            yaw_tenths = static_cast<int16_t>(payload[4] | (payload[5] << 8));
        } else {
            throw std::runtime_error("MSP_ATTITUDE payload size " + std::to_string(recv_size) +
                                     " (expected >= 6)\n");
        }
    }

    friend std::ostream& operator<<(std::ostream& os, const AttitudeData& attitude) {
        os << "Attitude ~ roll=" << (attitude.roll_tenths / 10.0)
           << "° pitch=" << (attitude.pitch_tenths / 10.0)
           << "° yaw=" << (attitude.yaw_tenths / 10.0) << "°";
        return os;
    }
};
}  // namespace msp

#endif  // !MSP_TYPES_HPP
