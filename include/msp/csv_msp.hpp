#ifndef CSV_MSP_HPP
#define CSV_MSP_HPP

#include <termios.h>

#include <chrono>
#include <stdexcept>  // Added for std::runtime_error
#include <vector>

#include "bitaflught_msp.hpp"
#include "contracts/IMsp.hpp"
#include "msp_types.hpp"

namespace msp {

struct AltitudeRecord {
    double timestamp;
    double altitude;
    double vario;
};

struct AttitudeRecord {
    double timestamp;
    double roll;
    double pitch;
    double yaw;
};
/**
 * @brief High-level MSP client providing typed command methods.
 *
 * Wraps BitaflughtMsp to expose domain-specific MSP commands (altitude,
 * attitude, etc.) with structured return types. Each method sends the
 * corresponding MSP request and parses the binary response into a convenient
 * C++ struct.
 *
 * Operations use the underlying serial stream configured at construction time.
 * Methods throw std::runtime_error when the flight controller does not respond
 * or returns invalid data.
 */
class CsvMsp : public contracts::IMsp {
   public:
    CsvMsp() = delete;

    explicit CsvMsp(std::string altitude_data, std::string attitude_data, std::string raw_imu_data);
    /**
     * @brief Request flight controller status information.
     *
     * Behavior:
     * - Sends MSP_STATUS (command 101) request with no payload.
     * - Waits for a response containing cycle time, error counters, sensor
     * flags, flight mode flags, PID profile, and system load.
     * - Parses the response into a StatusData struct.
     * - Throws std::runtime_error if the request times out, the response is
     * invalid, or the payload size is incorrect.
     *
     * @return StatusData on success.
     */
    [[nodiscard]] StatusData status() override;

    /**
     * @brief Request RC channel values from the flight controller.
     *
     * Behavior:
     * - Sends MSP_RC (command 105) request with no payload.
     * - Waits for a response containing RC channel values (uint16_t per channel).
     * - Parses the response into an RcData struct.
     * - Throws std::runtime_error if the request times out, the response is
     * invalid, or the payload size is incorrect.
     *
     * @return RcData on success.
     *
     * @note RC channel values are typically in the range [1000, 2000].
     */
    [[nodiscard]] RcData rc() override;

    /**
     * @brief Request raw IMU sensor data from the flight controller.
     *
     * Behavior:
     * - Sends MSP_RAW_IMU (command 102) request with no payload.
     * - Waits for an 18-byte response containing:
     * - bytes [0..5]: accelerometer X, Y, Z (int16_t each, little-endian)
     * - bytes [6..11]: gyroscope X, Y, Z (int16_t each, little-endian)
     * - bytes [12..17]: magnetometer X, Y, Z (int16_t each, little-endian)
     * - Parses the response into a RawImuData struct.
     * - Throws std::runtime_error if the request times out, the response is
     * invalid, or the payload size is incorrect.
     *
     * @return RawImuData on success.
     *
     * @note Raw values are sensor-dependent and may require calibration.
     * Gyroscope values are typically in degrees/second.
     */
    [[nodiscard]] RawImuData rawImu() override;

    /**
     * @brief Send RC channel values to the flight controller.
     *
     * Behavior:
     * - Sends MSP_SET_RAW_RC (command 200) with RC channel values as payload.
     * - This overrides the RC input from the physical receiver.
     * - Typically used for autonomous flight or MSP-based RC control.
     * - Throws std::runtime_error if the send fails.
     *
     * @param data SetRawRcData containing channel count and channel values.
     *
     * @note Requires the flight controller to be compiled with USE_RX_MSP.
     * @note The MSPOVERRIDE flight mode may need to be active.
     */
    void setRawRc(const SetRawRcData& data) override;

    /**
     * @brief Request attitude (Roll, Pitch, Yaw) from the flight controller.
     * * @return AttitudeData on success.
     */
    [[nodiscard]] AttitudeData attitude() override;

    /**
     * @brief Request altitude and vertical velocity from the flight controller.
     *
     * Behavior:
     * - Sends MSP_ALTITUDE (command 109) request with no payload.
     * - Waits for a 6-byte response:
     * - bytes [0..3]: int32_t altitude (cm, little-endian)
     * - bytes [4..5]: int16_t vario (cm/s, little-endian)
     * - Parses the response into an AltitudeData struct.
     * - Throws std::runtime_error if the request times out, the response is
     * invalid, or the payload size is incorrect.
     *
     * @return AltitudeData on success.
     *
     * @note Altitude is relative to the flight controller's initial position
     * (typically armed at ground level). Vario is positive when ascending.
     */
    [[nodiscard]] AltitudeData altitude() override;

   private:
    std::chrono::steady_clock::time_point start;
    std::vector<AltitudeData> altitude_data;

    std::vector<AttitudeData> attitude_data;
    BitaflughtMsp bitaflught_msp_;

    AltitudeRecord interpolateAltitude(double timestamp) const;

    void load_altitude();

    void load_attitude();
    AttitudeRecord interpolateAttitude(double timestamp) const;
};

}  // namespace msp

#endif  // MSP_HPP
