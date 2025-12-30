#ifndef IMSP_HPP
#define IMSP_HPP

#include "msp/msp_types.hpp"

namespace contracts {

/**
 * @brief High-level MSP client interface providing typed command methods.
 *
 * Defines the contract for communicating with a flight controller via MSP.
 * Implementations should wrap the underlying serial communication and parsing
 * logic to return structured C++ types.
 */
class IMsp {
   public:
    virtual ~IMsp() = default;

    /**
     * @brief Request flight controller status information.
     * @return StatusData on success.
     */
    [[nodiscard]] virtual StatusData status() = 0;

    /**
     * @brief Request RC channel values from the flight controller.
     * @return RcData on success.
     */
    [[nodiscard]] virtual RcData rc() = 0;

    /**
     * @brief Request altitude and vertical velocity from the flight controller.
     * @return AltitudeData on success.
     */
    [[nodiscard]] virtual AltitudeData altitude() = 0;

    /**
     * @brief Request attitude (Roll, Pitch, Yaw) from the flight controller.
     * @return AttitudeData on success.
     */
    [[nodiscard]] virtual AttitudeData attitude() = 0;

    /**
     * @brief Request raw IMU sensor data from the flight controller.
     * @return RawImuData on success.
     */
    [[nodiscard]] virtual RawImuData rawImu() = 0;

    /**
     * @brief Send RC channel values to the flight controller.
     * @param data SetRawRcData containing channel count and channel values.
     */
    virtual void setRawRc(const SetRawRcData& data) = 0;
};

}  // namespace contracts

#endif  // !IMSP_HPP
