/**
 * @file pid.hpp
 * @brief PID Controller implementation for position hold control
 */

#ifndef PID_HPP
#define PID_HPP

#include <arm_neon.h>
#include <chrono>

/**
 * @class PidController
 * @brief PID (Proportional-Integral-Derivative) controller for 2D position
 * control
 *
 * This controller calculates RC PWM values based on position error between
 * current and desired positions. It uses NEON SIMD instructions for efficient
 * computation and applies derivative filtering to reduce noise. The controller
 * includes anti-windup protection for the integral term.
 *
 * @note The derivative term is calculated on measurement (not error) to avoid
 *       derivative kick when the setpoint changes.
 */
class PidController {
public:
  /**
   * @brief Default constructor
   */
  PidController() = default;

  /**
   * @brief Construct a new PID Controller with specified gains
   *
   * @param k_p Proportional gain - determines response to current error
   * @param k_i Integral gain - eliminates steady-state error
   * @param k_d Derivative gain - dampens oscillations and improves stability
   * @param k_df Derivative filter coefficient (0-1) - smooths derivative term
   * to reduce noise
   */
  PidController(float k_p, float k_i, float k_d, float k_df);

  /**
   * @brief Calculate raw RC PWM values from position error
   *
   * Computes PID control output and converts it to RC PWM values (1000-2000
   * microseconds). The controller processes X and Y axes independently and
   * outputs 4-channel RC values.
   *
   * @param current_position Current position as [x, y] in NEON float32x2_t
   * format
   * @param desired_position Desired position as [x, y], defaults to origin [0,
   * 0]
   * @return uint32x4_t Four RC channel PWM values (1500 is center, 1000-2000
   * range)
   */
  uint32x2_t calculate_raw_rc(float32x2_t current_position,
                              float32x2_t desired_position = {0, 0});

private:
  float k_p_ = 0.0f;  ///< Proportional gain
  float k_i_ = 0.0f;  ///< Integral gain
  float k_d_ = 0.0f;  ///< Derivative gain
  float k_df_ = 0.0f; ///< Derivative filter coefficient (low-pass filter)

  std::chrono::milliseconds
      last_time; ///< Timestamp of last calculation for dt computation

  float32x2_t last_value_ = {
      0, 0}; ///< Previous position measurement for derivative calculation
  float32x2_t filtered_derivative_ = {0,
                                      0}; ///< Low-pass filtered derivative term

  float32x2_t integral_ = {0.0f, 0.0f}; ///< Integral accumulator vector

  float32x2_t integral_min_ = {
      -100.0f, -100.0f}; ///< Minimum integral value for anti-windup
  float32x2_t integral_max_ = {
      100.0f, 100.0f}; ///< Maximum integral value for anti-windup
};

#endif
