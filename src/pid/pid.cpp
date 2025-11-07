#include "pid/pid.hpp"
#include <algorithm>
#include <arm_neon.h>

PidController::PidController(float k_p, float k_i, float k_d, float k_df)
    : k_p_(k_p), k_i_(k_i), k_d_(k_d), k_df_(k_df) {
  last_time = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now().time_since_epoch());
}

uint32x2_t PidController::caldulate_raw_rc(float32x2_t current_position,
                                           float32x2_t desired_position) {
  auto current_time = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now().time_since_epoch());

  float dt = (current_time - last_time).count();
  last_time = current_time;

  // Calculate error: desired - current (for both x and y)
  float32x2_t error = vsub_f32(desired_position, current_position);

  // === INTEGRAL TERM (with anti-windup) ===
  integral_ = vfma_n_f32(integral_, error, dt);

  // Apply anti-windup clamping
  integral_ = vmin_f32(vmax_f32(integral_, integral_min_), integral_max_);

  float32x2_t i_term = vmul_n_f32(integral_, k_i_);

  // === DERIVATIVE TERM (derivative on measurement, not error) ===
  // This avoids "derivative kick" when setpoint changes

  // Raw derivative
  float32x2_t derivative_raw =
      vmul_n_f32(vsub_f32(last_value_, current_position), 1.0f / dt);

  // Apply low-pass filter to derivative (exponential smoothing)
  // filtered = (1 - alpha) * filtered_prev + alpha * raw
  // k_df is the filter coefficient (0 = no filtering, 1 = no smoothing)
  float32x2_t first_part =
      vfma_n_f32(filtered_derivative_, filtered_derivative_, -k_df_);
  filtered_derivative_ = vfma_n_f32(first_part, derivative_raw, k_df_);

  float d_term_x = k_d_ * filtered_d_x;
  float d_term_y = k_d_ * filtered_d_y;

  // === PROPORTIONAL TERM ===
  float p_term_x = k_p_ * error_x;
  float p_term_y = k_p_ * error_y;

  // === COMBINE PID TERMS ===
  float output_x = p_term_x + i_term_x + d_term_x;
  float output_y = p_term_y + i_term_y + d_term_y;

  // === CONVERT TO RC PWM VALUES (1000-2000 range, 1500 center) ===
  // Clamp output to reasonable range (e.g., ±500 from center)
  const float MAX_PWM_OFFSET = 500.0f;
  output_x = std::clamp(output_x, -MAX_PWM_OFFSET, MAX_PWM_OFFSET);
  output_y = std::clamp(output_y, -MAX_PWM_OFFSET, MAX_PWM_OFFSET);

  uint32_t pwm_x = static_cast<uint32_t>(1500.0f + output_x);
  uint32_t pwm_y = static_cast<uint32_t>(1500.0f + output_y);

  // Additional safety clamping to ensure valid PWM range
  pwm_x = std::clamp(pwm_x, 1000u, 2000u);
  pwm_y = std::clamp(pwm_y, 1000u, 2000u);

  // Store current position for next derivative calculation
  last_value_ = current_position;

  // Return 4-channel RC values (x, y, and two additional channels at neutral)
  // Channels 3 and 4 are set to neutral (1500)
  uint32x4_t result = {pwm_x, pwm_y, 1500, 1500};
  return result;
}
