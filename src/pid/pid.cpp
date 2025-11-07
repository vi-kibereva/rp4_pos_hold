#include "pid/pid.hpp"
#include <algorithm>
#include <arm_neon.h>

PidController::PidController(float k_p, float k_i, float k_d, float k_df)
    : k_p_(k_p), k_i_(k_i), k_d_(k_d), k_df_(k_df) {
  last_time = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now().time_since_epoch());
}

uint32x2_t PidController::calculate_raw_rc(float32x2_t current_position,
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

  float32x2_t d_term = vmul_n_f32(filtered_derivative_, k_d_);


  // === PROPORTIONAL TERM ===
  float32x2_t p_term = vmul_n_f32(error, k_d_);

  // === COMBINE PID TERMS ===
  float32x2_t temp = vadd_f32(i_term, p_term);
  float32x2_t output = vadd_f32(temp, d_term);

  // === CONVERT TO RC PWM VALUES (1000-2000 range, 1500 center) ===
  // Clamp output to reasonable range (e.g., ±500 from center)
  const float MAX_PWM_OFFSET = 500.0f;
  float32x2_t offset = vdup_n_f32(MAX_PWM_OFFSET);
    output = vminq_f32(output, offset);
    offset = vneg_f32(offset);
    output = vmaxq_f32(output, offset);


    const float BASE_SPEED = 1500.0f;
    float32x2_t base_speed = vdup_n_f32(BASE_SPEED);
    output = vadd_f32(output, base_speed);
    uint32x2_t int_output = vcvt_u32_f32(output);

    const float MAX_VALUE = 2000.0f;
    const float MIN_VALUE = 1000.0f;
    float32x2_t max_value = vdup_n_f32(MAX_VALUE);
    float32x2_t min_value = vdup_n_f32(MIN_VALUE);
    output = vminq_f32(output, max_value);
    output = vmaxq_f32(output, min_value);

  // Store current position for next derivative calculation
  last_value_ = current_position;

  // Return 4-channel RC values (x, y, and two additional channels at neutral)
  // Channels 3 and 4 are set to neutral (1500)
  uint32x2_t result = {pwm_x, pwm_y};
  return result;
}
