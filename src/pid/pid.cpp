#include "pid/pid.hpp"
#include <algorithm>
#include <arm_neon.h>
#include <iostream>

PidController::PidController(float k_p, float k_i, float k_d, float k_df)
		: k_p_(k_p), k_i_(k_i), k_d_(k_d), k_df_(k_df) {
	last_time = std::chrono::duration_cast<std::chrono::microseconds>(
			std::chrono::steady_clock::now().time_since_epoch());
}

uint32x2_t PidController::calculate_raw_rc(float32x2_t current_position,
										   float32x2_t desired_position) {
	auto current_time = std::chrono::duration_cast<std::chrono::microseconds>(
			std::chrono::steady_clock::now().time_since_epoch());
	float32x2_t error = vsub_f32(desired_position, current_position);
	float dt_sec = (current_time - last_time).count() / 1000000.0f;
	last_time = current_time;

	if (dt_sec <= 0.0f) {
		filtered_derivative_ = vdup_n_f32(0.0f);
	} else {
		float32x2_t derivative_raw =
				vmul_n_f32(vsub_f32(last_value_, current_position), 1.0f / dt_sec);

		float32x2_t first_part =
				vfma_n_f32(filtered_derivative_, filtered_derivative_, -k_df_);
		filtered_derivative_ = vfma_n_f32(first_part, derivative_raw, k_df_);
	}

	integral_ = vfma_n_f32(integral_, error, dt_sec);

	integral_ = vmin_f32(vmax_f32(integral_, integral_min_), integral_max_);

	float32x2_t i_term = vmul_n_f32(integral_, k_i_);

	float32x2_t d_term = vmul_n_f32(filtered_derivative_, k_d_);

	float32x2_t p_term = vmul_n_f32(error, k_p_);

	float32x2_t temp = vadd_f32(i_term, p_term);
	float32x2_t output = vadd_f32(temp, d_term);

	const float MAX_PWM_OFFSET = 500.0f;
	float32x2_t offset = vdup_n_f32(MAX_PWM_OFFSET);
	output = vmin_f32(output, offset);
	offset = vneg_f32(offset);
	output = vmax_f32(output, offset);


	const float BASE_SPEED = 1500.0f;
	float32x2_t base_speed = vdup_n_f32(BASE_SPEED);
	output = vadd_f32(output, base_speed);
	uint32x2_t int_output = vcvt_u32_f32(output);

	const uint32_t MAX_VALUE = 2000u;
	const uint32_t MIN_VALUE = 1000u;
	uint32x2_t max_value = vdup_n_u32(MAX_VALUE);
	uint32x2_t min_value = vdup_n_u32(MIN_VALUE);
	int_output = vmin_u32(int_output, max_value);
	int_output = vmax_u32(int_output, min_value);


	uint32_t my_values[2];
	vst1_u32(my_values, int_output);

	printf("PID Output PWM: x=%d, y=%d\n", my_values[0], my_values[1]);
	last_value_ = current_position;

	return int_output;
}
