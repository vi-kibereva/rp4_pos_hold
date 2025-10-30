#ifndef PID_HPP
#define PID_HPP

#include <arm_neon.h>
#include <chrono>

class PidController {
public:
  uint32x4_t caldulate_raw_rc(float32x2_t current_position,
                              float32x2_t desired_position = {0, 0});

private:
  float k_p = 0.0f;
  float k_i = 0.0f;
  float k_d = 0.0f;
  float k_df = 0.0f;

  std::chrono::milliseconds last_time;

  float32x2_t last_value = {0, 0};

  float integral = 0.0f;
};

#endif
