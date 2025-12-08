#include <iostream>
#include <chrono>
#include <thread>
#include <csignal>
#include <atomic>
#include <fstream>
#include <iomanip>
#include <arm_neon.h>

#include "msp/msp.hpp"
#include "posHold/Drone.hpp"
#include "posHold/VecMove.hpp"
#include "pid/pid.hpp"

using namespace std::chrono_literals;

// Global state for signal handler
std::atomic<bool> running(true);

// Signal handler for graceful shutdown
void signal_handler(int signal) {
    if (signal == SIGINT) {
        std::cout << "\n[SHUTDOWN] Ctrl+C detected, stopping position hold..." << std::endl;
        running = false;
    }
}

int main(int argc, char* argv[]) {
    // Configuration constants
    constexpr int CONTROL_RATE_HZ = 10;  // Match camera FPS
    constexpr uint16_t THROTTLE_VALUE = 1500;  // Doesn't matter (not overridden)
    constexpr uint16_t YAW_VALUE = 1500;       // Neutral yaw

    // Default PID gains
    float k_p = -10.0f;
    float k_i = 1.0f;
    float k_d = 1.0f;
    float k_df = 0.0f;

    // Parse command-line arguments
    const char* port = (argc > 1) ? argv[1] : "/dev/serial0";
    if (argc > 2) k_p = std::stof(argv[2]);
    if (argc > 3) k_i = std::stof(argv[3]);
    if (argc > 4) k_d = std::stof(argv[4]);
    if (argc > 5) k_df = std::stof(argv[5]);

    std::cout << "[INIT] Position Hold System Starting..." << std::endl;
    std::cout << "[INIT] MSP Port: " << port << std::endl;
    std::cout << "[INIT] PID Gains: k_p=" << k_p << ", k_i=" << k_i
              << ", k_d=" << k_d << ", k_df=" << k_df << std::endl;

    // Install signal handler
    std::signal(SIGINT, signal_handler);

    // === INITIALIZATION PHASE ===

    // 1. Initialize MSP
    msp::Msp* msp = nullptr;
    try {
        std::cout << "[INIT] Connecting to flight controller..." << std::endl;
        msp = new msp::Msp(port, B115200, 10);

        // Verify connection
        msp::StatusData status = msp->status();
        std::cout << "[INIT] Flight controller connected" << std::endl;
        std::cout << status << std::endl;

    } catch (const std::exception& ex) {
        std::cerr << "[ERROR] MSP initialization failed: " << ex.what() << std::endl;
        return 1;
    }

    // 2. Initialize Drone (starts camera automatically)
    Drone* drone = nullptr;
    try {
        std::cout << "[INIT] Starting camera and sensors..." << std::endl;
        drone = new Drone(*msp);
        std::cout << "[INIT] Camera started at " << drone->cameraInfo.fps << " FPS" << std::endl;
    } catch (const std::exception& ex) {
        std::cerr << "[ERROR] Drone initialization failed: " << ex.what() << std::endl;
        delete msp;
        return 1;
    }

    // 3. Initialize VecMove (optical flow)
    VecMove* vecMove = nullptr;
    try {
        std::cout << "[INIT] Initializing optical flow..." << std::endl;
        vecMove = new VecMove(*drone);
        std::cout << "[INIT] Optical flow ready" << std::endl;
    } catch (const std::exception& ex) {
        std::cerr << "[ERROR] VecMove initialization failed: " << ex.what() << std::endl;
        delete drone;
        delete msp;
        return 1;
    }

    // 4. Initialize PID Controller
    std::cout << "[INIT] Creating PID controller..." << std::endl;
    PidController pid(k_p, k_i, k_d, k_df);

    // 5. Setup telemetry logging
    std::ofstream telemetry_log("position_hold_telemetry.csv");
    telemetry_log << "timestamp,pos_x,pos_y,vel_x,vel_y,roll_pwm,pitch_pwm" << std::endl;
    telemetry_log << std::fixed << std::setprecision(6);

    // 6. Wait for stabilization
    std::cout << "[INIT] Waiting 2 seconds for system stabilization..." << std::endl;
    std::this_thread::sleep_for(2s);

    std::cout << "\n[START] Position hold ACTIVE - Press Ctrl+C to stop\n" << std::endl;

    // === CONTROL LOOP ===

    // State variables
    cv::Point2f current_position(0.0f, 0.0f);   // Integrated position (start at origin)
    cv::Point2f previous_velocity(0.0f, 0.0f);  // For trapezoidal integration

    auto loop_start = std::chrono::steady_clock::now();
    auto frame_duration = std::chrono::milliseconds(1000 / CONTROL_RATE_HZ);
    int frame_count = 0;

    while (running) {
        // Frame timing - sleep until target time
        auto target_time = loop_start + (frame_count * frame_duration);
        std::this_thread::sleep_until(target_time);

        // Calculate actual dt
        auto now = std::chrono::steady_clock::now();
        float dt = 0.1f;  // Default to expected dt (100ms)
        if (frame_count > 0) {
            auto prev_target = loop_start + ((frame_count - 1) * frame_duration);
            dt = std::chrono::duration<float>(target_time - prev_target).count();
        }

        try {
            // Calculate optical flow
            vecMove->calc();
            cv::Point2f velocity = vecMove->getVecMove();  // World-space velocity (m/s)

            // Integrate velocity to position (trapezoidal rule)
            if (frame_count > 0) {
                current_position.x += (previous_velocity.x + velocity.x) * dt / 2.0f;
                current_position.y += (previous_velocity.y + velocity.y) * dt / 2.0f;
            }
            previous_velocity = velocity;

            // Prepare position for PID (convert to NEON format)
            float32x2_t position_neon = {current_position.x, current_position.y};
            float32x2_t target_neon = vdup_n_f32(0.0f);  // Target = (0,0) - hold origin

            // Calculate PID output
            uint32x2_t pwm_output = pid.calculate_raw_rc(position_neon, target_neon);

            // Extract PWM values from NEON vector
            uint32_t pwm_values[2];
            vst1_u32(pwm_values, pwm_output);
            uint16_t roll_pwm = static_cast<uint16_t>(pwm_values[0]);
            uint16_t pitch_pwm = static_cast<uint16_t>(pwm_values[1]);

            // Send RC commands
            msp::SetRawRcData rc_command(roll_pwm, pitch_pwm, THROTTLE_VALUE, YAW_VALUE);
            msp->setRawRc(rc_command);

            // Log telemetry
            float timestamp = frame_count / static_cast<float>(CONTROL_RATE_HZ);
            telemetry_log << timestamp << ","
                         << current_position.x << "," << current_position.y << ","
                         << velocity.x << "," << velocity.y << ","
                         << roll_pwm << "," << pitch_pwm << std::endl;

            // Console status display (every second)
            if (frame_count % CONTROL_RATE_HZ == 0) {
                std::cout << "[" << (frame_count / CONTROL_RATE_HZ) << "s] "
                         << "Pos: (" << std::setw(6) << current_position.x << ", "
                         << std::setw(6) << current_position.y << ") "
                         << "Vel: (" << std::setw(5) << velocity.x << ", "
                         << std::setw(5) << velocity.y << ") "
                         << "PWM: R=" << roll_pwm << " P=" << pitch_pwm << std::endl;
            }

        } catch (const std::exception& ex) {
            std::cerr << "[ERROR] Control loop exception: " << ex.what() << std::endl;
            // Continue loop - might recover
        }

        frame_count++;
    }

    // === SHUTDOWN PHASE ===

    std::cout << "\n[SHUTDOWN] Sending neutral commands..." << std::endl;
    try {
        msp::SetRawRcData neutral(1500, 1500, 1500, 1500);
        msp->setRawRc(neutral);
        std::this_thread::sleep_for(100ms);  // Ensure command is sent
    } catch (const std::exception& ex) {
        std::cerr << "[WARNING] Could not send shutdown commands: " << ex.what() << std::endl;
    }

    // Close telemetry log
    telemetry_log.close();

    // Cleanup (destructors handle camera/MSP cleanup)
    delete vecMove;
    delete drone;  // Stops camera
    delete msp;

    std::cout << "[SHUTDOWN] Position hold terminated. Total frames: " << frame_count << std::endl;
    std::cout << "[SHUTDOWN] Telemetry saved to: position_hold_telemetry.csv" << std::endl;

    return 0;
}
