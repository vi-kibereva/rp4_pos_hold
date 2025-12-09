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
    float k_p = -100.0f;
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
    telemetry_log << "timestamp,pos_x,pos_y,target_x,target_y,vel_x,vel_y,roll_pwm,pitch_pwm,aux3,aux3_active" << std::endl;
    telemetry_log << std::fixed << std::setprecision(6);

    // 6. Wait for stabilization
    std::cout << "[INIT] Waiting 2 seconds for system stabilization..." << std::endl;
    std::this_thread::sleep_for(2s);

    std::cout << "\n[START] Position hold ACTIVE - Press Ctrl+C to stop\n" << std::endl;

    // === CONTROL LOOP ===

    // State variables
    cv::Point2f current_position(0.0f, 0.0f);   // Integrated position (start at origin)
    cv::Point2f previous_velocity(0.0f, 0.0f);  // For trapezoidal integration
    cv::Point2f target_position(0.0f, 0.0f);    // Dynamic target position
    uint16_t previous_aux3 = 1000;               // For rising edge detection (start LOW)
    bool aux3_active = false;                    // Current AUX3 state

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

            // Read AUX3 channel for MSP override mode detection
            uint16_t current_aux3 = 1000;  // Default to LOW in case of read failure
            bool msp_read_success = false;

            try {
                msp::RcData rc_data = msp->rc();
                // AUX3 is channel index 6 (0-indexed: roll, pitch, throttle, yaw, aux1, aux2, aux3)
                if (rc_data.channel_count > 6) {
                    current_aux3 = rc_data.channels[6];
                    msp_read_success = true;
                } else {
                    std::cerr << "[WARNING] Not enough RC channels ("
                              << static_cast<int>(rc_data.channel_count)
                              << "), expected >= 7. Using default AUX3=1000" << std::endl;
                }
            } catch (const std::exception& ex) {
                std::cerr << "[WARNING] Failed to read RC channels: " << ex.what()
                          << ". Using default AUX3=1000" << std::endl;
            }

            // Detect rising edge (LOW -> HIGH transition)
            constexpr uint16_t AUX3_THRESHOLD = 1500;
            bool current_aux3_state = current_aux3 >= AUX3_THRESHOLD;
            bool rising_edge = !aux3_active && current_aux3_state;

            if (rising_edge) {
                std::cout << "[MODE] AUX3 rising edge detected! Zeroing position hold target..." << std::endl;
                std::cout << "[MODE] Current position: (" << current_position.x << ", "
                          << current_position.y << ") -> new target" << std::endl;

                // Set target to current position (zero the error)
                target_position = current_position;

                // Reset PID state to prevent windup from old target
                float32x2_t current_neon = {current_position.x, current_position.y};
                pid.reset(current_neon);

                std::cout << "[MODE] PID state reset complete" << std::endl;
            }

            // Update state for next iteration
            aux3_active = current_aux3_state;

            // Prepare position for PID (convert to NEON format)
            float32x2_t position_neon = {current_position.x, current_position.y};
            float32x2_t target_neon = {target_position.x, target_position.y};

            // Calculate PID output
            uint32x2_t pid_output = pid.calculate_raw_rc(position_neon, target_neon);

            // Extract PWM values from NEON vector
            uint32_t pid_values[2];
            vst1_u32(pid_values, pid_output);
            uint16_t roll_pid = static_cast<uint16_t>(pid_values[0]);
            uint16_t pitch_pid = static_cast<uint16_t>(pid_values[1]);

            // Send RC commands
            msp::SetRawRcData rc_command(roll_pid, pitch_pid, THROTTLE_VALUE, YAW_VALUE);
            msp->setRawRc(rc_command);

            // Log telemetry
            float timestamp = frame_count / static_cast<float>(CONTROL_RATE_HZ);
            telemetry_log << timestamp << ","
                         << current_position.x << "," << current_position.y << ","
                         << target_position.x << "," << target_position.y << ","
                         << velocity.x << "," << velocity.y << ","
                         << roll_pid << "," << pitch_pid << ","
                         << current_aux3 << "," << (aux3_active ? 1 : 0) << std::endl;

            // Console status display (every second)
            if (frame_count % CONTROL_RATE_HZ == 0) {
                std::cout << "[" << (frame_count / CONTROL_RATE_HZ) << "s] "
                         << (aux3_active ? "[MSP_OVERRIDE] " : "[NORMAL] ")
                         << "Pos: (" << std::setw(6) << current_position.x << ", "
                         << std::setw(6) << current_position.y << ") "
                         << "Target: (" << std::setw(6) << target_position.x << ", "
                         << std::setw(6) << target_position.y << ") "
                         << "Vel: (" << std::setw(5) << velocity.x << ", "
                         << std::setw(5) << velocity.y << ") "
                         << "PID: R=" << roll_pid << " P=" << pitch_pid
                         << " AUX3=" << current_aux3 << std::endl;
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
