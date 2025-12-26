#include <arm_neon.h>

#include <atomic>
#include <chrono>
#include <csignal>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <thread>

#include "msp/msp.hpp"
#include "pid/pid.hpp"
#include "posHold/Drone.hpp"
#include "posHold/VecMove.hpp"

using namespace std::chrono_literals;

std::atomic<bool> running(true);

void signal_handler(int signal) {
    if (signal == SIGINT) {
        std::cout << "\n[SHUTDOWN] Ctrl+C detected, stopping position hold..." << std::endl;
        running = false;
    }
}

int main(int argc, char* argv[]) {
    constexpr int CONTROL_RATE_HZ = 10;        // Match camera FPS
    constexpr uint16_t THROTTLE_VALUE = 1500;  // Doesn't matter (not overridden)
    constexpr uint16_t YAW_VALUE = 1500;       // Neutral yaw

    float k_p = -900.0f;
    float k_i = 0.0f;
    float k_d = 0.0f;
    float k_df = 0.0f;

    const char* port = (argc > 1) ? argv[1] : "/dev/serial0";
    if (argc > 2) k_p = std::stof(argv[2]);
    if (argc > 3) k_i = std::stof(argv[3]);
    if (argc > 4) k_d = std::stof(argv[4]);
    if (argc > 5) k_df = std::stof(argv[5]);

    std::cout << "[INIT] Position Hold System Starting..." << std::endl;
    std::cout << "[INIT] MSP Port: " << port << std::endl;
    std::cout << "[INIT] PID Gains: k_p=" << k_p << ", k_i=" << k_i << ", k_d=" << k_d
              << ", k_df=" << k_df << std::endl;

    std::signal(SIGINT, signal_handler);

    msp::Msp* msp = nullptr;
    try {
        std::cout << "[INIT] Connecting to flight controller..." << std::endl;
        msp = new msp::Msp(port, B115200, 10);

        msp::StatusData status = msp->status();
        std::cout << "[INIT] Flight controller connected" << std::endl;
        std::cout << status << std::endl;

    } catch (const std::exception& ex) {
        std::cerr << "[ERROR] MSP initialization failed: " << ex.what() << std::endl;
        return 1;
    }

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

    std::cout << "[INIT] Creating PID controller..." << std::endl;
    PidController pid(k_p, k_i, k_d, k_df);

    std::ofstream telemetry_log("position_hold_telemetry.csv");
    telemetry_log << "timestamp,pos_x,pos_y,target_x,target_y,vel_x,vel_y,roll_"
                     "pwm,pitch_pwm,aux3,aux3_active"
                  << std::endl;
    telemetry_log << std::fixed << std::setprecision(6);

    std::cout << "[INIT] Waiting 2 seconds for system stabilization..." << std::endl;
    std::this_thread::sleep_for(2s);

    std::cout << "\n[START] Position hold ACTIVE - Press Ctrl+C to stop\n" << std::endl;

    cv::Point2f current_position(0.0f, 0.0f);
    cv::Point2f previous_velocity(0.0f, 0.0f);
    cv::Point2f target_position(0.0f, 0.0f);
    uint16_t previous_aux3 = 1000;
    bool aux3_active = false;

    auto loop_start = std::chrono::steady_clock::now();
    auto frame_duration = std::chrono::milliseconds(1000 / CONTROL_RATE_HZ);
    int frame_count = 0;
    double start_yaw = drone->getGyroData().yaw;

    while (running) {
        auto target_time = loop_start + (frame_count * frame_duration);
        std::this_thread::sleep_until(target_time);

        auto now = std::chrono::steady_clock::now();
        float dt = 0.1f;  // Default to expected dt (100ms)
        if (frame_count > 0) {
            auto prev_target = loop_start + ((frame_count - 1) * frame_duration);
            dt = std::chrono::duration<float>(target_time - prev_target).count();
        }

        try {
            vecMove->calc();
            cv::Point2f velocity = vecMove->getVecMove();

            if (frame_count > 0) {
                current_position.x += (previous_velocity.x + velocity.x) * dt / 2.0f;
                current_position.y += (previous_velocity.y + velocity.y) * dt / 2.0f;
            }
            previous_velocity = velocity;

            uint16_t current_aux3 = 1000;  // Default to LOW in case of read failure
            bool msp_read_success = false;

            try {
                msp::RcData rc_data = msp->rc();
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
            constexpr uint16_t AUX3_THRESHOLD = 1700;
            bool current_aux3_state = current_aux3 >= AUX3_THRESHOLD;
            bool rising_edge = !aux3_active && current_aux3_state;

            if (rising_edge) {
                std::cout << "[MODE] AUX3 rising edge detected! Zeroing position hold "
                             "target..."
                          << std::endl;
                std::cout << "[MODE] Current position: (" << current_position.x << ", "
                          << current_position.y << ") -> new target" << std::endl;

                target_position = current_position;

                start_yaw = drone->getGyroData().yaw;

                float32x2_t current_neon = {current_position.x, current_position.y};
                pid.reset(current_neon);

                std::cout << "[MODE] PID state reset complete" << std::endl;
            }

            aux3_active = current_aux3_state;

            double yaw = drone->getGyroData().yaw - start_yaw + CV_PI;
            double cy = std::cos(yaw);
            double sy = std::sin(yaw);

            float32x2_t position_neon = {cy * current_position.x + sy * current_position.y,
                                         -sy * current_position.x + cy * current_position.y};

            float32x2_t target_neon = {cy * target_position.x + sy * target_position.y,
                                       -sy * target_position.x + cy * target_position.y};

            uint32x2_t pid_output = pid.calculate_raw_rc(position_neon, target_neon);

            uint32_t pid_values[2];
            vst1_u32(pid_values, pid_output);
            uint16_t roll_pid = static_cast<uint16_t>(pid_values[0]);
            uint16_t pitch_pid = static_cast<uint16_t>(pid_values[1]);

            msp::SetRawRcData rc_command(roll_pid, pitch_pid, THROTTLE_VALUE, YAW_VALUE);
            msp->setRawRc(rc_command);

            float timestamp = frame_count / static_cast<float>(CONTROL_RATE_HZ);
            telemetry_log << timestamp << "," << current_position.x << "," << current_position.y
                          << "," << target_position.x << "," << target_position.y << ","
                          << velocity.x << "," << velocity.y << "," << roll_pid << "," << pitch_pid
                          << "," << current_aux3 << "," << (aux3_active ? 1 : 0) << std::endl;

            if (frame_count % CONTROL_RATE_HZ == 0) {
                std::cout << "[" << (frame_count / CONTROL_RATE_HZ) << "s] "
                          << (aux3_active ? "[MSP_OVERRIDE] " : "[NORMAL] ") << "Pos: ("
                          << std::setw(6) << current_position.x << ", " << std::setw(6)
                          << current_position.y << ") "
                          << "Target: (" << std::setw(6) << target_position.x << ", "
                          << std::setw(6) << target_position.y << ") "
                          << "Vel: (" << std::setw(5) << velocity.x << ", " << std::setw(5)
                          << velocity.y << ") "
                          << "PID: R=" << roll_pid << " P=" << pitch_pid << " AUX3=" << current_aux3
                          << std::endl;
            }

        } catch (const std::exception& ex) {
            std::cerr << "[ERROR] Control loop exception: " << ex.what() << std::endl;
            // Continue loop - might recover
        }

        frame_count++;
    }

    std::cout << "\n[SHUTDOWN] Sending neutral commands..." << std::endl;
    try {
        msp::SetRawRcData neutral(1500, 1500, 1500, 1500);
        msp->setRawRc(neutral);
        std::this_thread::sleep_for(100ms);  // Ensure command is sent
    } catch (const std::exception& ex) {
        std::cerr << "[WARNING] Could not send shutdown commands: " << ex.what() << std::endl;
    }

    telemetry_log.close();

    delete vecMove;
    delete drone;
    delete msp;

    std::cout << "[SHUTDOWN] Position hold terminated. Total frames: " << frame_count << std::endl;
    std::cout << "[SHUTDOWN] Telemetry saved to: position_hold_telemetry.csv" << std::endl;

    return 0;
}
