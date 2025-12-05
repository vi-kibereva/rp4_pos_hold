#include <iostream>
#include <chrono>
#include <thread>
#include <fstream>
#include <iomanip>

#include "msp/msp.hpp"
#include "video/RpiVideo.hpp"

using namespace std::chrono_literals;

int main(int argc, char* argv[]) {
    const char* port = argc > 1 ? argv[1] : "/dev/ttyAMA0";

    msp::Msp* msp;
	try {
		msp = new msp::Msp(port, B115200, 10);
	} catch (const std::exception &ex) {
		std::cout << "Error: " << ex.what() << '\n';
		return 1;
	}

    // Frame timing configuration
    constexpr int TARGET_FPS = 15;
    constexpr int TOTAL_FRAMES = 300;
    const auto FRAME_DURATION = std::chrono::microseconds(1'000'000 / TARGET_FPS);

    // Video recording setup
    std::vector<cv::Mat> videoData{};
    videoData.reserve(TOTAL_FRAMES);

    // Initialize RpiVideo directly (no Drone wrapper)
    video::RpiVideo camera(1080, 1920, TARGET_FPS);
    camera.start_camera();

    // CSV file setup with descriptive headers
    std::ofstream rawImuCsv("raw_imu_data.csv");
    std::ofstream attitudeCsv("attitude_data.csv");
    std::ofstream altitudeCsv("altitude_data.csv");

    // Write CSV headers
    rawImuCsv << "timestamp,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z,mag_x,mag_y,mag_z\n";
    attitudeCsv << "timestamp,roll,pitch,yaw\n";
    altitudeCsv << "timestamp,altitude,vario\n";

    // Set floating point precision for timestamps
    rawImuCsv << std::fixed << std::setprecision(6);
    attitudeCsv << std::fixed << std::setprecision(6);
    altitudeCsv << std::fixed << std::setprecision(6);

    auto start_time = std::chrono::steady_clock::now();

    std::cout << "Recording " << TOTAL_FRAMES << " frames at " << TARGET_FPS
              << " FPS" << std::endl;
    
    for (int i = 0; i < TOTAL_FRAMES; ++i) {
        // Precise frame timing
        auto target_time = start_time + (i * FRAME_DURATION);
        std::this_thread::sleep_until(target_time);

        // Calculate timestamp (float seconds from start)
        float timestamp = i / static_cast<float>(TARGET_FPS);

        // Progress reporting (every 30 frames)
        if (i % 30 == 0) {
            auto current_time = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time);
            std::cout << i << "/" << TOTAL_FRAMES << " frames captured" << std::endl;
        }

        // Capture video frame
        cv::Mat frame = camera.get_frame();
        videoData.push_back(frame);

        // Request MSP data with error handling
        try {
            // Raw IMU data
            msp::RawImuData rawImu = msp->rawImu();
            rawImuCsv << timestamp << ","
                      << rawImu.acc_x << "," << rawImu.acc_y << "," << rawImu.acc_z << ","
                      << rawImu.gyro_x << "," << rawImu.gyro_y << "," << rawImu.gyro_z << ","
                      << rawImu.mag_x << "," << rawImu.mag_y << "," << rawImu.mag_z << "\n";

            // Attitude data (convert tenths of degrees to degrees)
            msp::AttitudeData attitude = msp->attitude();
            attitudeCsv << timestamp << ","
                        << (attitude.roll_tenths / 10.0) << ","
                        << (attitude.pitch_tenths / 10.0) << ","
                        << (attitude.yaw_tenths / 10.0) << "\n";

            // Altitude data
            msp::AltitudeData altitude = msp->altitude();
            altitudeCsv << timestamp << ","
                        << altitude.altitude << ","
                        << altitude.vario << "\n";

        } catch (const std::runtime_error& e) {
            std::cerr << "MSP request failed at frame " << i
                      << ": " << e.what() << std::endl;
            // Skip CSV write for this frame to maintain data integrity
        }
    }

    // Stop camera
    camera.stop_camera();

    // Close CSV files
    rawImuCsv.close();
    attitudeCsv.close();
    altitudeCsv.close();

    std::cout << "\n=== Recording Complete ===" << std::endl;

    // Write video from buffered frames
    std::cout << "Writing video file..." << std::endl;

    cv::VideoWriter videoWriter;
    videoWriter.open("flow_output.avi",
                    cv::VideoWriter::fourcc('m', 'p', '4', 'v'),
                    TARGET_FPS,
                    videoData.front().size());

    if (!videoWriter.isOpened()) {
        std::cerr << "Failed to open video writer!" << std::endl;
        delete msp;
        return 1;
    }

    for (const cv::Mat& mat : videoData) {
        videoWriter.write(mat);
    }

    videoWriter.release();
    std::cout << "Video written successfully." << std::endl;

    delete msp;
    return 0;
}
