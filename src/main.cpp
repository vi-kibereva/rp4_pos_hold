#include "video/RpiVideo.hpp"
#include <iostream>
#include <chrono>
#include <thread>


int main() {
    auto video = video::RpiVideo();
    video.start_camera();

    cv::VideoWriter writer(
        "output.mp4",
        cv::VideoWriter::fourcc('m','p','4','v'),
        30.0,
        cv::Size(1920, 1080)
    );

    if (!writer.isOpened()) {
        std::cerr << "Error: Could not open output file (output.mp4) for writing" << std::endl;
        return -1;
    }

    // Frame timing configuration
    constexpr int TARGET_FPS = 30;
    constexpr int TOTAL_FRAMES = 300;
    const auto FRAME_DURATION = std::chrono::microseconds(1'000'000 / TARGET_FPS);  // 33333μs

    auto start_time = std::chrono::steady_clock::now();

    std::cout << "Recording " << TOTAL_FRAMES << " frames at " << TARGET_FPS
              << " FPS (1 frame every " << FRAME_DURATION.count() << " μs)" << std::endl;

    for (int i = 0; i < TOTAL_FRAMES; ++i) {
        // Calculate precise target time for this frame
        auto target_time = start_time + (i * FRAME_DURATION);

        // Get frame from camera (may block if not ready)
        cv::Mat frame = video.get_frame();

        // Sleep until target time to maintain exact 30 FPS pacing
        std::this_thread::sleep_until(target_time);

        // Write frame at precisely the target time
        writer.write(frame);

        // Log progress every 30 frames (1 second intervals)
        if (i % 30 == 0) {
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                now - start_time);
            auto expected_ms = (i * 1000) / TARGET_FPS;

            std::cout << i << "/300 | Elapsed: " << elapsed.count() << " ms"
                      << " | Expected: " << expected_ms << " ms"
                      << " | Deviation: " << (elapsed.count() - expected_ms) << " ms"
                      << std::endl;
        }
    }

    video.stop_camera();

    // Final timing report
    auto end_time = std::chrono::steady_clock::now();
    auto total_duration = std::chrono::duration_cast<std::chrono::milliseconds>(
        end_time - start_time);
    auto expected_duration = (TOTAL_FRAMES * 1000) / TARGET_FPS;

    std::cout << "\n=== Recording Complete ===" << std::endl;
    std::cout << "Total time: " << total_duration.count() << " ms" << std::endl;
    std::cout << "Expected:   " << expected_duration << " ms" << std::endl;
    std::cout << "Deviation:  " << (total_duration.count() - expected_duration) << " ms" << std::endl;

    return 0;
}
