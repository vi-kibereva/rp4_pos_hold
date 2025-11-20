#include <memory>

#include <opencv2/opencv.hpp>
#include <iostream>
#include <thread>
#include <atomic>
#include <chrono>

#include "posHold/Drone.h"
#include "video/FrameMonitor.hpp"


// Globals
video::FrameMonitor monitor;
std::atomic<bool> running(true);

// --- Consumer Thread (The Slow Processor) ---
void consumer_thread() {
    // We pre-allocate this, but its data pointer will change constantly via swapping
    cv::Mat processing_frame;
    int count = 0;

    std::cout << "[Consumer] Started." << std::endl;

    while (running) {
        // 1. Try to grab the latest frame (Moves memory, doesn't copy)
        if (monitor.get_latest(processing_frame)) {

            // Example processing
            cv::Scalar avg = cv::mean(processing_frame);

            count++;
            std::cout << "[Consumer] Processed Frame " << count
                      << " | Avg Brightness: " << (int)avg[0] << std::endl;

        } else {
            // No new frame? Sleep tiny amount to prevent CPU burning
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
}

int main() {
    std::cout << "Starting RpiCamera with libcamera...\n";

    std::unique_ptr<video::RpiCamera> camera = std::make_unique<video::RpiCamera>(0);

    std::cout << "Camera initialized: " << camera->getWidth() << "x" << camera->getHeight() << "\n";

    std::string filename = "output_image.png";

    std::cout << "Reading frame...\n";
    cv::Mat frame = camera->readFrame();

    // Run for 300 frames (~10 seconds)
    for(int i = 0; i < 300; i++) {
        if (!cam.getVideoFrame(producer_buffer, 35)) {
            std::cerr << "Timeout!" << std::endl;
            continue;
        }

        monitor.set_latest(producer_buffer);

        if (i % 30 == 0) {
            std::cout << "[Producer] Captured frame " << i << std::endl;
        }
    } else {
        std::cout << "Failed to capture frame\n";
    }

    return 0;
}
