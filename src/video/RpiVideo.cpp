#include <atomic>
#include <mutex>
#include <iostream>

#include <opencv2/opencv.hpp>
#include <thread>

#include "video/RpiVideo.hpp"
#include "video/RpiCamera.hpp"

namespace video {

RpiVideo::RpiVideo(std::uint32_t height, std::uint32_t width, int framerate) :
    shared_frame_(height, width, CV_8UC3),
    mtx_(),
    cv_(),
    new_data_available_(false),
    frame_timestamp_(),
    camera_(shared_frame_, new_data_available_, mtx_, cv_, frame_timestamp_, height, width, framerate),
    height_(height),
    width_(width) {}

RpiVideo::~RpiVideo() {
    stop_camera();
}

void RpiVideo::start_camera() { camera_.start(); }
void RpiVideo::stop_camera() { camera_.stop(); }

cv::Mat RpiVideo::get_frame() {
    std::unique_lock<std::mutex> lock(mtx_);

    // Wait with timeout for new frame (max 100ms)
    bool frame_ready = cv_.wait_for(
        lock,
        std::chrono::milliseconds(100),
        [this]() { return new_data_available_.load(std::memory_order_acquire); }
    );

    if (!frame_ready) {
        std::cerr << "Warning: get_frame() timeout, returning current frame" << std::endl;
    }

    // Swap frame out
    cv::Mat frame(height_, width_, CV_8UC3);
    std::swap(frame, shared_frame_);

    // Reset flag for next frame
    new_data_available_.store(false, std::memory_order_release);

    return frame;
}

}
