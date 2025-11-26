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
    mtx_(), new_data_available_(false),
    camera_(shared_frame_, new_data_available_, mtx_, height, width, framerate),
    height_(height), width_(width) {}

RpiVideo::~RpiVideo() {
    stop_camera();
}

void RpiVideo::start_camera() { camera_.start(); }
void RpiVideo::stop_camera() { camera_.stop(); }

cv::Mat RpiVideo::get_frame() {
    while (!new_data_available_.load(std::memory_order_acquire)) {
        std::this_thread::yield();
    }

    std::lock_guard<std::mutex> lock(mtx_);

    cv::Mat frame(height_, width_, CV_8UC3);

    while (!new_data_available_) {}
    std::swap(frame, shared_frame_);

    new_data_available_.store(false, std::memory_order_release);

    return frame;
}

}
