#include <mutex>

#include <opencv2/opencv.hpp>

#include "video/RpiVideo.hpp"
#include "video/RpiCamera.hpp"

namespace video {

RpiVideo::RpiVideo(std::uint32_t height, std::uint32_t width, int framerate) :
    shared_frame_(height, width, CV_8UC3),
    mtx_(), new_data_avaliable_(false),
    camera_(shared_frame_, new_data_avaliable_, mtx_, height, width, framerate),
    height_(height), width_(width) {}

RpiVideo::~RpiVideo() {
    stop_camera();
}

void RpiVideo::start_camera() { camera_.start(); }
void RpiVideo::stop_camera() { camera_.stop(); }

cv::Mat RpiVideo::get_frame() {
    std::lock_guard<std::mutex> lock(mtx_);

    cv::Mat frame(height_, width_, CV_8UC3);

    while (!new_data_avaliable_) {}
    std::swap(frame, shared_frame_);
    new_data_avaliable_ = false;

    return frame;
}

}
