#include <thread>

#include "video/RpiCamera.hpp"

namespace video {

RpiCamera::RpiCamera(
        std::function<void(cv::Mat)> set_latest, 
        std::uint32_t height,
        std::uint32_t width,
        int framerate
    ) : set_latest_(set_latest), 
        height_(height), 
        width_(width),
        framerate_(framerate), 
        running_(true),
        cam_(),
        producer_buffer_(height, width, CV_8UC3) { // TODO: idk
    cam_.options->video_width = width;
    cam_.options->video_height = height;
    cam_.options->framerate = framerate;
    cam_.options->verbose = true;
}

void RpiCamera::start() {
    running_ = true;
    if (!producer_thread_.has_value())
        producer_thread_ = std::thread(producer_thread, this);
}

void RpiCamera::stop() {
    running_ = false;

    if (producer_thread_.has_value() && producer_thread_->joinable())
        producer_thread_->join(); // TODO: what if has_value and not joinable?

    producer_thread_ = std::nullopt;
}

void RpiCamera::producer_thread(RpiCamera* rpi_cam) {
    while (rpi_cam->running_) {
        if (!rpi_cam->cam_.getVideoFrame(rpi_cam->producer_buffer_, 35)) {
            std::cerr << "Timeout!" << std::endl;
            continue;
        }

        rpi_cam->set_latest_(rpi_cam->producer_buffer_);
    }

}

}
