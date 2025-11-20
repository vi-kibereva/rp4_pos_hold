#include <mutex>
#include <thread>

#include "video/RpiCamera.hpp"

namespace video {

RpiCamera::RpiCamera(
        cv::Mat &shared_buffer,
        bool &new_data_available,
        std::mutex &mtx,
        std::uint32_t height,
        std::uint32_t width,
        int framerate
    ) : shared_buffer_(shared_buffer), 
        mtx_(mtx),
        new_data_available_(new_data_available),
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
    if (!producer_thread_.has_value()) {
        cam_.startVideo();
        producer_thread_ = std::thread(producer_thread, this);
    }
}

void RpiCamera::stop() {
    running_ = false;
    cam_.stopVideo();

    if (producer_thread_.has_value() && producer_thread_->joinable())
        producer_thread_->join();

    producer_thread_ = std::nullopt;
}

void RpiCamera::producer_thread(RpiCamera* rpi_cam) {
    while (rpi_cam->running_) {
        if (!rpi_cam->cam_.getVideoFrame(rpi_cam->producer_buffer_, 35)) {
            std::cerr << "Timeout!" << std::endl;
            continue;
        }

        rpi_cam->mtx_.lock();

        std::swap(rpi_cam->shared_buffer_, rpi_cam->producer_buffer_);
        rpi_cam->new_data_available_ = true;

        rpi_cam->mtx_.unlock();
    }
}

}
