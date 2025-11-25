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
    ) : running_(false),
        cam_(),
        producer_buffer_(height, width, CV_8UC3),
        shared_buffer_(shared_buffer),
        new_data_available_(new_data_available),
        mtx_(mtx) {
    cam_.options->video_width = width;
    cam_.options->video_height = height;
    cam_.options->framerate = framerate;
    cam_.options->verbose = true;
}

RpiCamera::~RpiCamera() {
    stop();
}

void RpiCamera::start() {
    running_ = true;
    if (!producer_thread_.has_value())
        producer_thread_ = std::thread(producer_thread, this);
}

void RpiCamera::stop() {
    running_ = false;

    if (producer_thread_.has_value() && producer_thread_->joinable())
        producer_thread_->join();

    producer_thread_ = std::nullopt;
}

void RpiCamera::producer_thread(RpiCamera* rpi_cam) {
    std::cout << "Started producer thread" << std::endl;
    try {
        rpi_cam->cam_.startVideo();

        while (rpi_cam->running_) {
            if (!rpi_cam->cam_.getVideoFrame(rpi_cam->producer_buffer_, 935)) {
                std::cerr << "Timeout!" << std::endl;
                continue;
            }

            {
                std::lock_guard<std::mutex> lock(rpi_cam->mtx_);
                std::swap(rpi_cam->shared_buffer_, rpi_cam->producer_buffer_);
                rpi_cam->new_data_available_ = true;
                std::cout << "new_data_available_ set true" << std::endl;
            }
        }

        rpi_cam->cam_.stopVideo();
    } catch (const std::exception& e) {
        std::cerr << "Exception in producer thread: " << e.what() << std::endl;
        rpi_cam->running_ = false;
    } catch (...) {
        std::cerr << "Unknown exception in producer thread" << std::endl;
        rpi_cam->running_ = false;
    }
}

}
