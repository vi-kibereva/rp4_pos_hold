#include <mutex>
#include <thread>

#include "video/RpiCamera.hpp"

namespace video {

RpiCamera::RpiCamera(
        cv::Mat &shared_buffer,
        std::atomic<bool> &new_data_available,
        std::mutex &mtx,
        std::condition_variable &cv,
        std::chrono::steady_clock::time_point &frame_timestamp,
        std::uint32_t height,
        std::uint32_t width,
        int framerate
    ) : running_(false),
        cam_(),
        producer_buffer_(height, width, CV_8UC3),
        last_valid_frame_(height, width, CV_8UC3),
        shared_buffer_(shared_buffer),
        new_data_available_(new_data_available),
        frame_timestamp_(frame_timestamp),
        mtx_(mtx),
        cv_(cv) {
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
    try {
        rpi_cam->cam_.startVideo();


        while (rpi_cam->running_) {
            bool frame_valid = rpi_cam->cam_.getVideoFrame(rpi_cam->producer_buffer_, 935);

            if (!frame_valid) {
                std::cerr << "Timeout! Reusing last valid frame" << std::endl;
                // Reuse last valid frame
                rpi_cam->last_valid_frame_.copyTo(rpi_cam->producer_buffer_);
            } else {
                // Cache this valid frame for future timeouts
                rpi_cam->producer_buffer_.copyTo(rpi_cam->last_valid_frame_);
            }

            {
                std::lock_guard<std::mutex> lock(rpi_cam->mtx_);
                // Swap buffers first
                std::swap(rpi_cam->shared_buffer_, rpi_cam->producer_buffer_);
                rpi_cam->frame_timestamp_ = std::chrono::steady_clock::now();
                // Set flag AFTER swap completes (fixes race condition)
                rpi_cam->new_data_available_.store(true, std::memory_order_release);
            }
            // Notify consumer AFTER releasing lock (more efficient)
            rpi_cam->cv_.notify_one();
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
