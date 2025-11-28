#ifndef RPI_CAMERA_HPP
#define RPI_CAMERA_HPP

#include <mutex>
#include <thread>
#include <optional>
#include <atomic>
#include <chrono>

#include <lccv.hpp>
#include <opencv2/opencv.hpp>

namespace video {

class RpiCamera {
public:
    RpiCamera() = delete;
    RpiCamera(
        cv::Mat &shared_buffer,
        std::atomic<bool> &new_data_available,
        std::mutex &mtx,
        std::chrono::steady_clock::time_point &frame_timestamp,
        std::uint32_t height = 1080,
        std::uint32_t width = 1920,
        int framerate = 30
    );
    ~RpiCamera();

    void start();
    void stop();

private:
    std::atomic<bool> running_;

    lccv::PiCamera cam_;

    cv::Mat producer_buffer_;
    cv::Mat &shared_buffer_;
    std::atomic<bool> &new_data_available_;
    std::chrono::steady_clock::time_point &frame_timestamp_;

    std::mutex &mtx_;

    std::optional<std::thread> producer_thread_ = std::nullopt;

    static void producer_thread(RpiCamera* rpi_cam);
};

}

#endif // !RPI_CAMERA_HPP
