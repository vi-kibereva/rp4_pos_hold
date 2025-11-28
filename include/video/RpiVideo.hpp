#ifndef RPI_VIDEO_HPP
#define RPI_VIDEO_HPP

#include <cstdint>
#include <mutex>
#include <atomic>
#include <chrono>

#include <opencv2/opencv.hpp>

#include "video/RpiCamera.hpp"

namespace video {

class RpiVideo {
public:
    explicit RpiVideo(std::uint32_t height = 1080, std::uint32_t width = 1920, int framerate = 30);
    ~RpiVideo();

    void start_camera();
    void stop_camera();

    cv::Mat get_frame();

private:
    cv::Mat shared_frame_;
    std::mutex mtx_;
    std::atomic<bool> new_data_available_;
    std::chrono::steady_clock::time_point frame_timestamp_;

    RpiCamera camera_;

    std::uint32_t height_;
    std::uint32_t width_;
};

} // namespace video

#endif // !RPI_VIDEO_HPP
