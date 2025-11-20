#ifndef RPI_VIDEO_HPP
#define RPI_VIDEO_HPP

#include <cstdint>
#include <mutex>

#include <opencv2/opencv.hpp>

#include "video/RpiCamera.hpp"

namespace video {

class RpiVideo {
public:
    explicit RpiVideo(std::uint32_t height = 1080, std::uint32_t width = 1920, int framerate = 30);

    void start_camera();
    void stop_camera();

    cv::Mat get_frame();

private:
    cv::Mat shared_frame_;
    std::mutex mtx_;
    bool new_data_avaliable_;

    RpiCamera camera_;
};

} // namespace video

#endif // !RPI_VIDEO_HPP
