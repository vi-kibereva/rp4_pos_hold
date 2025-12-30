#ifndef RPI_VIDEO_HPP
#define RPI_VIDEO_HPP

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <opencv2/opencv.hpp>

#include "contracts/IVideo.hpp"
#include "video/RpiCamera.hpp"

namespace video {

class RpiVideo : public contracts::IVideo {
   public:
    explicit RpiVideo(std::uint32_t height = 1080, std::uint32_t width = 1920, int framerate = 30);
    ~RpiVideo() override;

    void start_camera() override;
    void stop_camera() override;

    [[nodiscard]] cv::Mat get_frame() override;

   private:
    cv::Mat shared_frame_;
    std::mutex mtx_;
    std::condition_variable cv_;
    std::atomic<bool> new_data_available_;
    std::chrono::steady_clock::time_point frame_timestamp_;

    RpiCamera camera_;

    std::uint32_t height_;
    std::uint32_t width_;
};

}  // namespace video

#endif  // !RPI_VIDEO_HPP
