#include <functional>
#include <opencv2/opencv.hpp>
#include <thread>
#include <optional>

namespace video {

class RpiCamera {
public:
    RpiCamera() = delete;
    RpiCamera(
        std::function<void(cv::Mat)> set_latest, 
        std::uint32_t height_ = 1080,
        std::uint32_t width_ = 1920,
        int framerate_ = 30
    );

    void start();
    void stop();

private:
    std::function<void(cv::Mat)> set_latest_;

    std::uint32_t height_;
    std::uint32_t width_;
    int framerate_;

    std::atomic<bool> running_;

    lccv::PiCamera cam_;
    cv::Mat producer_buffer_;

    std::optional<std::thread> producer_thread_ = std::nullopt;

    static void producer_thread(RpiCamera* rpi_cam);
};

}

#endif // !RPI_CAMERA_HPP
