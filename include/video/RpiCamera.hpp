#ifndef RPI_CAMERA_HPP
#define RPI_CAMERA_HPP

#include <libcamera/libcamera.h>
#include <opencv2/core.hpp>

#include <memory>
#include <vector>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <map>

namespace video {

struct CameraConfig {
    unsigned int camera_index = 0;
    unsigned int width = 640;
    unsigned int height = 480;
    unsigned int framerate = 30;
};

class RpiCamera {
public:
    RpiCamera() = delete;
    RpiCamera(unsigned int camera_index);
    RpiCamera(const CameraConfig& config);
    ~RpiCamera();

    RpiCamera(const RpiCamera& other) = delete;
    RpiCamera& operator=(const RpiCamera& other) = delete;

    RpiCamera(RpiCamera&& other) = delete;
    RpiCamera& operator=(RpiCamera&& other) = delete;

    cv::Mat readFrame();

    unsigned int getWidth() const { return width_; }
    unsigned int getHeight() const { return height_; }

private:
    std::unique_ptr<libcamera::CameraManager> camera_manager_;
    std::shared_ptr<libcamera::Camera> camera_;
    std::unique_ptr<libcamera::CameraConfiguration> config_;
    std::unique_ptr<libcamera::FrameBufferAllocator> allocator_;
    libcamera::Stream* stream_;

    std::map<libcamera::FrameBuffer*, void*> mapped_buffers_;
    std::vector<std::unique_ptr<libcamera::Request>> requests_;

    std::queue<libcamera::Request*> completed_requests_;
    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;

    unsigned int width_;
    unsigned int height_;
    bool started_;

    void setupCamera(const CameraConfig& cfg);
    void allocateBuffers();
    void mapBuffers();
    void unmapBuffers();
    void onRequestCompleted(libcamera::Request* request);
    cv::Mat convertToBGR(libcamera::FrameBuffer* buffer, libcamera::StreamConfiguration const &cfg);
};

} // namespace video

#endif // RPI_CAMERA_HPP
