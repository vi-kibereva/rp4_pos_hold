#include "video/RpiCamera.hpp"

#include <iostream>
#include <stdexcept>
#include <cstring>

#include <opencv2/imgproc.hpp>
#include <sys/mman.h>

namespace video {

RpiCamera::RpiCamera(unsigned int camera_index)
    : width_(640), height_(480), started_(false) {
    CameraConfig config;
    config.camera_index = camera_index;
    config.width = 640;
    config.height = 480;
    config.framerate = 30;
    setupCamera(config);
}

RpiCamera::RpiCamera(const CameraConfig& config)
    : width_(config.width), height_(config.height), started_(false) {
    setupCamera(config);
}

void RpiCamera::setupCamera(const CameraConfig& cfg) {
    camera_manager_ = std::make_unique<libcamera::CameraManager>();

    int ret = camera_manager_->start();
    if (ret)
        throw std::runtime_error(std::string("Failed to start camera manager"));

    if (camera_manager_->cameras().empty())
        throw std::runtime_error(std::string("No cameras available"));

    if (cfg.camera_index >= camera_manager_->cameras().size())
        throw std::runtime_error(std::string("Camera index out of range"));

    camera_ = camera_manager_->cameras()[cfg.camera_index];

    ret = camera_->acquire();
    if (ret)
        throw std::runtime_error(std::string("Failed to acquire camera"));

    config_ = camera_->generateConfiguration({libcamera::StreamRole::VideoRecording});
    if (!config_)
        throw std::runtime_error(std::string("Failed to generate camera configuration"));

    libcamera::StreamConfiguration& stream_config = config_->at(0);
    stream_config.size.width = cfg.width;
    stream_config.size.height = cfg.height;
    stream_config.pixelFormat = libcamera::formats::YUV420;

    config_->validate();

    ret = camera_->configure(config_.get());
    if (ret)
        throw std::runtime_error(std::string("Failed to configure camera"));

    stream_ = stream_config.stream();

    allocateBuffers();

    camera_->requestCompleted.connect(this, &RpiCamera::onRequestCompleted);

    ret = camera_->start();
    if (ret)
        throw std::runtime_error(std::string("Failed to start camera"));

    for (auto& request : requests_)
        queueRequest(request.get());

    started_ = true;

    bgr_frame_ = cv::Mat(height_, width_, CV_8UC3);
}

void RpiCamera::allocateBuffers() {
    allocator_ = std::make_unique<libcamera::FrameBufferAllocator>(camera_);

    int ret = allocator_->allocate(stream_);
    if (ret < 0)
        throw std::runtime_error(std::string("Failed to allocate buffers"));

    const std::vector<std::unique_ptr<libcamera::FrameBuffer>>& buffers =
        allocator_->buffers(stream_);

    for (unsigned int i = 0; i < buffers.size(); ++i) {
        std::unique_ptr<libcamera::Request> request = camera_->createRequest();
        if (!request)
            throw std::runtime_error(std::string("Failed to create request"));

        const libcamera::FrameBuffer* buffer = buffers[i].get();
        ret = request->addBuffer(stream_, buffer);
        if (ret < 0)
            throw std::runtime_error(std::string("Failed to add buffer to request"));

        requests_.push_back(std::move(request));
    }
}

void RpiCamera::queueRequest(libcamera::Request* request) {
    camera_->queueRequest(request);
}

void RpiCamera::onRequestCompleted(libcamera::Request* request) {
    if (request->status() == libcamera::Request::RequestCancelled)
        return;

    std::lock_guard<std::mutex> lock(queue_mutex_);
    completed_requests_.push(request);
    queue_cv_.notify_one();
}

cv::Mat RpiCamera::readFrame() {
    libcamera::Request* request = nullptr;

    {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        queue_cv_.wait(lock, [this] { return !completed_requests_.empty(); });
        request = completed_requests_.front();
        completed_requests_.pop();
    }

    if (!request)
        return cv::Mat();

    libcamera::FrameBuffer* buffer = request->buffers().begin()->second;
    const libcamera::StreamConfiguration& stream_config = config_->at(0);

    cv::Mat frame = convertToBGR(buffer, stream_config.pixelFormat);

    queueRequest(request);

    return frame;
}

cv::Mat RpiCamera::convertToBGR(libcamera::FrameBuffer* buffer,
                                 const libcamera::PixelFormat& format) {
    const libcamera::FrameBuffer::Plane& plane_y = buffer->planes()[0];
    const libcamera::FrameBuffer::Plane& plane_uv = buffer->planes()[1];

    void* y_data = mmap(nullptr, plane_y.length, PROT_READ, MAP_SHARED,
                        plane_y.fd.get(), 0);
    if (y_data == MAP_FAILED)
        return cv::Mat();

    void* uv_data = mmap(nullptr, plane_uv.length, PROT_READ, MAP_SHARED,
                         plane_uv.fd.get(), 0);
    if (uv_data == MAP_FAILED) {
        munmap(y_data, plane_y.length);
        return cv::Mat();
    }

    cv::Mat yuv_y(height_, width_, CV_8UC1, y_data);
    cv::Mat yuv_uv(height_ / 2, width_ / 2, CV_8UC2, uv_data);

    cv::Mat yuv(height_ + height_ / 2, width_, CV_8UC1);
    yuv_y.copyTo(yuv(cv::Rect(0, 0, width_, height_)));

    cv::Mat uv_resized;
    cv::resize(yuv_uv, uv_resized, cv::Size(width_, height_));

    for (int i = 0; i < height_; ++i) {
        for (int j = 0; j < width_; ++j) {
            yuv.at<uint8_t>(height_ + i, j) = uv_resized.at<cv::Vec2b>(i, j)[i % 2];
        }
    }

    cv::cvtColor(yuv, bgr_frame_, cv::COLOR_YUV2BGR_NV12);

    munmap(y_data, plane_y.length);
    munmap(uv_data, plane_uv.length);

    return bgr_frame_.clone();
}

RpiCamera::~RpiCamera() {
    if (started_ && camera_) {
        camera_->stop();
        started_ = false;
    }

    requests_.clear();
    allocator_.reset();

    if (camera_) {
        camera_->release();
        camera_.reset();
    }

    if (camera_manager_) {
        camera_manager_->stop();
        camera_manager_.reset();
    }
}

RpiCamera::RpiCamera(RpiCamera&& other) noexcept
    : camera_manager_(std::move(other.camera_manager_)),
      camera_(std::move(other.camera_)),
      config_(std::move(other.config_)),
      allocator_(std::move(other.allocator_)),
      stream_(other.stream_),
      bgr_frame_(std::move(other.bgr_frame_)),
      requests_(std::move(other.requests_)),
      width_(other.width_),
      height_(other.height_),
      started_(other.started_) {

    other.stream_ = nullptr;
    other.width_ = 0;
    other.height_ = 0;
    other.started_ = false;
}

RpiCamera& RpiCamera::operator=(RpiCamera&& other) noexcept {
    if (this != &other) {
        if (started_ && camera_) {
            camera_->stop();
        }

        requests_.clear();
        allocator_.reset();

        if (camera_) {
            camera_->release();
            camera_.reset();
        }

        if (camera_manager_) {
            camera_manager_->stop();
            camera_manager_.reset();
        }

        camera_manager_ = std::move(other.camera_manager_);
        camera_ = std::move(other.camera_);
        config_ = std::move(other.config_);
        allocator_ = std::move(other.allocator_);
        stream_ = other.stream_;
        bgr_frame_ = std::move(other.bgr_frame_);
        requests_ = std::move(other.requests_);
        width_ = other.width_;
        height_ = other.height_;
        started_ = other.started_;

        other.stream_ = nullptr;
        other.width_ = 0;
        other.height_ = 0;
        other.started_ = false;
    }
    return *this;
}

} // namespace video
