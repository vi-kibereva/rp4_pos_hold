#include "video/RpiCamera.hpp"

#include <iostream>
#include <stdexcept>
#include <cstdint>
#include <cstring>
#include <chrono>

#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <sys/mman.h>

namespace video {

RpiCamera::RpiCamera(unsigned int camera_index)
    : width_(1920), height_(1080), started_(false) {
    CameraConfig config;
    config.camera_index = camera_index;
    config.width = 1920;
    config.height = 1080;
    config.framerate = 30;
    setupCamera(config);
}

RpiCamera::RpiCamera(const CameraConfig& config)
    : width_(config.width), height_(config.height), started_(false) {
    setupCamera(config);
}

void RpiCamera::setupCamera(const CameraConfig& cfg) {
    std::cout << "Setting up camera " << cfg.camera_index << " at "
              << cfg.width << "x" << cfg.height << "@" << cfg.framerate << "\n";

    camera_manager_ = std::make_unique<libcamera::CameraManager>();

    int ret = camera_manager_->start();
    if (ret) {
        std::cerr << "Failed to start camera manager, error: " << ret << "\n";
        throw std::runtime_error(std::string("Failed to start camera manager"));
    }

    if (camera_manager_->cameras().empty())
        throw std::runtime_error(std::string("No cameras available"));

    std::cout << "Found " << camera_manager_->cameras().size() << " camera(s)\n";

    if (cfg.camera_index >= camera_manager_->cameras().size())
        throw std::runtime_error(std::string("Camera index out of range"));

    camera_ = camera_manager_->cameras()[cfg.camera_index];
    std::cout << "Selected camera: " << camera_->id() << "\n";

    ret = camera_->acquire();
    if (ret) {
        std::cerr << "Failed to acquire camera, error: " << ret << "\n";
        throw std::runtime_error(std::string("Failed to acquire camera"));
    }

    config_ = camera_->generateConfiguration({libcamera::StreamRole::VideoRecording});
    if (!config_)
        throw std::runtime_error(std::string("Failed to generate camera configuration"));

    libcamera::StreamConfiguration& stream_config = config_->at(0);
    stream_config.size.width = cfg.width;
    stream_config.size.height = cfg.height;
    stream_config.pixelFormat = libcamera::formats::BGR888;

    std::cout << "Requested format: " << stream_config.toString() << "\n";

    config_->validate();

    std::cout << "Validated format: " << stream_config.toString() << "\n";

    ret = camera_->configure(config_.get());
    if (ret) {
        std::cerr << "Failed to configure camera, error: " << ret << "\n";
        throw std::runtime_error(std::string("Failed to configure camera"));
    }

    stream_ = stream_config.stream();
    if (!stream_)
        throw std::runtime_error(std::string("Failed to get stream from configuration"));

    allocateBuffers();

    std::cout << "Connecting request completion callback\n";
    camera_->requestCompleted.connect(this, &RpiCamera::onRequestCompleted);

    ret = camera_->start();
    if (ret) {
        std::cerr << "Failed to start camera, error: " << ret << "\n";
        throw std::runtime_error(std::string("Failed to start camera"));
    }

    std::cout << "Queueing " << requests_.size() << " requests\n";
    for (auto& request : requests_)
        camera_->queueRequest(request.get());

    started_ = true;

    bgr_frame_ = cv::Mat(height_, width_, CV_8UC3);
    std::cout << "Camera setup complete\n";
}

void RpiCamera::allocateBuffers() {
    allocator_ = std::make_unique<libcamera::FrameBufferAllocator>(camera_);

    int ret = allocator_->allocate(stream_);
    if (ret < 0) {
        std::cerr << "Failed to allocate buffers, error: " << ret << "\n";
        throw std::runtime_error(std::string("Failed to allocate buffers"));
    }

    const std::vector<std::unique_ptr<libcamera::FrameBuffer>>& buffers =
        allocator_->buffers(stream_);

    std::cout << "Allocated " << buffers.size() << " buffers\n";

    for (unsigned int i = 0; i < buffers.size(); ++i) {
        std::unique_ptr<libcamera::Request> request = camera_->createRequest();
        if (!request)
            throw std::runtime_error(std::string("Failed to create request"));

        libcamera::FrameBuffer* buffer = buffers[i].get();
        ret = request->addBuffer(stream_, buffer);
        if (ret < 0) {
            std::cerr << "Failed to add buffer " << i << " to request, error: " << ret << "\n";
            throw std::runtime_error(std::string("Failed to add buffer to request"));
        }

        requests_.push_back(std::move(request));
    }
    std::cout << "Created " << requests_.size() << " requests\n";
}

void RpiCamera::onRequestCompleted(libcamera::Request* request) {
    if (request->status() == libcamera::Request::RequestCancelled) {
        std::cerr << "Request cancelled\n";
        return;
    }

    if (request->status() != libcamera::Request::RequestComplete) {
        std::cerr << "Request completed with status: " << static_cast<int>(request->status()) << "\n";
    }

    std::lock_guard<std::mutex> lock(queue_mutex_);
    completed_requests_.push(request);
    queue_cv_.notify_one();
}

cv::Mat RpiCamera::readFrame() {
    libcamera::Request* request = nullptr;

    {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        if (!queue_cv_.wait_for(lock, std::chrono::seconds(5),
                                [this] { return !completed_requests_.empty(); })) {
            std::cerr << "Timeout waiting for frame\n";
            return cv::Mat();
        }
        request = completed_requests_.front();
        completed_requests_.pop();
    }

    if (!request) {
        std::cerr << "Null request\n";
        return cv::Mat();
    }

    if (request->status() != libcamera::Request::RequestComplete) {
        std::cerr << "Request not complete, status: " << static_cast<int>(request->status()) << "\n";
        return cv::Mat();
    }

    if (request->buffers().empty()) {
        std::cerr << "No buffers in request\n";
        return cv::Mat();
    }

    libcamera::FrameBuffer* buffer = request->buffers().begin()->second;
    if (!buffer) {
        std::cerr << "Null buffer\n";
        return cv::Mat();
    }

    const libcamera::StreamConfiguration& stream_config = config_->at(0);

    cv::Mat frame = convertToBGR(buffer, stream_config);

    request->reuse(libcamera::Request::ReuseBuffers);
    if (int ret = camera_->queueRequest(request); ret) {
        std::cerr << "queueRequest failed: " << ret << "\n";
        return frame;
    }

    return frame;
}

cv::Mat RpiCamera::convertToBGR(libcamera::FrameBuffer* buffer,
                                 const libcamera::StreamConfiguration& cfg) {
    if (cfg.pixelFormat != libcamera::formats::BGR888)
        throw std::runtime_error("unsupported format");

    const libcamera::FrameBuffer::Plane &plane = buffer->planes()[0];

    std::uint8_t *data = static_cast<std::uint_8*>mmap(
        nullptr,
        plane.length,
        PROT_READ | PROT_WRITE,
        MAP_SHARED,
        plane.fd.get(),
        plane.offset
    );

    if (!data)
        throw std::runtime_error("invalid data");

    cv::Mat frame(cfg.size.height, cfg.size.width, CV_8UC3, data, cfg.stride);

    return frame;
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
