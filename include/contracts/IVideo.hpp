#ifndef IVIDEO_HPP
#define IVIDEO_HPP

#include <opencv2/opencv.hpp>

namespace contracts {

/**
 * @brief Abstract interface for video capture devices.
 * * Defines the contract for starting/stopping a camera and retrieving
 * frames as OpenCV matrices.
 */
class IVideo {
   public:
    virtual ~IVideo() = default;

    /**
     * @brief Initialize and start the camera hardware/stream.
     */
    virtual void start_camera() = 0;

    /**
     * @brief Stop the camera stream and release hardware resources.
     */
    virtual void stop_camera() = 0;

    /**
     * @brief Retrieve the latest available frame.
     * * Implementations should handle thread safety if the grabber runs
     * asynchronously.
     * * @return cv::Mat The captured frame.
     */
    [[nodiscard]] virtual cv::Mat get_frame() = 0;
};

}  // namespace contracts

#endif  // !IVIDEO_HPP
