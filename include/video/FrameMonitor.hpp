#ifndef FRAME_MONITOR_HPP
#define FRAME_MONITOR_HPP

#include <mutex>
#include <opencv2/opencv.hpp>

namespace video {

// --- The SPSC Shared Memory Class ---
class FrameMonitor {
private:
    cv::Mat shared_frame_;
    std::mutex mtx_;
    bool new_data_available_ = false;

public:
    // PRODUCER calls this
    void set_latest(cv::Mat& input_frame);

    // CONSUMER calls this
    bool get_latest(cv::Mat& output_frame);
};

} // namespace video

#endif // !FRAME_MONITOR_HPP
