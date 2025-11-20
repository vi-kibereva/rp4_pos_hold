#include <mutex>

#include "video/FrameMonitor.hpp"

namespace video {


void FrameMonitor::set_latest(cv::Mat& input_frame) {
    std::lock_guard<std::mutex> lock(mtx_);

    std::swap(shared_frame_, input_frame);
    new_data_available_ = true;
}


bool FrameMonitor::get_latest(cv::Mat& output_frame) {
    std::lock_guard<std::mutex> lock(mtx_);

    if (!new_data_available_)
        return false;

    std::swap(output_frame, shared_frame_);

    new_data_available_ = false;
    return true;
}

}
