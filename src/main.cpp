#include "video/RpiVideo.hpp"
#include <iostream>


int main() {
    auto video = video::RpiVideo();
    video.start_camera();
    cv::VideoWriter writer(
        "output.mp4",
        cv::VideoWriter::fourcc('m','p','4','v'),
        30.0,
        cv::Size(1080, 1920)
    );
    if (!writer.isOpened()) {
        std::cerr << "Error: Could not open output file (output.mp4) for writing" << std::endl;
        return -1;
    }

    for (int i = 0; i<300; ++i) {
        cv::Mat frame = video.get_frame();
        writer.write(frame);
    }
    video.stop_camera();
}
