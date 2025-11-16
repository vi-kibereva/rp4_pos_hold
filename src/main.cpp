#include <memory>

#include <opencv2/opencv.hpp>
#include <iostream>
#include <csignal>

#include "video/RpiCamera.hpp"


int main() {
    std::cout << "321\n";
    std::unique_ptr<video::RpiCamera> camera = std::make_unique<video::RpiCamera>("/dev/video0");

    cv::Mat frame = camera->readFrame();
}

