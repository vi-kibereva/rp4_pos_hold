#include <memory>

#include <opencv2/opencv.hpp>
#include <iostream>
#include <csignal>

#include "video/RpiCamera.hpp"


int main() {
    std::cout << "Starting RpiCamera with libcamera...\n";

    std::unique_ptr<video::RpiCamera> camera = std::make_unique<video::RpiCamera>(0);

    std::cout << "Camera initialized: " << camera->getWidth() << "x" << camera->getHeight() << "\n";

    std::string filename = "output_image.png";

    std::cout << "Reading frame...\n";
    cv::Mat frame = camera->readFrame();

    if (!frame.empty()) {
        std::cout << "Successfully captured frame: " << frame.cols << "x" << frame.rows << "\n";

        bool success = cv::imwrite(filename, frame);

        if (success) {
            std::cout << "Image saved successfully as " << filename << std::endl;
        } else {
            std::cerr << "Error: Could not save image." << std::endl;
        }
    } else {
        std::cout << "Failed to capture frame\n";
    }

    return 0;
}

