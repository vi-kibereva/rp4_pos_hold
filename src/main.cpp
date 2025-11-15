#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    // Put the pipeline on one line
    std::string pipeline =
        "v4l2src device=/dev/video0 ! "
        "video/x-raw, width=1280, height=720, framerate=30/1 ! "
        "videoconvert ! "
        "tee name=t "
        "t. ! queue max-size-buffers=2 leaky=downstream ! videoconvert ! video/x-raw,format=I420 ! "
        "x264enc tune=zerolatency bitrate=1500 speed-preset=superfast ! h264parse ! mp4mux ! filesink location=/home/pi/output.mp4 sync=false "
        "t. ! queue max-size-buffers=1 leaky=downstream ! videoconvert ! video/x-raw,format=BGR ! appsink name=mysink max-buffers=1 drop=true";

    cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
    if (!cap.isOpened()) {
        std::cerr << "Failed to open pipeline\n";
        return -1;
    }

    cv::Mat frame;
    while (true) {
        if (!cap.read(frame)) {
            std::cerr << "Frame read failed\n";
            break;
        }

        // --- your real-time analysis here ---
        // do cheap processing in-place if possible to avoid allocations
        // e.g. cv::Mat gray; cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        // For demonstration, print a timestamp every second
        static int cnt = 0;
        if (++cnt % 30 == 0) std::cout << "frame: " << cnt << std::endl;

        // Break condition example:
        if (cnt > 30*60) break; // run for ~60 seconds (30 fps)
    }

    cap.release();
    return 0;
}

