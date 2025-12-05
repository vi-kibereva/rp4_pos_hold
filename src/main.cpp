#include <iostream>
#include <chrono>
#include <thread>

#include "posHold/Drone.hpp"

#include "posHold/VecMove.hpp"

#include "pid/pid.hpp"

using namespace std::chrono_literals;

int main(int argc, char* argv[]) {
    if (argc < 2) {
		std::cerr << "Usage: " << argv[0] << " /dev/ttyUSB0\n";
		return 2;
	}

	const char *port = argv[1];
    msp::Msp* msp;
	try {
		msp = new msp::Msp(port, B115200, 10);
	} catch (const std::exception &ex) {
		std::cout << "Error: " << ex.what() << '\n';
		return 1;
	}

    Drone drone{};
    VecMove vecMove(drone);

    PidController controller(1.0f, 0.0f, 0.0f, 0.0f);

    auto t1 = std::chrono::high_resolution_clock::now();

    // Frame timing configuration
    constexpr int TARGET_FPS = 10;
    constexpr int TOTAL_FRAMES = 300;
    const auto FRAME_DURATION = std::chrono::microseconds(1'000'000 / TARGET_FPS);
    
    cv::VideoWriter videoWriter;
    std::ofstream textWriter;
    std::vector<cv::Mat> videoData{};
    videoData.reserve(TOTAL_FRAMES);

    auto start_time = std::chrono::steady_clock::now();

    std::cout << "Recording " << TOTAL_FRAMES << " frames at " << TARGET_FPS
              << " FPS (1 frame every " << FRAME_DURATION.count() << " μs)" << std::endl;

    cv::Point2f cvVecMove_base;
    static auto next_trigger = std::chrono::steady_clock::now() + 1s;
    
    for (int i = 0; i < TOTAL_FRAMES; ++i) {
        // Calculate precise target time for this frame
        auto target_time = start_time + (i * FRAME_DURATION);

        // Check if we're already late
        auto now = std::chrono::steady_clock::now();
        if (now < target_time) {
            // We have time - sleep until target
            std::this_thread::sleep_until(target_time);
        } else {
            // We're late - log warning but continue
            auto late_by = std::chrono::duration_cast<std::chrono::microseconds>(
                now - target_time);
            if (late_by.count() > 1000) {}
        }

        if (i % 30 == 0) {
            auto current_time = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                current_time - start_time);
            auto expected_ms = (i * 1000) / TARGET_FPS;

            std::cout << i << "/" << TOTAL_FRAMES << " | Elapsed: " << elapsed.count() << " ms"
                      << " | Expected: " << expected_ms << " ms"
                      << " | Deviation: " << (elapsed.count() - expected_ms) << " ms"
                      << std::endl;
        }

        vecMove.calc();
        auto t2 = std::chrono::high_resolution_clock::now();
        cv::Point2f cvVecMove = vecMove.getVecMove(); // / (std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() / 1e6);
        t1 = t2;

        videoData.push_back(grayFrame);
        
        // Visualization
        
        if (!textWriter.is_open())
        {
            textWriter.open("flow_txt_data.txt");
        }

        cv::cvtColor(videoData.back(), videoData.back(), cv::COLOR_GRAY2BGR);

        // Draw mean flow arrow in corner
        cv::Point corner(100, 100);
        cv::Point cornerTo(
            corner.x + cvRound(cvVecMove.x * 3000),
            corner.y + cvRound(cvVecMove.y * 3000)
        );
        cv::arrowedLine(videoData.back(), corner, cornerTo, cv::Scalar(0, 255, 0), 3, cv::LINE_AA);
        
        textWriter << "x: " << cvVecMove.x << ", " << "y: " << cvVecMove.y << '\n';
    }

    // Final timing report
    auto end_time = std::chrono::steady_clock::now();
    auto total_duration = std::chrono::duration_cast<std::chrono::milliseconds>(
        end_time - start_time);
    auto expected_duration = (TOTAL_FRAMES * 1000) / TARGET_FPS;
    auto deviation = total_duration.count() - expected_duration;

    std::cout << "\n=== Recording Complete ===" << std::endl;
    std::cout << "Total time: " << total_duration.count() << " ms" << std::endl;
    std::cout << "Expected:   " << expected_duration << " ms" << std::endl;
    std::cout << "Deviation:  " << deviation << " ms" << std::endl;

    if (!videoWriter.isOpened())
    {
        videoWriter.open("flow_output.avi",
                    cv::VideoWriter::fourcc('m', 'p', '4', 'v'),
                    10,
                    videoData.back().size());
    }

    for (const cv::Mat& mat : videoData)
    {
        videoWriter.write(mat);
    }
    
    return 0;
}
