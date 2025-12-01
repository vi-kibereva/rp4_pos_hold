#include <iostream>
#include <chrono>
#include <thread>

#include "posHold/Drone.hpp"

#include "posHold/VecMove.hpp"

#include "pid/pid.hpp"


using namespace std::chrono_literals;   // ← only this one line!

int main(int argc, char* argv[]) {

    if (argc < 2) {
		std::cerr << "Usage: " << argv[0] << " /dev/ttyUSB0\n";
		return 2;
	}

	const char *port = argv[1];
    msp::Msp* msp;
	try {
		// Construct MSP client
		// msp = new msp::Msp(port, B115200, 10);

	} catch (const std::exception &ex) {
		std::cout << "Error: " << ex.what() << '\n';
		return 1;
	}

    Drone drone(*msp);
    VecMove vecMove(drone);

    PidController controller(1.0f, 0.0f, 0.0f, 0.0f);

    auto t1 = std::chrono::high_resolution_clock::now();


    std::vector<cv::Mat> frames{};

    // Frame timing configuration
    constexpr int TARGET_FPS = 30;
    constexpr int TOTAL_FRAMES = 300;
    const auto FRAME_DURATION = std::chrono::microseconds(1'000'000 / TARGET_FPS);  // 33333μs

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

        // writer.write(drone.getGrayscaleImage());
        frames.push_back(drone.getGrayscaleImage());

        if (i % 30 == 0) {
            auto current_time = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                current_time - start_time);
            auto expected_ms = (i * 1000) / TARGET_FPS;

            std::cout << i << "/300 | Elapsed: " << elapsed.count() << " ms"
                      << " | Expected: " << expected_ms << " ms"
                      << " | Deviation: " << (elapsed.count() - expected_ms) << " ms"
                      << std::endl;
        }

        vecMove.calc();
        auto t2 = std::chrono::high_resolution_clock::now();
        cv::Point2f cvVecMove = vecMove.getVecMove() / (std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() / 1e6);
        t1 = t2;
        if (now >= next_trigger) {
            cvVecMove_base = cvVecMove;
            next_trigger = now + 1s;
            std::cout << "------------\n" << cvVecMove_base  << "--------" << std::endl;
        }

        // std::cout << "Difference: \t" << cvVecMove - cvVecMove_base  << std::endl;
        // std::cout << "Difference normal:\t" << cv::norm(cvVecMove - cvVecMove_base)  << std::endl;
        // if (cv::norm(cvVecMove - cvVecMove_base) / cv::norm(cvVecMove_base) > 0.10) {
        //     std::cout << cvVecMove << '\n';
        // }
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

    cv::VideoWriter writer(
        "output.mp4",
        cv::VideoWriter::fourcc('H','2','6','4'),
        30.0,
        cv::Size(1920, 1080)
    );

    if (!writer.isOpened()) {
        std::cerr << "Error: Could not open output file (output.mp4) for writing" << std::endl;
        return -1;
    }

    for (auto& frame : frames)
    {
        writer.write(frame);
    }

    // Success criteria
    if (std::abs(deviation) <= 5) {
        std::cout << "SUCCESS: Deviation within ±5ms tolerance!" << std::endl;
        return 0;
    } else {
        std::cout << "FAILED: Deviation exceeds ±5ms tolerance" << std::endl;
        return 1;
    }
}
