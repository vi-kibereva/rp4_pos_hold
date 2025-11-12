#include "msp/msp.hpp"

#include <exception>
#include <iostream>
#include <unistd.h>
#include <chrono>
#include <thread>

#include "posHold/VecMove.h"
#include "pid/pid.hpp"

int main(int argc, char* argv[])
{
	/*if (argc < 2) {
		std::cerr << "Usage: " << argv[0] << " /dev/ttyUSB0\n";
		return 2;
	}

	const char *port = argv[1];

	try {
		// Construct MSP client
		msp::Msp msp(port, B115200, 10);

		// --- Example: MSP_STATUS ---
		std::cout << msp.status() << '\n';

		// --- Example: MSP_RC ---
		std::cout << msp.rc() << '\n';

		// --- Example: MSP_ATTITUDE ---
		std::cout << msp.attitude() << '\n';

		// --- Example: MSP_ALTITUDE ---
		std::cout << msp.altitude() << '\n';

		auto start = std::chrono::steady_clock::now();

		// --- Example: MSP_SET_RAW_RC (commented out for safety) ---
		msp::SetRawRcData rc_data(1500, 1500, 1000, 1500, 1900, 1000, 1700, 1000);
		std::cout << "Sending: " << rc_data << '\n';
		for (int i = 0; i<200; ++i){
			msp.setRawRc(rc_data);
			std::cout << msp.rc() << '\n';
		}
		std::cout << "Armed"<< '\n';
		msp::SetRawRcData rc_data_throttle(1500, 1500, 1300, 1500, 1900, 1000, 1700, 1000);
		for (int i = 0; i<200; ++i){
			msp.setRawRc(rc_data_throttle);
			std::cout << msp.rc() << '\n';
		}
		std::cout << "RC values sent successfully\n";

		sleep(1);

		std::cout << msp.rc() << '\n';

		return 0;

	} catch (const std::exception &ex) {
		std::cerr << "Error: " << ex.what() << '\n';
		return 1;
	}*/

	/*if (argc < 2)
	{
		std::cerr << "Usage: " << argv[0] << " /dev/ttyUSB0\n";
		return 2;
	}

	const char *port = argv[1];

	try
	{
		msp::Msp msp(port, B115200, 10);

		Drone drone(msp);

		VecMove vecMove(drone);

		PidController controller{};

		auto t1 = std::chrono::high_resolution_clock::now();

		while (true)
		{
			vecMove.calc();
			auto t2 = std::chrono::high_resolution_clock::now();
			cv::Point2f cvVecMove = vecMove.getVecMove() / (std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() / 1e6);
			t1 = t2;
			uint32x2_t result = controller.calculate_raw_rc(
				vdup_n_f32(0.0f),
				float32x2_t{ cvVecMove.x, cvVecMove.y }
			);
			msp.setRawRc(msp::SetRawRcData(result[0], result[1], 0, 0));
		}
	}
	catch (const std::exception &ex)
	{
		std::cerr << "Error: " << ex.what() << '\n';
		return 1;
	}*/

	cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cerr << "Failed to open camera" << std::endl;
        return -1;
    }

    // Optionally set resolution
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cap.set(cv::CAP_PROP_FPS, 30);

    cv::Mat frame, gray;

    while (true) {
        cap >> frame; // Capture a frame
        if (frame.empty()) continue;

        // Convert to grayscale
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        // Show the grayscale frame
		std::cout << 123 << '\n';
        cv::imshow("Grayscale Camera", gray);

        // Exit on ESC key
        if (cv::waitKey(1) == 27) break;
    }
}
