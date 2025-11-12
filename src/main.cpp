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
	if (argc < 2) {
		std::cerr << "Usage: " << argv[0] << " /dev/ttyUSB0\n";
		return 2;
	}

	const char *port = argv[1];

	try {
		// Construct MSP client
		msp::Msp msp(port, B115200, 10);

		// --- Example: MSP_STATUS ---
		std::cout << "frefer1" << '\n';
		std::cout << msp.status() << '\n';
		std::cout << "frefer2" << '\n';

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
		std::cout << "Error: " << ex.what() << '\n';
		return 1;
	}

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

		PidController controller(1.0f, 0.0f, 0.0f, 0.0f);

		auto t1 = std::chrono::high_resolution_clock::now();

		while (true)
		{
			std::cout << "2\n";
			vecMove.calc();
			std::cout << "3\n";
			auto t2 = std::chrono::high_resolution_clock::now();
			cv::Point2f cvVecMove = vecMove.getVecMove() / (std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() / 1e6);
			t1 = t2;
			std::cout << cvVecMove << '\n';
			uint32x2_t result = controller.calculate_raw_rc(
				vdup_n_f32(0.0f),
				float32x2_t{ cvVecMove.x, cvVecMove.y }
			);
			msp.setRawRc(msp::SetRawRcData(result[0], result[1], 0, 0));
		}
	}
	catch (const std::exception& e)
	{
		std::cerr << "Error: " << e.what() << '\n';
		return 1;
	}*/

	// cv::VideoCapture cap(0);
    // if (!cap.isOpened()) {
    //     std::cerr << "Failed to open camera" << std::endl;
    //     return -1;
    // }

    // cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    // cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);

    // cv::Mat frame;

    // while (true) {
    //     cap >> frame; // Capture a frame

    //     // Convert to grayscale
    //     cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);

	// 	std::cout << frame.size() << '\n';

    //     // Exit on ESC key
    //     if (cv::waitKey(1) == 27) break;
    // }
}
