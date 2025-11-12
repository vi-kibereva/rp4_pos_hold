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

	std::string pipeline =
        "libcamera-vid -t 0 --inline --width 640 --height 480 "
        "--codec yuv420 --nopreview -o - | "
        "videoconvert ! appsink";

	cv::VideoCapture camera(pipeline, cv::CAP_GSTREAMER);
	if (!camera.isOpened())
	{
		std::cout << "gergerger\n";
	}
	while (true)
	{
		cv::Mat frame;
    	camera >> frame;
		if (!frame.empty())
		{
			std::cout << "showing\n";
			cv::imshow("img", frame);
		}
		else
		{
			std::cout << "not showing\n";
		}
	}
	
    //cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);

	return 0;
}
