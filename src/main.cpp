// #include "msp/msp.hpp"

// #include <exception>
// #include <iostream>
// #include <unistd.h>
// #include <chrono>
// #include <thread>

// #include "posHold/VecMove.h"
// #include "pid/pid.hpp"

// int main(int argc, char* argv[])
// {
// 	/*if (argc < 2) {
// 		std::cerr << "Usage: " << argv[0] << " /dev/ttyUSB0\n";
// 		return 2;
// 	}

// 	const char *port = argv[1];

// 	try {
// 		// Construct MSP client
// 		msp::Msp msp(port, B115200, 10);

// 		// --- Example: MSP_STATUS ---
// 		std::cout << "frefer1" << '\n';
// 		std::cout << msp.status() << '\n';
// 		std::cout << "frefer2" << '\n';

// 		// --- Example: MSP_RC ---
// 		std::cout << msp.rc() << '\n';

// 		// --- Example: MSP_ATTITUDE ---
// 		std::cout << msp.attitude() << '\n';

// 		// --- Example: MSP_ALTITUDE ---
// 		std::cout << msp.altitude() << '\n';

// 		auto start = std::chrono::steady_clock::now();

// 		// --- Example: MSP_SET_RAW_RC (commented out for safety) ---
// 		msp::SetRawRcData rc_data(1500, 1500, 1000, 1500, 1900, 1000, 1700, 1000);
// 		std::cout << "Sending: " << rc_data << '\n';
// 		for (int i = 0; i<200; ++i){
// 			msp.setRawRc(rc_data);
// 			std::cout << msp.rc() << '\n';
// 		}
// 		std::cout << "Armed"<< '\n';
// 		msp::SetRawRcData rc_data_throttle(1500, 1500, 1300, 1500, 1900, 1000, 1700, 1000);
// 		for (int i = 0; i<200; ++i){
// 			msp.setRawRc(rc_data_throttle);
// 			std::cout << msp.rc() << '\n';
// 		}
// 		std::cout << "RC values sent successfully\n";

// 		sleep(1);

// 		std::cout << msp.rc() << '\n';

// 		return 0;

// 	} catch (const std::exception &ex) {
// 		std::cout << "Error: " << ex.what() << '\n';
// 		return 1;
// 	}*/

// 	if (argc < 2)
// 	{
// 		std::cerr << "Usage: " << argv[0] << " /dev/ttyUSB0\n";
// 		return 2;
// 	}

// 	const char *port = argv[1];

// 	try
// 	{
// 		// msp::Msp msp(port, B115200, 10);

// 		// Drone drone(msp);

// 		Drone drone{};

// 		VecMove vecMove(drone);

// 		PidController controller(1.0f, 0.0f, 0.0f, 0.0f);

// 		auto t1 = std::chrono::high_resolution_clock::now();

// 		while (true)
// 		{
// 			vecMove.calc();
// 			auto t2 = std::chrono::high_resolution_clock::now();
// 			cv::Point2f cvVecMove = vecMove.getVecMove() / (std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() / 1e6);
// 			t1 = t2;
// 			std::cout << cvVecMove << '\n';
// 			/*uint32x2_t result = controller.calculate_raw_rc(
// 				vdup_n_f32(0.0f),
// 				float32x2_t{ cvVecMove.x, cvVecMove.y }
// 			);
// 			msp.setRawRc(msp::SetRawRcData(result[0], result[1], 0, 0));*/
// 		}
// 	}
// 	catch (const std::exception& e)
// 	{
// 		std::cerr << "Error: " << e.what() << '\n';
// 		return 1;
// 	}

// 	// cv::VideoCapture cap(0);
//     // if (!cap.isOpened()) {
//     //     std::cerr << "Failed to open camera" << std::endl;
//     //     return -1;
//     // }

//     // cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
//     // cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);

//     // cv::Mat frame;

//     // while (true) {
//     //     cap >> frame; // Capture a frame

//     //     // Convert to grayscale
//     //     cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);

// 	// 	std::cout << frame.size() << '\n';

//     //     // Exit on ESC key
//     //     if (cv::waitKey(1) == 27) break;
//     // }
// }

#include <iostream>
#include <opencv2/opencv.hpp>


#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>




#include <new>
#include <string>
#include <sstream>

#define INPUT_WIDTH 3264
#define INPUT_HEIGHT 2464

#define DISPLAY_WIDTH 640
#define DISPLAY_HEIGHT 480

#define CAMERA_FRAMERATE 21/1
#define FLIP 2


#include <raspicam/raspicam_cv.h>


void DisplayVersion()
{
    std::cout << "OpenCV version: " << cv::getVersionMajor() << "." << cv::getVersionMinor() << "." << cv::getVersionRevision() << std::endl;
}


int main(int argc, const char** argv)
{

    DisplayVersion();

    std::stringstream ss;

    ss << "nvarguscamerasrc !  video/x-raw(memory:NVMM), width=3264, height=2464, format=NV12, framerate=21/1 ! nvvidconv flip-method=2 ! video/x-raw, width=480, height=680, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink";

    //ss << "nvarguscamerasrc !  video/x-raw(memory:NVMM), width=" << INPUT_WIDTH <<
    //", height=" << INPUT_HEIGHT <<
    //", format=NV12, framerate=" << CAMERA_FRAMERATE <<
    //" ! nvvidconv flip-method=" << FLIP <<
    //" ! video/x-raw, width=" << DISPLAY_WIDTH <<
    //", height=" << DISPLAY_HEIGHT <<
    //", format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink";

    cv::VideoCapture video;

    video.open(ss.str());

    if (!video.isOpened())
    {
        std::cout << "Unable to get video from the camera!" << std::endl;

        return -1;
    }

    std::cout << "Got here!" << std::endl;

    cv::Mat frame;

    // Create VideoWriter
    int frame_width = static_cast<int>(cap.get(CAP_PROP_FRAME_WIDTH));
    int frame_height = static_cast<int>(cap.get(CAP_PROP_FRAME_HEIGHT));
    VideoWriter writer(
        "output.mp4",
        VideoWriter::fourcc('m','p','4','v'),  // MP4
        30.0,
        Size(frame_width, frame_height)
    );
    if (!writer.isOpened()) {
        cerr << "Could not open output file for write" << endl;
        return -1;
    }

    cout << "Recording... Press ESC to stop." << endl;

    Mat frame;
    for (int i = 0; i < 150; ++i) {
		std::cout << i << '\n';
        // Grab and retrieve frame
        // cap.grab();
        // cap.retrieve(frame);
        video.read(frame);
        if (frame.empty()) {
            cerr << "Empty frame, exiting..." << endl;
            break;
        }

        // Write frame to video
        writer.write(frame);
    }
    // video.release()
    writer.release();
    destroyAllWindows();

    cout << "Video saved as output.mp4" << endl;
    return 0;

//     while (video.read(frame))
//     {
//         cv::imshow("Video feed", frame);

//         if (cv::waitKey(25) >= 0)
//         {
//             break;
//         }
//    }

//     std::cout << "Finished!" << std::endl;

//     return 0;
}


// int main()
// {
//     using namespace std;
//     using namespace cv;

//     // Create camera object
//     raspicam::RaspiCam_Cv cap;

//     // Set camera properties if needed
//     cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
//     cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
//     cap.set(cv::CAP_PROP_FPS, 30);

//     // Open camera
//     if (!cap.open()) {
//         cerr << "Could not initialize capturing..." << endl;
//         return -1;
//     }

//     cout << "Camera opened successfully!" << endl;

//     // Create VideoWriter
//     int frame_width = static_cast<int>(cap.get(CAP_PROP_FRAME_WIDTH));
//     int frame_height = static_cast<int>(cap.get(CAP_PROP_FRAME_HEIGHT));
//     VideoWriter writer(
//         "output.mp4",
//         VideoWriter::fourcc('m','p','4','v'),  // MP4
//         30.0,
//         Size(frame_width, frame_height)
//     );
//     if (!writer.isOpened()) {
//         cerr << "Could not open output file for write" << endl;
//         return -1;
//     }

//     cout << "Recording... Press ESC to stop." << endl;

//     Mat frame;
//     for (int i = 0; i < 150; ++i) {
// 		std::cout << i << '\n';
//         // Grab and retrieve frame
//         cap.grab();
//         cap.retrieve(frame);

//         if (frame.empty()) {
//             cerr << "Empty frame, exiting..." << endl;
//             break;
//         }

//         // Write frame to video
//         writer.write(frame);
//     }

//     cap.release();
//     writer.release();
//     destroyAllWindows();

//     cout << "Video saved as output.mp4" << endl;
//     return 0;
// }