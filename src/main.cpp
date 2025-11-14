#include "msp/msp.hpp"

#include <exception>
#include <iostream>
#include <iomanip>
#include <unistd.h>
#include <chrono>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include "posHold/VecMove.h"
#include "pid/pid.hpp"

using namespace std;

int main(int argc, char* argv[])
{
	if (argc < 2)
	{
		std::cerr << "Usage: " << argv[0] << " /dev/ttyUSB0\n";
		return 2;
	}

	const char *port = argv[1];

	try
	{
		Drone drone{"rtsp://localhost:8554/stream"};

        VecMove vecMove(drone);

		PidController controller(1.0f, 0.0f, 0.0f, 0.0f);

        // Create VideoWriter
        auto& camera = drone.getCamera();

        // Check if camera opened successfully
        if (!camera.isOpened()) {
            cerr << "Error: Could not open RTSP stream (rtsp://localhost:8554/stream)" << endl;
            cerr << "Make sure the stream server is running." << endl;
            return -1;
        }

        int frame_width = static_cast<int>(camera.get(cv::CAP_PROP_FRAME_WIDTH));
        int frame_height = static_cast<int>(camera.get(cv::CAP_PROP_FRAME_HEIGHT));

        cout << "Camera opened successfully (" << frame_width << "x" << frame_height << ")" << endl;

        cv::VideoWriter writer(
            "output.mp4",
            cv::VideoWriter::fourcc('m','p','4','v'),  // MP4
            30.0,
            cv::Size(frame_width, frame_height)
        );

        if (!writer.isOpened()) {
            cerr << "Error: Could not open output file (output.mp4) for writing" << endl;
            return -1;
        }

        cout << "Starting 90-second video recording to output.mp4..." << endl;
        cout << "---------------------------------------------------" << endl;

        cv::Mat frame;
        int frame_count = 0;
        int last_progress_second = 0;

        // Time-based recording: exactly 90 seconds
        auto start_time = std::chrono::high_resolution_clock::now();
        auto target_duration = std::chrono::seconds(30);

        while (true) {
            auto current_time = std::chrono::high_resolution_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time);

            // Check if we've reached 90 seconds
            if (elapsed >= target_duration) {
                break;
            }

            // Read frame from camera
            camera.read(frame);

            // Check if frame is valid
            if (frame.empty()) {
                cerr << "Warning: Empty frame received at " << elapsed.count() / 1000.0 << " seconds" << endl;
                continue;
            }

            // Write frame to video file
            writer.write(frame);
            frame_count++;

            // Print progress every 10 seconds
            int current_second = elapsed.count() / 1000;
            if (current_second >= last_progress_second + 10) {
                double actual_fps = frame_count / (elapsed.count() / 1000.0);
                cout << "Progress: " << current_second << "s / 90s"
                     << " | Frames: " << frame_count
                     << " | FPS: " << fixed << setprecision(1) << actual_fps
                     << endl;
                last_progress_second = current_second;
            }
        }

        // Final statistics
        auto end_time = std::chrono::high_resolution_clock::now();
        auto total_duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        double actual_fps = frame_count / (total_duration.count() / 1000.0);

        writer.release();

        cout << "---------------------------------------------------" << endl;
        cout << "Recording complete!" << endl;
        cout << "Duration: " << fixed << setprecision(2) << total_duration.count() / 1000.0 << " seconds" << endl;
        cout << "Total frames: " << frame_count << endl;
        cout << "Average FPS: " << fixed << setprecision(2) << actual_fps << endl;
        cout << "Output saved to: output.mp4" << endl;
	}
	catch (const std::exception& e)
	{
		std::cerr << "Error: " << e.what() << '\n';
		return 1;
	}

	return 0;
}
