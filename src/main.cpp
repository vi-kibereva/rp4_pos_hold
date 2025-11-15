#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <mutex>
#include <thread>
#include <atomic>

std::atomic<bool> running{true};
std::mutex frame_mutex;
cv::Mat latest_frame;

GstFlowReturn on_new_sample(GstAppSink *appsink, gpointer user_data) {
    GstSample *sample = gst_app_sink_pull_sample(appsink);
    if (!sample) return GST_FLOW_ERROR;

    GstBuffer *buffer = gst_sample_get_buffer(sample);
    GstCaps *caps = gst_sample_get_caps(sample);
    GstStructure *s = gst_caps_get_structure(caps, 0);

    int width, height;
    gst_structure_get_int(s, "width", &width);
    gst_structure_get_int(s, "height", &height);

    GstMapInfo map;
    if (gst_buffer_map(buffer, &map, GST_MAP_READ)) {
        cv::Mat frame(height, width, CV_8UC3, (char*)map.data);
        {
            std::lock_guard<std::mutex> lock(frame_mutex);
            frame.copyTo(latest_frame);  // Only copy once
        }
        gst_buffer_unmap(buffer, &map);
    }

    gst_sample_unref(sample);
    return GST_FLOW_OK;
}

int main(int argc, char **argv) {
    gst_init(&argc, &argv);

    // DMABUF-based high-performance pipeline
    std::string pipeline_str = R"(
libcamerasrc ! video/x-raw,width=1280,height=720,framerate=30/1,format=NV12 ! \
    tee name=t \
        t. ! queue ! videoconvert ! video/x-raw,format=I420 ! \
             x264enc tune=zerolatency bitrate=1500 speed-preset=superfast ! \
             h264parse ! mp4mux ! filesink location=output.mp4 sync=false \
        t. ! queue ! glupload ! glcolorconvert ! appsink name=appsink0 emit-signals=true max-buffers=2 drop=true
)";

    GError *error = nullptr;
    GstElement *pipeline = gst_parse_launch(pipeline_str.c_str(), &error);
    if (!pipeline) {
        std::cerr << "Failed to create pipeline: " << error->message << std::endl;
        g_clear_error(&error);
        return -1;
    }

    GstElement *appsink = gst_bin_get_by_name(GST_BIN(pipeline), "appsink0");
    gst_app_sink_set_emit_signals((GstAppSink*)appsink, true);
    gst_app_sink_set_max_buffers((GstAppSink*)appsink, 2);
    gst_app_sink_set_drop((GstAppSink*)appsink, true);
    g_signal_connect(appsink, "new-sample", G_CALLBACK(on_new_sample), nullptr);

    gst_element_set_state(pipeline, GST_STATE_PLAYING);

    // OpenCV display thread
    std::thread cv_thread([](){
        while (running) {
            cv::Mat frame;
            {
                std::lock_guard<std::mutex> lock(frame_mutex);
                if (!latest_frame.empty())
                    latest_frame.copyTo(frame);
            }
            if (!frame.empty()) {
                cv::imshow("Camera", frame);
                if (cv::waitKey(1) == 27) break;  // ESC to quit
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });

    // Wait for Ctrl+C
    std::cout << "Streaming... Press Ctrl+C to stop." << std::endl;
    std::signal(SIGINT, [](int){ running = false; });

    while (running) std::this_thread::sleep_for(std::chrono::milliseconds(100));

    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(appsink);
    gst_object_unref(pipeline);
    cv_thread.join();

    return 0;
}

