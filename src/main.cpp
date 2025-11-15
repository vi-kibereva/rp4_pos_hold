extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>
}

#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    const char* input_url = "/dev/video0";  // Pi Camera via v4l2
    const char* output_file = "output.mp4";
    const int target_fps = 30;
    const int duration_sec = 30;
    const int max_frames = target_fps * duration_sec;  // 30s recording

    avformat_network_init();

    // -------- Open input video stream --------
    AVFormatContext* fmt_ctx = nullptr;
    if (avformat_open_input(&fmt_ctx, input_url, nullptr, nullptr) < 0) {
        std::cerr << "Could not open input!\n";
        return -1;
    }

    if (avformat_find_stream_info(fmt_ctx, nullptr) < 0) {
        std::cerr << "Could not find stream info!\n";
        return -1;
    }

    // -------- Find video stream --------
    int video_stream_idx = av_find_best_stream(fmt_ctx, AVMEDIA_TYPE_VIDEO, -1, -1, nullptr, 0);
    if (video_stream_idx < 0) {
        std::cerr << "Could not find video stream!\n";
        return -1;
    }

    AVCodecParameters* codecpar = fmt_ctx->streams[video_stream_idx]->codecpar;
    const AVCodec* codec = avcodec_find_decoder(codecpar->codec_id);
    AVCodecContext* codec_ctx = avcodec_alloc_context3(codec);
    avcodec_parameters_to_context(codec_ctx, codecpar);

    if (avcodec_open2(codec_ctx, codec, nullptr) < 0) {
        std::cerr << "Could not open codec!\n";
        return -1;
    }

    // -------- Prepare scaler to BGR24 --------
    SwsContext* sws = sws_getContext(
        codec_ctx->width,
        codec_ctx->height,
        codec_ctx->pix_fmt,
        codec_ctx->width,
        codec_ctx->height,
        AV_PIX_FMT_BGR24,
        SWS_FAST_BILINEAR,
        nullptr, nullptr, nullptr
    );

    // -------- Prepare OpenCV VideoWriter --------
    cv::VideoWriter writer;
    writer.open(
        output_file,
        cv::VideoWriter::fourcc('a', 'v', 'c', '1'), // H.264
        target_fps,
        cv::Size(codec_ctx->width, codec_ctx->height)
    );

    if (!writer.isOpened()) {
        std::cerr << "Cannot open output video file!\n";
        return -1;
    }

    // -------- Read frames --------
    AVPacket* pkt = av_packet_alloc();
    AVFrame* frame = av_frame_alloc();
    AVFrame* bgr = av_frame_alloc();

    int num_bytes = av_image_get_buffer_size(AV_PIX_FMT_BGR24, codec_ctx->width,
                                             codec_ctx->height, 1);
    uint8_t* buffer = (uint8_t*)av_malloc(num_bytes);
    av_image_fill_arrays(bgr->data, bgr->linesize, buffer,
                         AV_PIX_FMT_BGR24, codec_ctx->width, codec_ctx->height, 1);

    int frame_count = 0;

    while (av_read_frame(fmt_ctx, pkt) >= 0 && frame_count < max_frames) {
        if (pkt->stream_index == video_stream_idx) {
            avcodec_send_packet(codec_ctx, pkt);

            while (avcodec_receive_frame(codec_ctx, frame) == 0 && frame_count < max_frames) {
                // Convert to BGR
                sws_scale(
                    sws,
                    frame->data, frame->linesize,
                    0, codec_ctx->height,
                    bgr->data, bgr->linesize
                );

                // Convert to cv::Mat
                cv::Mat img(codec_ctx->height, codec_ctx->width, CV_8UC3, bgr->data[0]);

                // Save frame to file
                writer.write(img);

                frame_count++;
            }
        }
        av_packet_unref(pkt);
    }

    std::cout << "Recording finished: " << frame_count << " frames captured.\n";

    // cleanup
    writer.release();
    sws_freeContext(sws);
    av_frame_free(&frame);
    av_frame_free(&bgr);
    av_packet_free(&pkt);
    avcodec_free_context(&codec_ctx);
    avformat_close_input(&fmt_ctx);

    return 0;
}

