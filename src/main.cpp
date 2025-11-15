extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>
#include <libavutil/imgutils.h>
}

#include <opencv2/opencv.hpp>
#include <iostream>
#include <csignal>

volatile bool stop_signal = false;

// Graceful stop on Ctrl+C
void signal_handler(int signum) {
    stop_signal = true;
}

int main() {
    const char* input_url = "/dev/video0"; // Pi Camera via v4l2
    const char* output_file = "output.mp4";
    const int target_fps = 30;
    const int duration_sec = 30;
    const int max_frames = target_fps * duration_sec;

    // Register Ctrl+C handler
    std::signal(SIGINT, signal_handler);

    avformat_network_init();

    // -------- Open input video stream with V4L2 --------
    AVFormatContext* fmt_ctx = avformat_alloc_context();

    if (avformat_open_input(&fmt_ctx, input_url, nullptr, nullptr) < 0) {
        std::cerr << "Could not open input!\n";
        return -1;
    }


    if (avformat_find_stream_info(fmt_ctx, nullptr) < 0) {
        std::cerr << "[ERROR] Could not find stream info!\n";
        avformat_close_input(&fmt_ctx);
        return -1;
    }

    int video_stream_idx = av_find_best_stream(fmt_ctx, AVMEDIA_TYPE_VIDEO, -1, -1, nullptr, 0);
    if (video_stream_idx < 0) {
        std::cerr << "[ERROR] Could not find video stream!\n";
        avformat_close_input(&fmt_ctx);
        return -1;
    }

    AVCodecParameters* codecpar = fmt_ctx->streams[video_stream_idx]->codecpar;
    const AVCodec* codec = avcodec_find_decoder(codecpar->codec_id);
    if (!codec) {
        std::cerr << "[ERROR] Codec not found!\n";
        avformat_close_input(&fmt_ctx);
        return -1;
    }

    AVCodecContext* codec_ctx = avcodec_alloc_context3(codec);
    if (!codec_ctx) {
        std::cerr << "[ERROR] Could not allocate codec context!\n";
        avformat_close_input(&fmt_ctx);
        return -1;
    }

    if (avcodec_parameters_to_context(codec_ctx, codecpar) < 0) {
        std::cerr << "[ERROR] Failed to copy codec parameters to context!\n";
        avcodec_free_context(&codec_ctx);
        avformat_close_input(&fmt_ctx);
        return -1;
    }

    if (avcodec_open2(codec_ctx, codec, nullptr) < 0) {
        std::cerr << "[ERROR] Could not open codec!\n";
        avcodec_free_context(&codec_ctx);
        avformat_close_input(&fmt_ctx);
        return -1;
    }

    // -------- Prepare OpenCV VideoWriter --------
    cv::VideoWriter writer;
    if (!writer.open(output_file,
                     cv::VideoWriter::fourcc('a','v','c','1'),
                     target_fps,
                     cv::Size(codec_ctx->width, codec_ctx->height))) {
        std::cerr << "[ERROR] Cannot open output video file: " << output_file << "\n";
        avcodec_free_context(&codec_ctx);
        avformat_close_input(&fmt_ctx);
        return -1;
    }

    // -------- Prepare SwsContext if needed --------
    SwsContext* sws = nullptr;
    AVPixelFormat input_pix_fmt = codec_ctx->pix_fmt;
    AVPixelFormat target_pix_fmt = AV_PIX_FMT_BGR24;

    if (input_pix_fmt != target_pix_fmt) {
        sws = sws_getContext(codec_ctx->width, codec_ctx->height, input_pix_fmt,
                             codec_ctx->width, codec_ctx->height, target_pix_fmt,
                             SWS_FAST_BILINEAR, nullptr, nullptr, nullptr);
        if (!sws) {
            std::cerr << "[ERROR] Could not create SwsContext!\n";
            writer.release();
            avcodec_free_context(&codec_ctx);
            avformat_close_input(&fmt_ctx);
            return -1;
        }
    }

    // -------- Allocate frames and buffers --------
    AVPacket* pkt = av_packet_alloc();
    AVFrame* frame = av_frame_alloc();
    AVFrame* bgr = av_frame_alloc();
    if (!pkt || !frame || !bgr) {
        std::cerr << "[ERROR] Could not allocate frames or packet!\n";
        writer.release();
        if (sws) sws_freeContext(sws);
        av_frame_free(&frame);
        av_frame_free(&bgr);
        av_packet_free(&pkt);
        avcodec_free_context(&codec_ctx);
        avformat_close_input(&fmt_ctx);
        return -1;
    }

    int num_bytes = av_image_get_buffer_size(target_pix_fmt, codec_ctx->width,
                                             codec_ctx->height, 1);
    uint8_t* buffer = (uint8_t*)av_malloc(num_bytes);
    av_image_fill_arrays(bgr->data, bgr->linesize, buffer,
                         target_pix_fmt, codec_ctx->width, codec_ctx->height, 1);

    std::cout << "[INFO] Starting recording for " << duration_sec << " seconds...\n";

    int frame_count = 0;
    while (!stop_signal && av_read_frame(fmt_ctx, pkt) >= 0 && frame_count < max_frames) {
        if (pkt->stream_index == video_stream_idx) {
            if (avcodec_send_packet(codec_ctx, pkt) < 0) {
                av_packet_unref(pkt);
                continue;
            }

            while (avcodec_receive_frame(codec_ctx, frame) == 0 && frame_count < max_frames) {
                if (sws) {
                    sws_scale(sws, frame->data, frame->linesize, 0, codec_ctx->height,
                              bgr->data, bgr->linesize);
                } else {
                    // If BGR24, copy data pointer
                    av_image_copy(bgr->data, bgr->linesize,
                                  (const uint8_t**)frame->data, frame->linesize,
                                  target_pix_fmt, codec_ctx->width, codec_ctx->height);
                }

                cv::Mat img(codec_ctx->height, codec_ctx->width, CV_8UC3, bgr->data[0]);
                writer.write(img);

                frame_count++;
            }
        }
        av_packet_unref(pkt);
    }

    std::cout << "[INFO] Recording finished: " << frame_count << " frames captured.\n";

    // -------- Cleanup --------
    writer.release();
    if (sws) sws_freeContext(sws);
    av_free(buffer);
    av_frame_free(&frame);
    av_frame_free(&bgr);
    av_packet_free(&pkt);
    avcodec_free_context(&codec_ctx);
    avformat_close_input(&fmt_ctx);

    return 0;
}

