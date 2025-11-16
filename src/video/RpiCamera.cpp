#include <iostream>
#include <stdexcept>
#include <unordered_map>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/pixdesc.h>
#include <libavdevice/avdevice.h>
}

#include "video/RpiCamera.hpp"


namespace video {

static const std::unordered_map<::AVPixelFormat, int> pixel_format_scores = {
    {::AV_PIX_FMT_BGR24, 100},     // Perfect match - no conversion needed
    {::AV_PIX_FMT_RGB24, 90},      // Trivial conversion (channel swap)
    {::AV_PIX_FMT_YUYV422, 70},    // Simple YUV conversion
    {::AV_PIX_FMT_UYVY422, 70},    // Simple YUV conversion
    {::AV_PIX_FMT_YUV422P, 65},    // Planar YUV
    {::AV_PIX_FMT_YUV420P, 65},    // Planar YUV
    {::AV_PIX_FMT_NV12, 65},       // Semi-planar YUV
    {::AV_PIX_FMT_NV21, 65},       // Semi-planar YUV
};

static const std::unordered_map<::AVCodecID, int> codec_scores = {
    {::AV_CODEC_ID_MJPEG, 50},     // Good OpenCV compatibility, needs decode
    {::AV_CODEC_ID_H264, 30},      // More complex decode
};

int RpiCamera::score_format_for_opencv(::AVCodecID codec_id, ::AVPixelFormat pix_fmt) {
    int score = 0;

    auto pix_it = pixel_format_scores.find(pix_fmt);
    if (pix_it != pixel_format_scores.end())
        score = pix_it->second;

    if (!score) {
        auto codec_it = codec_scores.find(codec_id);
        if (codec_it != codec_scores.end())
            score = std::max(score, codec_it->second);
    }

    return score;
}

void RpiCamera::convert_rgb24_to_bgr24(const ::AVFrame* src, ::AVFrame* dst) {
    const int pixels_per_row = width_ * 3;
    for (int y = 0; y < height_; ++y) {
        const uint8_t* src_row = src->data[0] + y * src->linesize[0];
        uint8_t* dst_row = dst->data[0] + y * dst->linesize[0];

        for (int x = 0; x < pixels_per_row; x += 3) {
            dst_row[x + 0] = src_row[x + 2];  // B = R
            dst_row[x + 1] = src_row[x + 1];  // G = G
            dst_row[x + 2] = src_row[x + 0];  // R = B
        }
    }
}

void RpiCamera::init_converter() {
    width_ = codec_ctx_->width;
    height_ = codec_ctx_->height;
    ::AVPixelFormat input_fmt = static_cast<::AVPixelFormat>(codec_ctx_->pix_fmt);
    ::AVPixelFormat target_fmt = ::AV_PIX_FMT_BGR24;

    needs_conversion_ = (input_fmt != target_fmt);

    bgr_frame_ = ::av_frame_alloc();
    if (!bgr_frame_)
        throw std::runtime_error(std::string("Failed to allocate BGR frame"));

    int num_bytes = ::av_image_get_buffer_size(target_fmt, width_, height_, 1);
    bgr_buffer_ = static_cast<uint8_t*>(::av_malloc(num_bytes));
    if (!bgr_buffer_) {
        ::av_frame_free(&bgr_frame_);
        throw std::runtime_error(std::string("Failed to allocate BGR buffer"));
    }

    ::av_image_fill_arrays(bgr_frame_->data, bgr_frame_->linesize, bgr_buffer_,
                          target_fmt, width_, height_, 1);

    if (needs_conversion_ && input_fmt != ::AV_PIX_FMT_RGB24) {
        sws_ctx_ = ::sws_getContext(
            width_, height_, input_fmt,
            width_, height_, target_fmt,
            SWS_BILINEAR, nullptr, nullptr, nullptr
        );
        if (!sws_ctx_) {
            ::av_free(bgr_buffer_);
            ::av_frame_free(&bgr_frame_);
            throw std::runtime_error(std::string("Failed to initialize swscale context"));
        }
    } else {
        sws_ctx_ = nullptr;
    }
}

void RpiCamera::find_codec(int codec_type, const ::AVCodec **codec, const ::AVCodecParameters **codec_params){
    ::avformat_find_stream_info(format_ctx_, NULL);

    int best_score = -1;
    int best_stream_idx = -1;
    const ::AVCodec *best_codec = nullptr;
    const ::AVCodecParameters *best_codec_params = nullptr;

    for (int i = 0; i < format_ctx_->nb_streams; i++) {
        ::AVCodecParameters *local_codec_parameters =
            format_ctx_->streams[i]->codecpar;

        const ::AVCodec *local_codec =
            ::avcodec_find_decoder(local_codec_parameters->codec_id);

        if (local_codec->type == codec_type) {
            // Score this stream based on OpenCV compatibility
            int score = score_format_for_opencv(
                local_codec_parameters->codec_id,
                static_cast<::AVPixelFormat>(local_codec_parameters->format)
            );

            std::cout << "Stream " << i << ": "
                      << local_codec->long_name
                      << ", Format: " << ::av_get_pix_fmt_name(static_cast<::AVPixelFormat>(local_codec_parameters->format))
                      << ", Score: " << score << "\n";

            if (score > best_score) {
                best_score = score;
                best_stream_idx = i;
                best_codec = local_codec;
                best_codec_params = local_codec_parameters;
            }
        }
    }

    if (best_stream_idx >= 0) {
        *codec = best_codec;
        *codec_params = best_codec_params;
        stream_id_ = best_stream_idx;

        std::cout << "Selected stream " << best_stream_idx
                  << " (score: " << best_score << "): "
                  << best_codec->long_name
                  << ", Format: " << ::av_get_pix_fmt_name(static_cast<::AVPixelFormat>(best_codec_params->format))
                  << ", bit_rate: " << best_codec_params->bit_rate << "\n";
    }
}

cv::Mat RpiCamera::readFrame() {
    while (true) {
        int ret = ::av_read_frame(format_ctx_, packet_);
        if (ret < 0)
            return cv::Mat();

        if (packet_->stream_index != stream_id_) {
            ::av_packet_unref(packet_);
            continue;
        }

        ret = ::avcodec_send_packet(codec_ctx_, packet_);
        ::av_packet_unref(packet_);
        if (ret < 0)
            continue;

        ret = ::avcodec_receive_frame(codec_ctx_, frame_);
        if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
            continue;

        if (ret < 0)
            return cv::Mat();

        ::AVPixelFormat fmt = static_cast<::AVPixelFormat>(frame_->format);

        if (fmt == ::AV_PIX_FMT_BGR24)
            return cv::Mat(height_, width_, CV_8UC3, frame_->data[0], frame_->linesize[0]);

        if (fmt == ::AV_PIX_FMT_RGB24) {
            convert_rgb24_to_bgr24(frame_, bgr_frame_);
            return cv::Mat(height_, width_, CV_8UC3, bgr_buffer_, bgr_frame_->linesize[0]);
        }

        if (sws_ctx_) {
            ::sws_scale(sws_ctx_, frame_->data, frame_->linesize,
                       0, height_, bgr_frame_->data, bgr_frame_->linesize);
            return cv::Mat(height_, width_, CV_8UC3, bgr_buffer_, bgr_frame_->linesize[0]);
        }

        return cv::Mat();
    }
}

RpiCamera::RpiCamera(std::string device) {

        
    format_ctx_ = ::avformat_alloc_context();
    if (format_ctx_ == nullptr)
        throw std::runtime_error(std::string("Error allocating the AVFormatContext"));

    ::avdevice_register_all();

    const int result = ::avformat_open_input(&format_ctx_, device.c_str(), NULL, NULL);
    if (result < 0) {
        ::avformat_free_context(format_ctx_);

        char errbuf[128];
        ::av_strerror(result, errbuf, sizeof(errbuf));

        throw std::runtime_error(std::string("Error opening input: ") + errbuf);
    }


    const ::AVCodec *video_codec;
    const ::AVCodecParameters *video_codec_parameters;

    find_codec(::AVMEDIA_TYPE_VIDEO, &video_codec, &video_codec_parameters);
    if (stream_id_ == -1)
        throw std::runtime_error(std::string("Can't find the video codec"));

    codec_ctx_ = ::avcodec_alloc_context3(video_codec);
    if (codec_ctx_ == nullptr)
        throw std::runtime_error(std::string("Failed to allocate memory for AVCodecContext"));

    if (::avcodec_parameters_to_context(codec_ctx_, video_codec_parameters) < 0)
        throw std::runtime_error(std::string("Failed to copy codec params to codec context"));

    if (::avcodec_open2(codec_ctx_, video_codec, NULL) < 0)
        throw std::runtime_error(std::string("Failed to open codec through avcodec_open2"));

    frame_ = ::av_frame_alloc();
    if (!frame_)
        throw std::runtime_error(std::string("Failed to allocate memory for AVFrame"));

    packet_ = ::av_packet_alloc();
    if (!packet_)
        throw std::runtime_error(std::string("Failed to allocate memory for AVPacket"));

    init_converter();
}

RpiCamera::~RpiCamera() {
    if (sws_ctx_)
        ::sws_freeContext(sws_ctx_);
    if (bgr_buffer_)
        ::av_free(bgr_buffer_);
    if (bgr_frame_)
        ::av_frame_free(&bgr_frame_);
    if (frame_)
        ::av_frame_free(&frame_);
    if (packet_)
        ::av_packet_free(&packet_);
    if (codec_ctx_)
        ::avcodec_free_context(&codec_ctx_);
    if (format_ctx_)
        ::avformat_close_input(&format_ctx_);
}

RpiCamera::RpiCamera(const RpiCamera& other) {
    throw std::runtime_error(std::string("RpiCamera cannot be copied"));
}

RpiCamera& RpiCamera::operator=(const RpiCamera& other) {
    throw std::runtime_error(std::string("RpiCamera cannot be copied"));
}

RpiCamera::RpiCamera(RpiCamera&& other) noexcept
    : format_ctx_(other.format_ctx_),
      codec_ctx_(other.codec_ctx_),
      packet_(other.packet_),
      frame_(other.frame_),
      sws_ctx_(other.sws_ctx_),
      bgr_frame_(other.bgr_frame_),
      bgr_buffer_(other.bgr_buffer_),
      stream_id_(other.stream_id_),
      width_(other.width_),
      height_(other.height_),
      needs_conversion_(other.needs_conversion_) {

    other.format_ctx_ = nullptr;
    other.codec_ctx_ = nullptr;
    other.packet_ = nullptr;
    other.frame_ = nullptr;
    other.sws_ctx_ = nullptr;
    other.bgr_frame_ = nullptr;
    other.bgr_buffer_ = nullptr;
    other.stream_id_ = -1;
    other.width_ = 0;
    other.height_ = 0;
    other.needs_conversion_ = false;
}

RpiCamera& RpiCamera::operator=(RpiCamera&& other) noexcept {
    if (this != &other) {
        if (sws_ctx_)
            ::sws_freeContext(sws_ctx_);
        if (bgr_buffer_)
            ::av_free(bgr_buffer_);
        if (bgr_frame_)
            ::av_frame_free(&bgr_frame_);
        if (frame_)
            ::av_frame_free(&frame_);
        if (packet_)
            ::av_packet_free(&packet_);
        if (codec_ctx_)
            ::avcodec_free_context(&codec_ctx_);
        if (format_ctx_)
            ::avformat_close_input(&format_ctx_);

        format_ctx_ = other.format_ctx_;
        codec_ctx_ = other.codec_ctx_;
        packet_ = other.packet_;
        frame_ = other.frame_;
        sws_ctx_ = other.sws_ctx_;
        bgr_frame_ = other.bgr_frame_;
        bgr_buffer_ = other.bgr_buffer_;
        stream_id_ = other.stream_id_;
        width_ = other.width_;
        height_ = other.height_;
        needs_conversion_ = other.needs_conversion_;

        other.format_ctx_ = nullptr;
        other.codec_ctx_ = nullptr;
        other.packet_ = nullptr;
        other.frame_ = nullptr;
        other.sws_ctx_ = nullptr;
        other.bgr_frame_ = nullptr;
        other.bgr_buffer_ = nullptr;
        other.stream_id_ = -1;
        other.width_ = 0;
        other.height_ = 0;
        other.needs_conversion_ = false;
    }
    return *this;
}

}
