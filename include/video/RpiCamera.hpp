#ifndef RPI_CAMERA_HPP
#define RPI_CAMERA_HPP

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavcodec/packet.h>
#include <libavformat/avformat.h>
#include <libavcodec/codec.h>
#include <libswscale/swscale.h>
#include <libavutil/imgutils.h>
}

#include <opencv2/core.hpp>

namespace video {

class RpiCamera {
public:
    RpiCamera() = delete;
    RpiCamera(std::string device);

    ~RpiCamera();

    RpiCamera(const RpiCamera& other);
    RpiCamera& operator=(const RpiCamera& other);

    RpiCamera(RpiCamera&& other) noexcept;
    RpiCamera& operator=(RpiCamera&& other) noexcept;

    cv::Mat readFrame();

private:
    ::AVFormatContext* format_ctx_;
    ::AVCodecContext* codec_ctx_;
    ::AVPacket* packet_;
    ::AVFrame* frame_;

    ::SwsContext* sws_ctx_;
    ::AVFrame* bgr_frame_;
    uint8_t* bgr_buffer_;

    int stream_id_ = -1;
    int width_;
    int height_;
    bool needs_conversion_;

    void find_codec(int codec_type, const ::AVCodec **codec, const ::AVCodecParameters **codec_params);
    int score_format_for_opencv(::AVCodecID codec_id, ::AVPixelFormat pix_fmt);
    void init_converter();
    void convert_rgb24_to_bgr24(const ::AVFrame* src, ::AVFrame* dst);

};

} // namespace video

#endif // RPI_CAMERA_HPP
