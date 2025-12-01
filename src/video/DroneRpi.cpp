#include "posHold/DroneRpi.h"

DroneRpi::DroneRpi() :
    m_cameraRpi()
{
    // m_cameraRpi.set(cv::CAP_PROP_FRAME_WIDTH, cameraInfo.resolutionX);
    // m_cameraRpi.set(cv::CAP_PROP_FRAME_HEIGHT, cameraInfo.resolutionY);
}

DroneRpi::DroneRpi(msp::Msp& msp) :
    m_msp{ &msp },
    m_cameraRpi()
{
    // m_cameraRpi.set(cv::CAP_PROP_FRAME_WIDTH, cameraInfo.resolutionX);
    // m_cameraRpi.set(cv::CAP_PROP_FRAME_HEIGHT, cameraInfo.resolutionY);
}

[[nodiscard]] cv::Mat DroneRpi::getGrayscaleImage()
{
    cv::Mat frame = m_cameraRpi.get_frame();
    cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
    return frame;
}
