#include "posHold/Drone.h"

Drone::Drone() :
    m_camera(0)
{
    m_camera.set(cv::CAP_PROP_FRAME_WIDTH, cameraInfo.resolutionX);
    m_camera.set(cv::CAP_PROP_FRAME_HEIGHT, cameraInfo.resolutionY);
}

Drone::Drone(msp::Msp& msp) :
    m_msp{ &msp },
    m_camera(0)
{
    m_camera.set(cv::CAP_PROP_FRAME_WIDTH, cameraInfo.resolutionX);
    m_camera.set(cv::CAP_PROP_FRAME_HEIGHT, cameraInfo.resolutionY);
}

[[nodiscard]] cv::Mat Drone::getGrayscaleImage()
{
    std::cout << "Getting image...\n";
    cv::Mat frame;
    m_camera >> frame;
    cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
    std::cout << "Got image\n";
    return frame;
}

[[nodiscard]] Drone::GyroData Drone::getGyroData()
{
    // gyroData.roll - absolute rotation angle (not velocity) around horizontal forward-backward world axis
    // gyroData.pitch - absolute rotation angle (not velocity) around left-right world axis
    // gyroData.yaw - absolute rotation angle (not velocity) around vertical world axis

    return { 0.0, 0.0, 0.0 };

    msp::AttitudeData data = m_msp->attitude();
    return {
        data.roll_tenths * CV_PI / 1800,
        data.pitch_tenths * CV_PI / 1800,
        data.yaw_tenths * CV_PI / 1800
    };
}

[[nodiscard]] double Drone::getAltitude()
{
    return 1.0;
    return m_msp->altitude().altitude / 100.0;
}
