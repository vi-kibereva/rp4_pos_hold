#include "posHold/Drone.hpp"

Drone::Drone(msp::Msp& msp) :
    m_msp{ &msp },
    m_camera(cameraInfo.resolutionY, cameraInfo.resolutionX, cameraInfo.fps)
{
    m_camera.start_camera();
}

[[nodiscard]] cv::Mat Drone::getGrayscaleImage()
{
    cv::Mat frame = m_camera.get_frame();
    cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
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

Drone::~Drone()
{
    m_camera.stop_camera();
}
