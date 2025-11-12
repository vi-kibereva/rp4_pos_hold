#include "posHold/Drone.h"

Drone::Drone(msp::Msp& msp) :
    m_msp{ &msp },
    m_camera(0)
{
}

[[nodiscard]] cv::Mat Drone::getGrayscaleImage()
{
    cv::Mat frame;
    m_camera >> frame;
    cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
    return frame;
}

[[nodiscard]] Drone::GyroData Drone::getGyroData()
{
    // gyroData[0] - absolute rotation angle (not velocity) around horizontal forward-backward world axis (roll)
    // gyroData[1] - absolute rotation angle (not velocity) around left-right world axis (pitch)
    // gyroData[2] - absolute rotation angle (not velocity) around vertical world axis (yaw)

    msp::AttitudeData data = m_msp->attitude();
    return {
        data.roll_tenths * CV_PI / 180,
        data.pitch_tenths * CV_PI / 180,
        data.yaw_tenths * CV_PI / 180
    };
}

[[nodiscard]] double Drone::getAltitude()
{
    return m_msp->altitude().altitude / 100.0;
}
