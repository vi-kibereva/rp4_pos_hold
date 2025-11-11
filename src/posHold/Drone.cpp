#include "posHold/Drone.h"

Drone::Drone(msp::Msp& msp) :
    m_msp{ &msp }
{
}

[[nodiscard]] cv::Mat Drone::getGrayscaleImage() const
{
    // get camera grayscale image
    /*std::vector<std::uint8_t> imgBytes = std::get<0>(m_sim->getVisionSensorImg(m_visionSensor));
    cv::Mat frame(
        m_cameraFrameSize.second,
        m_cameraFrameSize.first,
        CV_8UC3, imgBytes.data());
    cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
    cv::flip(frame, frame, 0);
    return frame;*/
}

[[nodiscard]] Drone::GyroData Drone::getGyroData() const
{
    // gyroData[0] - absolute rotation angle (not velocity) around horizontal forward-backward world axis (roll)
    // gyroData[1] - absolute rotation angle (not velocity) around left-right world axis (pitch)
    // gyroData[2] - absolute rotation angle (not velocity) around vertical world axis (yaw)

    AttitudeData data = m_msp->attitude();
    return {
        data.roll_tenths * CV_PI / 180,
        data.pitch_tenths * CV_PI / 180,
        data.yaw_tenths * CV_PI / 180
    };
}

[[nodiscard]] double Drone::getAltitude() const
{
    return m_msp->altitude().altitude / 100.0;
}
