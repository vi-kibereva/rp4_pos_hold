#include "posHold/VecDown.h"

VecDown::VecDown(const Drone& drone) :
    m_drone{ &drone }
{
}

void VecDown::calc()
{
    if (!m_hasPrev)
    {
        m_vecDown = calcVecDownProjection();
        m_hasPrev = true;
    }

    const cv::Point2f vecDown = calcVecDownProjection();
    m_vecDownDisplacement = vecDown - m_vecDown;
    m_vecDown = vecDown;
}

cv::Point2f VecDown::getVecDown() const
{
    if (!m_hasPrev)
    {
        throw std::runtime_error("VecDown::getVecDown called before calling VecDown::calc");
    }
    return m_vecDown;
}

cv::Point2f VecDown::getVecDownDisplacement() const
{
    if (!m_hasPrev)
    {
        throw std::runtime_error("VecDown::getVecDownDisplacement called before calling VecDown::calc");
    }
    return m_vecDownDisplacement;
}

[[nodiscard]] cv::Point2f getVecDownDisplacement();

cv::Vec3d VecDown::calcVecDown3d() const
{
    Drone::GyroData gyroData = m_drone->getGyroData();

    cv::Vec3f vecDown{ 0.0f, 0.0f, -1.0f };

    cv::Matx33d Rx(1, 0, 0,
                   0, cos(-gyroData.roll), -sin(-gyroData.roll),
                   0, sin(-gyroData.roll),  cos(-gyroData.roll));

    cv::Matx33d Ry(cos(-gyroData.pitch), 0, sin(-gyroData.pitch),
                   0, 1, 0,
                   -sin(-gyroData.pitch), 0, cos(-gyroData.pitch));

    cv::Matx33d Rz(cos(-gyroData.yaw), -sin(-gyroData.yaw), 0,
                   sin(-gyroData.yaw),  cos(-gyroData.yaw), 0,
                   0, 0, 1);

    cv::Matx33d R = Rz * Ry * Rx;

    return R * vecDown;
}

cv::Point2f VecDown::calcVecDownProjection() const
{
    cv::Vec3d v = calcVecDown3d();

    const double depth = -v[2];

    if (depth <= 0.0)
    {
        return { 0.0, 0.0 };
    }

    double x_screen = -m_drone->cameraInfo.focalLength * (v[0] / depth);
    double y_screen = m_drone->cameraInfo.focalLength * (v[1] / depth);

    float u = m_drone->cameraInfo.resolutionX / 2.0 + x_screen;
    float v_scr = m_drone->cameraInfo.resolutionY / 2.0 + y_screen;

    return {
        std::max(std::min(u, static_cast<float>(m_drone->cameraInfo.resolutionX)), 0.0f),
        std::max(std::min(v_scr, static_cast<float>(m_drone->cameraInfo.resolutionY)), 0.0f)
    };
}
