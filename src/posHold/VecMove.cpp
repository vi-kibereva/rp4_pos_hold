#include "VecMove.h"

VecMove::VecMove(const Drone& drone) :
    m_drone{ &drone },
    m_vecDown(drone),
    m_cameraOpticalFlow(drone)
{
}

void VecMove::calc()
{
    m_vecDown.calc();
    m_cameraOpticalFlow.calc();

    const cv::Point2f p = m_vecDown.getVecDown();
    cv::Point2f meanOpticalFlow{ 0.0f, 0.0f };

    const int xMin = std::max(static_cast<int>(p.x) - 10, 0);
    const int xMax = std::min(static_cast<int>(p.x) + 10, m_drone->cameraInfo.resolutionX - 1);
    const int yMin = std::max(static_cast<int>(p.y) - 10, 0);
    const int yMax = std::min(static_cast<int>(p.y) + 10, m_drone->cameraInfo.resolutionY - 1);

    for (int x = xMin; x <= xMax; ++x)
    {
        for (int y = yMin; y <= yMax; ++y)
        {
            meanOpticalFlow += m_cameraOpticalFlow.getOpticalFlowAt(x, y);
        }
    }
    meanOpticalFlow /= std::max((xMax - xMin + 1) * (yMax - yMin + 1), 1);
    m_vecMove = m_vecDown.getVecDownDisplacement() - meanOpticalFlow;
    m_hasPrev = true;
}

cv::Point2f VecMove::getVecMove() const
{
    if (!m_hasPrev)
    {
        throw std::runtime_error("VecMove::getVecMove called before calling VecMove::calc");
    }
    return m_vecMove;
}
