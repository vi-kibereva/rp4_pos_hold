#include "posHold/VecMove.h"

VecMove::VecMove(Drone& drone) :
    m_drone{ &drone },
    m_vecDown(drone),
    m_cameraOpticalFlow(drone)
{
}

void VecMove::calc()
{
    m_vecDown.calc();

    const cv::Point2f p = m_vecDown.getVecDown();

    m_cameraOpticalFlow.calc(static_cast<int>(p.x), static_cast<int>(p.y), s_accountFlowPixels);

    if (p.x < 0 || static_cast<int>(p.x) >= m_drone->cameraInfo.resolutionX
        || p.y < 0 || static_cast<int>(p.y) >= m_drone->cameraInfo.resolutionY)
    {
        m_vecMove = p * s_noFlowBalanceVecMultiplier
            / std::sqrt((m_drone->cameraInfo.resolutionX * m_drone->cameraInfo.resolutionX
                + m_drone->cameraInfo.resolutionY * m_drone->cameraInfo.resolutionY
            ));
        return;
    }

    cv::Point2f meanOpticalFlow{ 0.0f, 0.0f };

    const int xMin = std::max(static_cast<int>(p.x) - s_accountFlowPixels, 0);
    const int xMax = std::min(static_cast<int>(p.x) + s_accountFlowPixels, m_drone->cameraInfo.resolutionX - 1);
    const int yMin = std::max(static_cast<int>(p.y) - s_accountFlowPixels, 0);
    const int yMax = std::min(static_cast<int>(p.y) + s_accountFlowPixels, m_drone->cameraInfo.resolutionY - 1);

    int counter = 0;
    for (int x = xMin; x <= xMax; ++x)
    {
        for (int y = yMin; y <= yMax; ++y)
        {
            if (static_cast<long long>(p.x - x) * (p.x - x)
                + static_cast<long long>(p.y - y) * (p.y - y)
                <= static_cast<long long>(s_accountFlowPixels) * s_accountFlowPixels)
            {
                meanOpticalFlow += m_cameraOpticalFlow.getOpticalFlowAt(x, y);
                ++counter;
            }
        }
    }

    meanOpticalFlow /= counter;

    std::cout << "Mean optical flow: " << meanOpticalFlow << '\n';

    m_vecMove = (m_drone->getAltitude() / m_drone->cameraInfo.focalLength) * (m_vecDown.getVecDownDisplacement() - meanOpticalFlow);

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
