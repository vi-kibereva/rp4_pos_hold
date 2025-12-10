#include "posHold/VecMove.hpp"

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
    }
    else
    {
        cv::Point2f meanOpticalFlow = m_cameraOpticalFlow.getOpticalFlow();
    
        m_vecMove = (m_drone->getAltitude() / m_drone->cameraInfo.focalLength)
                  * (/*m_vecDown.getVecDownDisplacement()*/ -meanOpticalFlow);
    }
    
    double yaw = m_drone->getGyroData().yaw;

    double cy = std::cos(yaw);
    double sy = std::sin(yaw);

    cv::Point2f v_world;
    v_world.x =  cy * m_vecMove.x - sy * m_vecMove.y;
    v_world.y =  sy * m_vecMove.x + cy * m_vecMove.y;

    m_vecMove = v_world;

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
