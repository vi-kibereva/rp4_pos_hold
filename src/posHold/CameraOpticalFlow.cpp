#include <opencv2/opencv.hpp>

#include "posHold/CameraOpticalFlow.h"

CameraOpticalFlow::CameraOpticalFlow(Drone& drone) :
    m_drone{ &drone }
{
}

void CameraOpticalFlow::calc(const int x, const int y, const int len)
{
    cv::Mat grayFrame = m_drone->getGrayscaleImage();

    if (m_prevFrame.empty())
    {
        m_prevFrame = grayFrame.clone();
    }

    std::cout << "Getting optical flow...\n";
    cv::calcOpticalFlowFarneback(
        m_prevFrame, grayFrame, m_opticalFlow,
        0.5,   // pyramid scale
        3,     // levels
        15,    // window size
        1,     // iterations
        5,     // poly_n
        1.2,   // poly_sigma
        0      // flags
    );
    std::cout << "Got optical flow\n";

    m_prevFrame = grayFrame.clone();
}

cv::Point2f CameraOpticalFlow::getOpticalFlowAt(const int x, const int y) const
{
    if (m_opticalFlow.empty())
    {
        throw std::runtime_error("CameraOpticalFlow::getOpticalFlowAt called before calling CameraOpticalFlow::calc");
    }
    return m_opticalFlow.at<cv::Point2f>(y, x);
}
