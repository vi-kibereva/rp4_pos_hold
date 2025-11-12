#include <opencv4/opencv2/opencv.hpp>

#include "posHold/CameraOpticalFlow.h"

CameraOpticalFlow::CameraOpticalFlow(Drone& drone) :
    m_drone{ &drone }
{
}

void CameraOpticalFlow::calc()
{
    cv::Mat grayFrame = m_drone->getGrayscaleImage();

    cv::imshow(grayFrame);

    if (m_prevFrame.empty())
    {
        m_prevFrame = grayFrame.clone();
    }

    cv::calcOpticalFlowFarneback(
        m_prevFrame, grayFrame, m_opticalFlow,
        0.5,   // pyramid scale
        3,     // levels
        15,    // window size
        3,     // iterations
        5,     // poly_n
        1.2,   // poly_sigma
        0      // flags
    );

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
