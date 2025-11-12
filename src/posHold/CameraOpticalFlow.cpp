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
        m_opticalFlow = cv::Mat::zeros(grayFrame.size(), CV_32FC2);
        return;
    }

    int x0 = std::max(x - len, 0);
    int y0 = std::max(y - len, 0);
    int x1 = std::min(x + len, grayFrame.cols - 1);
    int y1 = std::min(y + len, grayFrame.rows - 1);
    cv::Rect roi(x0, y0, x1 - x0 + 1, y1 - y0 + 1);

    cv::Mat prevROI = m_prevFrame(roi);
    cv::Mat currROI = grayFrame(roi);

    cv::Mat flowROI;
    cv::calcOpticalFlowFarneback(
        prevROI, currROI, flowROI,
        0.5,   // pyramid scale
        3,     // levels
        15,    // window size
        3,     // iterations
        5,     // poly_n
        1.2,   // poly_sigma
        0      // flags
    );

    if (m_opticalFlow.empty() || m_opticalFlow.size() != grayFrame.size())
    {
        m_opticalFlow = cv::Mat::zeros(grayFrame.size(), CV_32FC2);
    }

    flowROI.copyTo(m_opticalFlow(roi));

    // Optional: zero out outside the ROI (just in case)
    // cv::Mat mask = cv::Mat::zeros(grayFrame.size(), CV_8U);
    // mask(roi) = 1;
    // m_opticalFlow.setTo(cv::Scalar(0,0), mask == 0);

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
