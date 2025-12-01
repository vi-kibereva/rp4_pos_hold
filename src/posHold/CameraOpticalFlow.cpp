#include "CameraOpticalFlow.hpp"

CameraOpticalFlow::CameraOpticalFlow(Drone& drone)
    : m_drone(&drone), m_opticalFlow(0.0f, 0.0f)
{
}

void CameraOpticalFlow::calc(int x, int y, int len)
{
    cv::Mat gray = m_drone->getGrayscaleImage();

    // Define ROI
    int x0 = std::max(x - len, 0);
    int y0 = std::max(y - len, 0);
    int x1 = std::min(x + len, gray.cols - 1);
    int y1 = std::min(y + len, gray.rows - 1);
    cv::Rect roi(x0, y0, x1 - x0 + 1, y1 - y0 + 1);
    cv::Mat grayROI = gray(roi);

    if (m_prevGray.empty())
    {
        // First frame: detect points
        cv::goodFeaturesToTrack(grayROI, m_prevPoints, 200, 0.01, 5);
        for (auto &p : m_prevPoints) {
            p.x += roi.x;
            p.y += roi.y;  // adjust to full frame coordinates
        }
        m_prevGray = gray.clone();
        m_opticalFlow = cv::Point2f(0, 0);
        return;
    }

    // Track points using LK
    std::vector<cv::Point2f> nextPoints;
    std::vector<uchar> status;
    std::vector<float> err;
    cv::calcOpticalFlowPyrLK(m_prevGray, gray, m_prevPoints, nextPoints, status, err);

    // Compute mean flow
    cv::Point2f flowSum(0.0f, 0.0f);
    int count = 0;
    for (size_t i = 0; i < status.size(); ++i)
    {
        if (status[i])
        {
            cv::Point2f f = nextPoints[i] - m_prevPoints[i];
            flowSum += f;
            ++count;
            m_prevPoints[i] = nextPoints[i]; // update for next frame
        }
    }

    if (count > 0)
        m_opticalFlow = flowSum * (1.0f / count);
    else
        m_opticalFlow = cv::Point2f(0, 0);

    m_prevGray = gray.clone();

    // If points are lost, detect new points in ROI
    if (count < 50)
    {
        m_prevPoints.clear();
        cv::goodFeaturesToTrack(grayROI, m_prevPoints, 200, 0.01, 5);
        for (auto &p : m_prevPoints) {
            p.x += roi.x;
            p.y += roi.y;
        }
    }
}

cv::Point2f CameraOpticalFlow::getOpticalFlow() const
{
    return m_opticalFlow;
}
