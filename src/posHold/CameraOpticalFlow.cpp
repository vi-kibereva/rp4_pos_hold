#include "posHold/CameraOpticalFlow.hpp"

cv::VideoWriter writer(
    "output.mp4",
    cv::VideoWriter::fourcc('M','P','4','V'),
    30.0,
    cv::Size(1920, 1080)
);

CameraOpticalFlow::CameraOpticalFlow(Drone& drone)
    : m_drone(&drone), m_opticalFlow(0.0f, 0.0f)
{
}

void CameraOpticalFlow::calc(int x, int y, int len)
{
    cv::Mat gray = m_drone->getGrayscaleImage();

    // ROI
    int x0 = std::max(x - len, 0);
    int y0 = std::max(y - len, 0);
    int x1 = std::min(x + len, gray.cols - 1);
    int y1 = std::min(y + len, gray.rows - 1);
    cv::Rect roi(x0, y0, x1 - x0 + 1, y1 - y0 + 1);

    cv::Mat grayROI = gray(roi);

    // Initialize points if first frame
    if (m_prevGray.empty())
    {
        cv::goodFeaturesToTrack(grayROI, m_prevPoints, 200, 0.01, 5);
        for (auto &p : m_prevPoints) { p.x += roi.x; p.y += roi.y; }
        m_prevGray = gray.clone();
        m_opticalFlow = cv::Point2f(0, 0);
        return;
    }

    // Track points using Lucas-Kanade
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

    // --- Visualization ---
    cv::Mat vis;
    cv::cvtColor(gray, vis, cv::COLOR_GRAY2BGR);

    // Draw arrows for tracked points
    const double scale = 3.0;
    for (size_t i = 0; i < m_prevPoints.size(); ++i)
    {
        if (status[i])
        {
            cv::Point p0 = m_prevPoints[i] - (nextPoints[i] - m_prevPoints[i]);
            cv::Point p1 = nextPoints[i];
            cv::arrowedLine(vis, p0, p1, cv::Scalar(0,0,255), 1, cv::LINE_AA);
        }
    }

    // Draw mean flow arrow in corner
    cv::Point corner(50, 50);
    cv::Point cornerTo(
        50 + static_cast<int>(m_opticalFlow.x * 20.0),
        50 + static_cast<int>(m_opticalFlow.y * 20.0)
    );
    cv::arrowedLine(vis, corner, cornerTo, cv::Scalar(0,255,0), 3, cv::LINE_AA);

    // Optional text
    char buf[100];
    snprintf(buf, sizeof(buf), "Mean flow: (%.2f, %.2f)", m_opticalFlow.x, m_opticalFlow.y);
    cv::putText(vis, buf, cv::Point(50, 90), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0,255,0), 2, cv::LINE_AA);

    // Save to video
    writer.write(vis);
}

cv::Point2f CameraOpticalFlow::getOpticalFlow() const
{
    return m_opticalFlow;
}
