#include <opencv2/opencv.hpp>
#include "posHold/CameraOpticalFlow.hpp"
#include "posHold/Drone.hpp"
#include <iostream>

// Global video writer
cv::VideoWriter writer;

CameraOpticalFlow::CameraOpticalFlow(Drone& drone) :
    m_drone{ &drone },
    m_opticalFlow(0.0f, 0.0f)
{
    // Initialize global writer if not already opened
    if (!writer.isOpened())
    {
        writer.open("output.mp4",
                    cv::VideoWriter::fourcc('M','P','4','V'),
                    30.0,
                    cv::Size(1920, 1080));
        if (!writer.isOpened())
        {
            throw std::runtime_error("Cannot open video writer");
        }
    }
}

void CameraOpticalFlow::calc(int x, int y, int len)
{
    cv::Mat grayFrame = m_drone->getGrayscaleImage();

    int x0 = std::max(x - len, 0);
    int y0 = std::max(y - len, 0);
    int x1 = std::min(x + len, grayFrame.cols - 1);
    int y1 = std::min(y + len, grayFrame.rows - 1);
    cv::Rect roi(x0, y0, x1 - x0 + 1, y1 - y0 + 1);

    cv::Mat grayROI = grayFrame(roi);

    // First frame or no points? Detect features
    if (m_prevGray.empty() || m_prevPoints.empty())
    {
        cv::goodFeaturesToTrack(grayROI, m_prevPoints, 200, 0.01, 5);
        for (auto &p : m_prevPoints) { p.x += roi.x; p.y += roi.y; }
        m_prevGray = grayFrame.clone();
        m_opticalFlow = cv::Point2f(0, 0);
        return;
    }

    // If points exist, calculate LK flow
    if (!m_prevPoints.empty())
    {
        std::vector<cv::Point2f> nextPoints;
        std::vector<uchar> status;
        std::vector<float> err;
        cv::calcOpticalFlowPyrLK(m_prevGray, grayFrame, m_prevPoints, nextPoints, status, err);

        cv::Point2f flowSum(0, 0);
        int count = 0;

        for (size_t i = 0; i < status.size(); ++i)
        {
            if (status[i])
            {
                cv::Point2f f = nextPoints[i] - m_prevPoints[i];
                flowSum += f;
                ++count;
                m_prevPoints[i] = nextPoints[i];
            }
        }

        if (count > 0)
            m_opticalFlow = flowSum * (1.0f / count);
        else
            m_opticalFlow = cv::Point2f(0, 0);
    }

    m_prevGray = grayFrame.clone();

    // -----------------------------
    // VISUALIZATION + SAVE VIDEO
    // -----------------------------
    cv::Mat vis;
    cv::cvtColor(grayFrame, vis, cv::COLOR_GRAY2BGR);

    const double arrowScale = 3.0;

    for (const auto &p : m_prevPoints)
    {
        cv::Point2f f = m_opticalFlow; // For simplicity, draw mean flow at all points
        cv::Point p0(cvRound(p.x), cvRound(p.y));
        cv::Point p1(cvRound(p.x + f.x * arrowScale), cvRound(p.y + f.y * arrowScale));
        cv::arrowedLine(vis, p0, p1, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
    }

    // Draw mean flow in corner
    double bigScale = 20.0;
    cv::Point corner(50, 50);
    cv::Point cornerTo(
        50 + cvRound(m_opticalFlow.x * bigScale),
        50 + cvRound(m_opticalFlow.y * bigScale)
    );
    cv::arrowedLine(vis, corner, cornerTo, cv::Scalar(0, 255, 0), 3, cv::LINE_AA);

    char buf[100];
    snprintf(buf, sizeof(buf), "Mean flow: (%.2f, %.2f)", m_opticalFlow.x, m_opticalFlow.y);
    cv::putText(vis, buf, cv::Point(50, 90), cv::FONT_HERSHEY_SIMPLEX,
                0.7, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);

    // Save frame
    writer.write(vis);
}

cv::Point2f CameraOpticalFlow::getOpticalFlow() const
{
    return m_opticalFlow;
}