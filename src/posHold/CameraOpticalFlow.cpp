#include "posHold/CameraOpticalFlow.hpp"
#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>

// Global video writer
cv::VideoWriter g_writer;

CameraOpticalFlow::CameraOpticalFlow(Drone& drone)
    : m_drone(&drone), m_opticalFlow(0, 0)
{
}

void CameraOpticalFlow::calc(int x, int y, int len)
{
    cv::Mat frame;
    m_drone->getCameraFrame(frame); // Assuming this fills frame (CV_8UC3)

    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    if (m_prevFrame.empty()) {
        gray.copyTo(m_prevFrame);

        // Initialize video writer if not already
        if (!g_writer.isOpened()) {
            g_writer.open("flow_output.avi",
                          cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
                          30, frame.size());
        }
        return;
    }

    // Detect good features to track in previous frame
    std::vector<cv::Point2f> prevPoints;
    const int maxPoints = 200;
    cv::goodFeaturesToTrack(m_prevFrame, prevPoints, maxPoints, 0.01, 5);

    if (prevPoints.empty()) {
        gray.copyTo(m_prevFrame);
        return;
    }

    // Calculate optical flow using LK
    std::vector<cv::Point2f> nextPoints;
    std::vector<uchar> status;
    std::vector<float> err;
    cv::calcOpticalFlowPyrLK(m_prevFrame, gray, prevPoints, nextPoints, status, err);

    // Compute mean flow
    cv::Point2f flowSum(0, 0);
    int count = 0;
    for (size_t i = 0; i < prevPoints.size(); ++i) {
        if (status[i]) {
            cv::Point2f delta = nextPoints[i] - prevPoints[i];
            flowSum += delta;
            ++count;
        }
    }

    if (count > 0) {
        m_opticalFlow = flowSum * (1.0f / count);
    } else {
        m_opticalFlow = cv::Point2f(0, 0);
    }

    // Visualization
    cv::Mat vis;
    cv::cvtColor(gray, vis, cv::COLOR_GRAY2BGR);

    for (size_t i = 0; i < prevPoints.size(); ++i) {
        if (!status[i]) continue;
        cv::Point2f delta = (nextPoints[i] - prevPoints[i]) * 5.0f; // scale for visibility
        cv::Point p0(cvRound(prevPoints[i].x), cvRound(prevPoints[i].y));
        cv::Point p1(cvRound(prevPoints[i].x + delta.x), cvRound(prevPoints[i].y + delta.y));
        cv::arrowedLine(vis, p0, p1, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
    }

    // Draw mean flow
    cv::Point pCenter(frame.cols / 2, frame.rows / 2);
    cv::Point pMean(pCenter.x + cvRound(m_opticalFlow.x * 10),
                    pCenter.y + cvRound(m_opticalFlow.y * 10));
    cv::arrowedLine(vis, pCenter, pMean, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);

    // Write video frame
    if (g_writer.isOpened()) {
        g_writer.write(vis);
    }

    gray.copyTo(m_prevFrame);
}

cv::Point2f CameraOpticalFlow::getOpticalFlow() const
{
    return m_opticalFlow;
}
