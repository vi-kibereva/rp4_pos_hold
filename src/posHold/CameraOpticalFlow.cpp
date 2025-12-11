#include <opencv2/opencv.hpp>
#include "posHold/CameraOpticalFlow.hpp"
#include <iostream>
#include <fstream>
#include <vector>

/*
// Global video writer
cv::VideoWriter g_writer;
std::ofstream text_writer;

std::vector<cv::Mat> videoData{};
*/

cv::Mat grayFrame = cv::Mat{};

// Constructor
CameraOpticalFlow::CameraOpticalFlow(Drone& drone)
    : m_drone{ &drone }, m_opticalFlow{0.0f, 0.0f}
{
    // videoData.reserve(300);
}

// Main calculation function
void CameraOpticalFlow::calc(int x, int y, int len)
{
    grayFrame = m_drone->getGrayscaleImage();

    /*
    // Initialize writer if not already
    if (!g_writer.isOpened())
    {
        g_writer.open("flow_output.avi",
                      cv::VideoWriter::fourcc('m', 'p', '4', 'v'),
                      10,
                      grayFrame.size());
    }
    if (!text_writer.is_open())
    {
        text_writer.open("flow_txt_data.txt");
    }
    */

    // Define ROI for feature detection
    int x0 = std::max(x - len, 0);
    int y0 = std::max(y - len, 0);
    int x1 = std::min(x + len, grayFrame.cols - 1);
    int y1 = std::min(y + len, grayFrame.rows - 1);
    cv::Rect roi(x0, y0, x1 - x0 + 1, y1 - y0 + 1);

    // First frame: detect initial features
    if (m_prevFrame.empty())
    {
        m_prevFrame = grayFrame.clone();

        cv::goodFeaturesToTrack(grayFrame(roi), m_prevPoints, 200, 0.01, 5);

        // Shift points to full frame coordinates
        for (auto &p : m_prevPoints) { p.x += roi.x; p.y += roi.y; }

        m_opticalFlow = cv::Point2f(0, 0);
        return;
    }

    // Re-detect points if too few
    const int minPoints = 50;
    if (m_prevPoints.size() < minPoints)
    {
        //std::cout << "Redetecting features\n";
        std::vector<cv::Point2f> newPoints;
        cv::goodFeaturesToTrack(grayFrame(roi), newPoints, 200, 0.01, 5);
        for (auto &p : newPoints) { p.x += roi.x; p.y += roi.y; }
        m_prevPoints.insert(m_prevPoints.end(), newPoints.begin(), newPoints.end());
    }

    if (m_prevPoints.empty()) return;

    // Lucas-Kanade optical flow
    std::vector<cv::Point2f> nextPoints;
    std::vector<uchar> status;
    std::vector<float> err;

    cv::calcOpticalFlowPyrLK(
        m_prevFrame, grayFrame,
        m_prevPoints, nextPoints,
        status, err
    );

    m_prevFrame = grayFrame.clone();

    // Compute mean flow and keep good points
    cv::Point2f flowSum(0, 0);
    std::vector<cv::Point2f> goodNextPoints;
    std::vector<cv::Point2f> goodPrevPoints;

    for (size_t i = 0; i < nextPoints.size(); ++i)
    {
        if (!status[i] || nextPoints[i].x < x0 || nextPoints[i].x > x1
            || nextPoints[i].y < y0 || nextPoints[i].y > y1)
        {
            continue;
        }

        cv::Point2f f = nextPoints[i] - m_prevPoints[i];
        flowSum += f;
        goodNextPoints.push_back(nextPoints[i]);
        goodPrevPoints.push_back(m_prevPoints[i]);
    }

    if (!goodNextPoints.empty())
    {
        cv::Point2f meanFlow = flowSum * (1.0f / goodNextPoints.size());
        // Apply low-pass filter
        m_opticalFlow = s_alpha * meanFlow + (1.0f - s_alpha) * m_opticalFlow;
    }
    else
    {
        m_opticalFlow = cv::Point2f(0, 0);
    }

    m_prevPoints = goodNextPoints;

    /*
    // Add optical flow arrows

    cv::cvtColor(grayFrame, grayFrame, cv::COLOR_GRAY2BGR);

    double scale = 0.5;  // scale for flow arrows

    for (size_t i = 0; i < goodPrevPoints.size(); ++i)
    {
        cv::Point p0(cvRound(goodPrevPoints[i].x), cvRound(goodPrevPoints[i].y));
        cv::Point p1(
            cvRound(goodPrevPoints[i].x + (goodNextPoints[i].x - goodPrevPoints[i].x) * scale),
            cvRound(goodPrevPoints[i].y + (goodNextPoints[i].y - goodPrevPoints[i].y) * scale)
        );
        cv::arrowedLine(grayFrame, p0, p1, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
    }
    */

    /*
    // Visualization
    cv::Mat vis;
    cv::cvtColor(grayFrame, vis, cv::COLOR_GRAY2BGR);

    double scale = 0.5;  // scale for flow arrows

    for (size_t i = 0; i < goodPrevPoints.size(); ++i)
    {
        cv::Point p0(cvRound(goodPrevPoints[i].x), cvRound(goodPrevPoints[i].y));
        cv::Point p1(
            cvRound(goodPrevPoints[i].x + (goodNextPoints[i].x - goodPrevPoints[i].x) * scale),
            cvRound(goodPrevPoints[i].y + (goodNextPoints[i].y - goodPrevPoints[i].y) * scale)
        );
        cv::arrowedLine(vis, p0, p1, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
    }

    // Draw mean flow arrow in corner
    cv::Point corner(100, 100);
    cv::Point cornerTo(
        corner.x + cvRound(m_opticalFlow.x),
        corner.y + cvRound(m_opticalFlow.y)
    );
    cv::arrowedLine(vis, corner, cornerTo, cv::Scalar(0, 255, 0), 3, cv::LINE_AA);

    // Write to video and text
    if (g_writer.isOpened())
    {
        videoData.push_back(vis);
    }
    if (text_writer.is_open())
    {
        text_writer << "x: " << m_opticalFlow.x << ", " << "y: " << m_opticalFlow.y << '\n';
    }
    */
}

// Return current optical flow
cv::Point2f CameraOpticalFlow::getOpticalFlow() const
{
    return m_opticalFlow;
}

CameraOpticalFlow::~CameraOpticalFlow()
{
    /*
    // To save video
    for (const cv::Mat& mat : videoData)
    {
        g_writer.write(mat);
    }
    */
}
