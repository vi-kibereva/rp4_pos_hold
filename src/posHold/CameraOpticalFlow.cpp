#include <opencv2/opencv.hpp>

#include "posHold/CameraOpticalFlow.hpp"

cv::VideoWriter writer = cv::VideoWriter(
    "output.mp4",
    cv::VideoWriter::fourcc('M','P','4','V'),
    30.0,
    cv::Size(1920, 1080)
);

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

    m_prevFrame = grayFrame.clone();

    // --------------------------------------------
    // VISUALIZATION + SAVE TO VIDEO
    // --------------------------------------------
    cv::Mat vis;
    cv::cvtColor(grayFrame, vis, cv::COLOR_GRAY2BGR);

    // Draw arrows
    const int step = 10;       // draw each 10 pixels
    const double scale = 3.0;  // arrow size

    for (int yy = roi.y; yy < roi.y + roi.height; yy += step)
    {
        for (int xx = roi.x; xx < roi.x + roi.width; xx += step)
        {
            cv::Point2f f = m_opticalFlow.at<cv::Point2f>(yy, xx);

            cv::Point p0(xx, yy);
            cv::Point p1(xx + (int)(f.x * scale), yy + (int)(f.y * scale));

            cv::arrowedLine(vis, p0, p1, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
        }
    }

    // Save this frame
    writer.write(vis);
}

cv::Point2f CameraOpticalFlow::getOpticalFlowAt(const int x, const int y) const
{
    if (m_opticalFlow.empty())
    {
        throw std::runtime_error("CameraOpticalFlow::getOpticalFlowAt called before calling CameraOpticalFlow::calc");
    }
    return m_opticalFlow.at<cv::Point2f>(y, x);
}
