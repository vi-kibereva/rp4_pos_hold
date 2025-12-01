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

    // -------------------------------
    // Sparse points grid
    // -------------------------------
    std::vector<cv::Point2f> prevPts;
    const int step = 10;
    for (int yy = roi.y; yy < roi.y + roi.height; yy += step)
        for (int xx = roi.x; xx < roi.x + roi.width; xx += step)
            prevPts.emplace_back(float(xx), float(yy));

    std::vector<cv::Point2f> nextPts;
    std::vector<uchar> status;
    std::vector<float> err;

    cv::calcOpticalFlowPyrLK(
        m_prevFrame, grayFrame, prevPts, nextPts, status, err,
        cv::Size(21,21), 3
    );

    // -------------------------------
    // Interpolate sparse points into dense map
    // -------------------------------
    cv::Mat denseFlow(grayFrame.size(), CV_32FC2, cv::Scalar(0,0));

    for (size_t i = 0; i < prevPts.size(); ++i)
    {
        if (!status[i]) continue;

        int px = cvRound(prevPts[i].x);
        int py = cvRound(prevPts[i].y);
        cv::Point2f f = nextPts[i] - prevPts[i];

        denseFlow.at<cv::Point2f>(py, px) = f;
    }

    // Optional: smooth/interpolate the flow map
    cv::GaussianBlur(denseFlow, m_opticalFlow, cv::Size(21,21), 0);

    m_prevFrame = grayFrame.clone();

    // -------------------------------
    // Visualization + save
    // -------------------------------
    cv::Mat vis;
    cv::cvtColor(grayFrame, vis, cv::COLOR_GRAY2BGR);

    const double arrowScale = 3.0;

    for (int yy = roi.y; yy < roi.y + roi.height; yy += step)
    {
        for (int xx = roi.x; xx < roi.x + roi.width; xx += step)
        {
            cv::Point2f f = m_opticalFlow.at<cv::Point2f>(yy, xx);
            cv::arrowedLine(vis,
                cv::Point(xx, yy),
                cv::Point(xx + int(f.x * arrowScale), yy + int(f.y * arrowScale)),
                cv::Scalar(0,0,255), 1, cv::LINE_AA
            );
        }
    }

    // Mean flow arrow
    cv::Scalar meanFlow = cv::mean(m_opticalFlow(roi));
    cv::Point2f avg(meanFlow[0], meanFlow[1]);
    double bigScale = 20.0;
    cv::Point corner(50, 50);
    cv::arrowedLine(vis, corner,
        cv::Point(corner.x + int(avg.x * bigScale), corner.y + int(avg.y * bigScale)),
        cv::Scalar(0,255,0), 3, cv::LINE_AA
    );

    char buf[100];
    snprintf(buf, sizeof(buf), "Avg flow: (%.2f, %.2f)", avg.x, avg.y);
    cv::putText(vis, buf, cv::Point(50, 90), cv::FONT_HERSHEY_SIMPLEX,
                0.7, cv::Scalar(0,255,0), 2, cv::LINE_AA);

    writer.write(vis);
}

cv::Point2f CameraOpticalFlow::getOpticalFlowAt(const int x, const int y) const
{
    if (m_opticalFlow.empty())
        throw std::runtime_error("getOpticalFlowAt called before calc");
    return m_opticalFlow.at<cv::Point2f>(y, x);
}
