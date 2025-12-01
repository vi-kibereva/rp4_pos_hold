#ifndef CAMERAOPTICALFLOW_H
#define CAMERAOPTICALFLOW_H

#include <opencv2/opencv.hpp>
#include "posHold/Drone.hpp"

class CameraOpticalFlow
{
public:
    explicit CameraOpticalFlow(Drone& drone);

    void calc(int x, int y, int len);

    [[nodiscard]] cv::Point2f getOpticalFlow() const;

private:
    Drone* m_drone;
    cv::Mat m_prevGray;
    std::vector<cv::Point2f> m_prevPoints;
    cv::Point2f m_opticalFlow;
};

#endif
