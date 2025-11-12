#ifndef CAMERAOPTICALFLOW_H
#define CAMERAOPTICALFLOW_H

#include <opencv2/opencv.hpp>
#include "posHold/Drone.h"

class CameraOpticalFlow
{
public:
    explicit CameraOpticalFlow(Drone& drone);

    void calc();

    [[nodiscard]] cv::Point2f getOpticalFlowAt(int x, int y, int len) const;

private:
    Drone* m_drone;
    cv::Mat m_prevFrame;
    cv::Mat m_opticalFlow;
};

#endif
