#ifndef DRONE_H
#define DRONE_H

#include <array>
#include <cstdint>
#include <opencv2/opencv.hpp>

#include "msp/msp.hpp"
#include "posHold/Drone.h"
#include "video/RpiVideo.hpp"

class DroneRpi public Drone
{
public:
    DroneRpi();
    DroneRpi(msp::Msp& msp) :


    [[nodiscard]] cv::Mat getGrayscaleImage();
    video::RpiVideo& getCameraRpi() { return m_cameraRpi; }

private:
    video::RpiVideo m_cameraRpi;
};

#endif
