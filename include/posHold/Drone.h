#ifndef DRONE_H
#define DRONE_H

#include <array>
#include <cstdint>
#include <opencv2/opencv.hpp>

#include "msp/msp.hpp"

class Drone
{
public:
    struct CameraInfo
    {
        CameraInfo(
            const double fov,
            const int resolutionX,
            const int resolutionY,
            const double minDist,
            const double maxDist) :
            fov{ fov },
            resolutionX{ resolutionX },
            resolutionY{ resolutionY },
            minDist{ minDist },
            maxDist{ maxDist },
            focalLength{ resolutionX / (std::tan(fov / 2) * 2) }
        {
        }

        const double fov;
        const int resolutionX;
        const int resolutionY;
        const double minDist;
        const double maxDist;
        const double focalLength;
    };

    struct GyroData
    {
        double roll;
        double pitch;
        double yaw;
    };

    const CameraInfo cameraInfo = CameraInfo(
        CV_PI / 2,
        512,
        512,
        0.01,
        1000.0
    );

    explicit Drone(msp::Msp& m_msp);

    [[nodiscard]] cv::Mat getGrayscaleImage();

    [[nodiscard]] GyroData getGyroData();

    [[nodiscard]] double getAltitude();

private:
    msp::Msp* m_msp;
    cv::VideoCapture m_camera;
};

#endif
