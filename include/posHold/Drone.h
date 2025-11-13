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
        60 * CV_PI / 180,
        1280,
        720,
        0.01,
        1000.0
    );

    Drone();

    Drone(std::string port);

    explicit Drone(msp::Msp& m_msp, std::string);

    [[nodiscard]] cv::Mat getGrayscaleImage();

    [[nodiscard]] GyroData getGyroData();

    [[nodiscard]] double getAltitude();

    cv::VideoCapture& getCamera() { return m_camera; }

private:
    msp::Msp* m_msp;
    cv::VideoCapture m_camera;
};

#endif
