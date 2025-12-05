#ifndef DRONE_H
#define DRONE_H

#include <array>
#include <cstdint>
#include <opencv2/opencv.hpp>

#include "msp/msp.hpp"
#include "video/RpiVideo.hpp"

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
            const double maxDist,
            const int fps) :
            fov{ fov },
            resolutionX{ resolutionX },
            resolutionY{ resolutionY },
            minDist{ minDist },
            maxDist{ maxDist },
            focalLength{ resolutionX / (std::tan(fov / 2) * 2) },
            fps{ fps }
        {
        }

        const double fov;
        const int resolutionX;
        const int resolutionY;
        const double minDist;
        const double maxDist;
        const double focalLength;
        const int fps;
    };

    struct GyroData
    {
        double roll;
        double pitch;
        double yaw;
    };

    const CameraInfo cameraInfo = CameraInfo(
        60 * CV_PI / 180,
        1920,
        1080,
        0.01,
        1000.0,
        10
    );
    
    Drone();

    explicit Drone(msp::Msp& m_msp);

    [[nodiscard]] cv::Mat getGrayscaleImage();

    [[nodiscard]] GyroData getGyroData();

    [[nodiscard]] double getAltitude();

    ~Drone();

private:
    msp::Msp* m_msp;
    video::RpiVideo m_camera;
};

#endif
