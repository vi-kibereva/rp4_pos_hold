#ifndef DRONE_H
#define DRONE_H

#include <array>
#include <cstdint>
#include <memory>
#include <opencv2/opencv.hpp>

#include "contracts/IMsp.hpp"
#include "contracts/IVideo.hpp"

class Drone {
public:
  struct CameraInfo {
    CameraInfo(const double fov, const int resolutionX, const int resolutionY,
               const double minDist, const double maxDist, const int fps)
        : fov{fov}, resolutionX{resolutionX}, resolutionY{resolutionY},
          minDist{minDist}, maxDist{maxDist},
          focalLength{resolutionX / (std::tan(fov / 2) * 2)}, fps{fps} {}

    const double fov;
    const int resolutionX;
    const int resolutionY;
    const double minDist;
    const double maxDist;
    const double focalLength;
    const int fps;
  };

  struct GyroData {
    double roll;
    double pitch;
    double yaw;
  };

  const CameraInfo cameraInfo =
      CameraInfo(37.4 * CV_PI / 180, 1920, 1080, 0.01, 1000.0, 10);

  Drone();

  explicit Drone(contracts::IMsp &m_msp, contracts::IVideo &video);

  [[nodiscard]] cv::Mat getGrayscaleImage();

  [[nodiscard]] GyroData getGyroData();

  [[nodiscard]] double getAltitude();

  ~Drone();

private:
  contracts::IMsp *m_msp;
  contracts::IVideo *m_camera;
};

#endif
