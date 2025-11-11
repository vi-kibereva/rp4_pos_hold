#ifndef DRONE_H
#define DRONE_H

#include <array>
#include <cstdint>
#include <opencv2/opencv.hpp>

#include "RemoteAPIClient.h"

class Drone
{
public:
    struct CameraInfo
    {
        double fov;
        int resolutionX;
        int resolutionY;
        double minDist;
        double maxDist;
    };

    constexpr static std::uint64_t s_propellersCount = 4;

    const CameraInfo cameraInfo{
        CV_PI / 2,
        512,
        512,
        0.01,
        1000.0
    };

    const double kf = 3e-6;
    const double km = 3e-7;

    explicit Drone(RemoteAPIObject::sim& sim);

    [[nodiscard]] cv::Mat getGrayscaleImage() const;

    [[nodiscard]] std::vector<double> getGyroData() const;

    [[nodiscard]] double getAltitude() const;

    void setAngularVelocities(const std::array<double, s_propellersCount>& angularVelocities);

    void update();

private:
    static std::vector<double> rotateForce(const std::vector<double>& angles, double thrust);

    RemoteAPIObject::sim* m_sim;
    std::int64_t m_drone;
    std::array<std::int64_t, s_propellersCount> m_respondables;
    std::int64_t m_visionSensor;
    std::int64_t m_gyroSensorScript;
    std::pair<int, int> m_cameraFrameSize;

    std::array<double, s_propellersCount> m_angularVelocities{};

    const std::array<std::int64_t, s_propellersCount> m_propellerDirections{ 1, -1, 1, -1 };
};

#endif
