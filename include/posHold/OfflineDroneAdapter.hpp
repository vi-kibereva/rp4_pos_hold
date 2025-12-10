#ifndef OFFLINE_DRONE_ADAPTER_HPP
#define OFFLINE_DRONE_ADAPTER_HPP

#include <opencv2/opencv.hpp>
#include <cmath>

// Forward declarations
struct AltitudeRecord;
struct AttitudeRecord;
class CsvDataLoader;

class OfflineDroneAdapter
{
public:
    // CameraInfo structure (replicated from Drone.hpp to avoid dependency)
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

    // GyroData structure (replicated from Drone.hpp to avoid dependency)
    struct GyroData
    {
        double roll;
        double pitch;
        double yaw;
    };

    // Camera info with corrected FOV (37.4 degrees instead of 60)
    const CameraInfo cameraInfo = CameraInfo(
        37.4 * CV_PI / 180,  // 37.4 degree FOV (actual camera)
        1920,                // width
        1080,                // height
        0.01,                // min distance
        1000.0,              // max distance
        15                   // fps (video frame rate)
    );

    explicit OfflineDroneAdapter(CsvDataLoader& csv_loader);

    // Update current frame and timestamp
    void setFrame(const cv::Mat& frame, double timestamp);

    // Drone interface methods
    [[nodiscard]] cv::Mat getGrayscaleImage();
    [[nodiscard]] GyroData getGyroData();
    [[nodiscard]] double getAltitude();

private:
    CsvDataLoader* m_csv_loader;
    cv::Mat m_current_frame;
    cv::Mat m_current_gray;
    double m_current_timestamp;
};

#endif
