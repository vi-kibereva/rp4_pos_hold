#include "posHold/OfflineDroneAdapter.hpp"
#include <stdexcept>

// Forward declarations from analyze_main.cpp
struct AltitudeRecord {
    double timestamp;
    double altitude;
    double vario;
};

struct AttitudeRecord {
    double timestamp;
    double roll;
    double pitch;
    double yaw;
};

class CsvDataLoader;

// We need to declare these methods that will be defined in analyze_main.cpp
extern AltitudeRecord interpolateAltitude(CsvDataLoader* loader, double timestamp);
extern AttitudeRecord interpolateAttitude(CsvDataLoader* loader, double timestamp);

OfflineDroneAdapter::OfflineDroneAdapter(CsvDataLoader& csv_loader) :
    m_csv_loader(&csv_loader),
    m_current_timestamp(0.0)
{
}

void OfflineDroneAdapter::setFrame(const cv::Mat& frame, double timestamp)
{
    m_current_frame = frame.clone();
    m_current_timestamp = timestamp;

    // Pre-compute grayscale
    if (!m_current_frame.empty()) {
        cv::cvtColor(m_current_frame, m_current_gray, cv::COLOR_BGR2GRAY);
    }
}

cv::Mat OfflineDroneAdapter::getGrayscaleImage()
{
    if (m_current_gray.empty()) {
        throw std::runtime_error("No frame set in OfflineDroneAdapter");
    }
    return m_current_gray;
}

OfflineDroneAdapter::GyroData OfflineDroneAdapter::getGyroData()
{
    AttitudeRecord attitude = interpolateAttitude(m_csv_loader, m_current_timestamp);

    // CSV has degrees, convert to radians
    return {
        attitude.roll * CV_PI / 180.0,
        attitude.pitch * CV_PI / 180.0,
        attitude.yaw * CV_PI / 180.0
    };
}

double OfflineDroneAdapter::getAltitude()
{
    AltitudeRecord altitude = interpolateAltitude(m_csv_loader, m_current_timestamp);

    // CSV has cm, convert to meters (matching Drone.cpp:39)
    return altitude.altitude / 100.0;
}
