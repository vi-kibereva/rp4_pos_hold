#include <cmath>

#include "RemoteAPIClient.h"

#include "Drone.h"

Drone::Drone(RemoteAPIObject::sim& sim) :
    m_sim{ &sim },
    m_drone{ sim.getObject("/Quadcopter/base/target") },
    m_respondables{
        sim.getObject("/Quadcopter/propeller[0]/respondable"),
        sim.getObject("/Quadcopter/propeller[1]/respondable"),
        sim.getObject("/Quadcopter/propeller[2]/respondable"),
        sim.getObject("/Quadcopter/propeller[3]/respondable")
    },
    m_visionSensor{ sim.getObject("/Quadcopter/visionSensor") },
    m_gyroSensorScript{ sim.getScript(sim.scripttype_childscript, "/Quadcopter/gyroSensor/Script") }
{
    std::vector<std::int64_t> cameraFrameSize = std::get<1>(m_sim->getVisionSensorImg(m_visionSensor));
    m_cameraFrameSize = { cameraFrameSize[0], cameraFrameSize[1] };
}

[[nodiscard]] cv::Mat Drone::getGrayscaleImage() const
{
    std::vector<std::uint8_t> imgBytes = std::get<0>(m_sim->getVisionSensorImg(m_visionSensor));
    cv::Mat frame(
        m_cameraFrameSize.second,
        m_cameraFrameSize.first,
        CV_8UC3, imgBytes.data());
    cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
    cv::flip(frame, frame, 0);
    return frame;
}

[[nodiscard]] std::vector<double> Drone::getGyroData() const
{
    // gyroData[0] - absolute rotation angle (not velocity) around horizontal forward-backward world axis (roll)
    // gyroData[1] - absolute rotation angle (not velocity) around left-right world axis (pitch)
    // gyroData[2] - absolute rotation angle (not velocity) around vertical world axis (yaw)

    json data = m_sim->callScriptFunction("getGyroData", m_gyroSensorScript)[0];

    if (!data.is_array())
    {
        return { 0.0, 0.0, 0.0 };
    }

    return {
        data[0].as<double>(),
        data[1].as<double>(),
        data[2].as<double>()
    };
}

[[nodiscard]] double Drone::getAltitude() const
{
    return m_sim->getObjectPosition(m_drone)[2];
}

void Drone::setAngularVelocities(const std::array<double, s_propellersCount>& angularVelocities)
{
    m_angularVelocities = angularVelocities;
}

void Drone::update()
{
    for (std::uint64_t i = 0; i < s_propellersCount; ++i)
    {
        // Compute thrust and torque magnitudes
        const double thrust = kf * m_angularVelocities[i] * m_angularVelocities[i];
        const double torqueMag = km * m_angularVelocities[i] * m_angularVelocities[i] * m_propellerDirections[i];

        // Get propeller orientation in world frame
        std::vector<double> angles = m_sim->getObjectOrientation(m_respondables[i]);

        // Compute thrust direction in world coordinates
        std::vector<double> thrustVec = rotateForce(angles, thrust);

        // Compute torque vector (reaction around Z)
        std::vector<double> torqueVec = rotateForce(angles, torqueMag);

        // Apply both
        m_sim->addForceAndTorque(m_respondables[i], thrustVec, torqueVec);
    }
}

std::vector<double> Drone::rotateForce(const std::vector<double>& angles, const double thrust)
{
    const double cx = std::cos(angles[0]);
    const double sx = std::sin(angles[0]);
    const double cy = std::cos(angles[1]);
    const double sy = std::sin(angles[1]);
    const double cz = std::cos(angles[2]);
    const double sz = std::sin(angles[2]);

    double R[3][3];
    R[0][0] = cz*cy;             R[0][1] = cz*sy*sx - sz*cx;   R[0][2] = cz*sy*cx + sz*sx;
    R[1][0] = sz*cy;             R[1][1] = sz*sy*sx + cz*cx;   R[1][2] = sz*sy*cx - cz*sx;
    R[2][0] = -sy;               R[2][1] = cy*sx;              R[2][2] = cy*cx;

    return {
        R[0][2] * thrust,
        R[1][2] * thrust,
        R[2][2] * thrust
    };
}
