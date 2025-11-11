#include <iostream>
#include <windows.h>

#include "RemoteAPIClient.h"

#include "Drone.h"
#include "VecMove.h"

void showOpticalFlow(const cv::Mat& grayFrame,
                     const VecMove& vecMove,
                     const std::pair<int, int>& frameSize,
                     const int step = 8,
                     const float resizeFactor = 1.5f)
{
    cv::Mat display;
    cv::cvtColor(grayFrame, display, cv::COLOR_GRAY2BGR);
    cv::resize(display, display, cv::Size(), resizeFactor, resizeFactor, cv::INTER_LINEAR);

    cv::arrowedLine(
        display,
        { (int)(frameSize.first / 2 * resizeFactor), (int)(frameSize.second / 2 * resizeFactor) },
        { (int)((frameSize.first / 2 + vecMove.getVecMove().x) * resizeFactor), (int)((frameSize.second / 2 + vecMove.getVecMove().y) * resizeFactor) },
        cv::Scalar(255, 0, 0),
        2,
        cv::LINE_AA,
        0,
        0.3
    );

    cv::imshow("Bottom camera", display);
    cv::waitKey(1);
}

int main(int argc, char* argv[])
{
    RemoteAPIClient client;
    RemoteAPIObject::sim sim = client.getObject().sim();

    Drone drone(sim);
    sim.setStepping(true);
    sim.startSimulation();

    VecMove vecMove(drone);

    double t = 0.0;

    while (true)
    {
        t = sim.getSimulationTime();

        auto imgData = drone.getGrayscaleImage();
        if (!imgData.empty())
        {;
            vecMove.calc();
            showOpticalFlow(imgData, vecMove, { drone.cameraInfo.resolutionX, drone.cameraInfo.resolutionY }, 16, 1.5);
        }

        if (cv::waitKey(10) == 27)
        {
            break;
        }

        if (GetAsyncKeyState(VK_UP))
        {
            drone.setAngularVelocities({ 2000.0, 2000.0, 2000.0, 2000.0 });
        }
        else if (GetAsyncKeyState(VK_LEFT))
        {
            drone.setAngularVelocities({ 2000.0, 2000.0, 1900.0, 1900.0 });
        }
        else if (GetAsyncKeyState(VK_RIGHT))
        {
            drone.setAngularVelocities({ 1900.0, 1900.0, 2000.0, 2000.0 });
        }
        else
        {
            drone.setAngularVelocities({ 0.0, 0.0, 0.0, 0.0 });
        }

        drone.update();

        sim.step();
    }

    sim.stopSimulation();

    return 0;
}
