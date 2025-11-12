#ifndef VECMOVE_H
#define VECMOVE_H

#include "posHold/Drone.h"
#include "posHold/VecDown.h"
#include "posHold/CameraOpticalFlow.h"

class VecMove
{
public:
    explicit VecMove(Drone& drone);

    void calc();

    [[nodiscard]] cv::Point2f getVecMove() const;

private:
    static constexpr int s_accountFlowPixels = 10;
    static constexpr double s_noFlowBalanceVecMultiplier = 1.0f;
    Drone* m_drone;
    VecDown m_vecDown;
    CameraOpticalFlow m_cameraOpticalFlow;
    cv::Point2f m_vecMove;
    bool m_hasPrev = false;
};

#endif
