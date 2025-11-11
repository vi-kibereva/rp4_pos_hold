#ifndef VECDOWN_H
#define VECDOWN_H

#include "Drone.h"

class VecDown
{
public:
    explicit VecDown(const Drone& drone);

    void calc();

    [[nodiscard]] cv::Point2f getVecDown() const;

    [[nodiscard]] cv::Point2f getVecDownDisplacement() const;

private:
  	[[nodiscard]] cv::Vec3d calcVecDown3d() const;

    [[nodiscard]] cv::Point2f calcVecDownProjection() const;

    const Drone* m_drone;
    cv::Point2f m_vecDown;
    cv::Point2f m_vecDownDisplacement;
    bool m_hasPrev = false;
};

#endif
