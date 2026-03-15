#pragma once

// #include <driver.hpp>
#include <predictor/predictor.hpp>
#include <utils/utils.h>

namespace CONTROLLER
{
    struct BoardInformation
    {
        double aim_yaw;
        double aim_pitch;
        double aim_distance;
        double face_cos;
    };

    typedef std::vector<BoardInformation> BoardInformations;

} // namespace CONTROLLER