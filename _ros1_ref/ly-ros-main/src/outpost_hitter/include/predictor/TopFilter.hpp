#pragma once
#include "solver/solver.hpp"

namespace PREDICTOR
{
    class TopFilter
    {
    public:
        TopFilter();
        ~TopFilter();

        SOLVER::ArmorPoses filterTopArmor(const SOLVER::ArmorPoses &armor_poses);

    private:
    };
} // namespace PREDICTOR