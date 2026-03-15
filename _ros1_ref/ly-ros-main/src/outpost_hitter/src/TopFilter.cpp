#include "predictor/TopFilter.hpp"

using namespace SOLVER;
namespace PREDICTOR
{
    TopFilter::TopFilter()
    {
    }

    TopFilter::~TopFilter()
    {
    }

    ArmorPoses TopFilter::filterTopArmor(const ArmorPoses &armor_poses)
    {
        return armor_poses;
    }
} // namespace PREDICTOR