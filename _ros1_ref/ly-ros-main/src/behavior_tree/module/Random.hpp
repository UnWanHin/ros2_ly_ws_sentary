#pragma once

#include <random>

namespace LangYa {
    class Random  {
    public:
        Random() {
            rng = std::mt19937(static_cast<unsigned int>(rd()));
        }
        int Get(int min, int max){
            std::uniform_int_distribution<int> dist(min, max);
            return dist(rng);
        }

    private:
        std::random_device rd;
        std::mt19937 rng;
    };
}