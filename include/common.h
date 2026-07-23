#pragma once
#include <random>
#include <chrono>
#include <string>

namespace common {

    extern std::mt19937 rng;

    double rand_uniform();
    double rand_normal();
    double rand_range(double lo, double hi);

    double clamp(double v, double lo, double hi);
    double wrap_angle(double theta);
    double angle_diff(double a, double b);

    void ensure_dir(const std::string& path);

    struct Timer {
        using clock = std::chrono::high_resolution_clock;
        std::chrono::time_point<clock> start;
        Timer();
        double ms() const;
        double s() const;
    };

} // namespace common
