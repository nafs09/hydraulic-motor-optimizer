
#include "common.h"
#include "config.h"
#include <cmath>
#include <filesystem>
#include <iostream>
#include <random>
#include <chrono>

namespace common {

    static unsigned int make_runtime_seed() {
        if constexpr (config::RNG_SEED != 0u) {
            return config::RNG_SEED;
        }
        else {
            std::random_device rd;
            if (rd.entropy() > 0.0) {
                return rd();
            }
            return static_cast<unsigned int>(
                std::chrono::high_resolution_clock::now().time_since_epoch().count()
                );
        }
    }

    std::mt19937 rng(make_runtime_seed());

    double rand_uniform() {
        return std::uniform_real_distribution<double>(0.0, 1.0)(rng);
    }
    double rand_normal() {
        return std::normal_distribution<double>(0.0, 1.0)(rng);
    }
    double rand_range(double lo, double hi) {
        return std::uniform_real_distribution<double>(lo, hi)(rng);
    }

    double clamp(double v, double lo, double hi) {
        if (v < lo) return lo;
        if (v > hi) return hi;
        return v;
    }

    double wrap_angle(double theta) {
        const double two_pi = 2.0 * M_PI;
        theta = fmod(theta, two_pi);
        if (theta < 0) theta += two_pi;
        return theta;
    }

    double angle_diff(double a, double b) {
        double d = wrap_angle(a) - wrap_angle(b);
        if (d > M_PI) d -= 2.0 * M_PI;
        if (d < -M_PI) d += 2.0 * M_PI;
        return d;
    }

    void ensure_dir(const std::string& path) {
        std::filesystem::create_directories(path);
    }

    Timer::Timer() : start(clock::now()) {}
    double Timer::ms() const {
        return std::chrono::duration<double, std::milli>(clock::now() - start).count();
    }
    double Timer::s() const {
        return std::chrono::duration<double>(clock::now() - start).count();
    }

} // namespace common
