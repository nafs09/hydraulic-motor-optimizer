#include "constraints.h"
#include "config.h"
#include <cmath>
#include <cstdio>
#include <algorithm>

namespace constraints {

    static inline double clamp_double(double v, double lo, double hi) {
        if (v < lo) return lo;
        if (v > hi) return hi;
        return v;
    }

    void project(std::vector<double>& x) {
        const int expected = config::N_CTRL + 3;
        if ((int)x.size() != expected) {
            std::fprintf(stderr,
                "constraints::project: expected %d elements, got %d. Skipping.\n",
                expected, (int)x.size());
            return;
        }

        // Cam parameters are now Fourier sine-power coefficients.
        // cam_utils applies relu() so negatives are clamped to zero anyway,
        // but projecting to [0, STROKE] here keeps the optimizer in a bounded
        // search region and avoids very large coefficients that would be
        // scaled away by the peak-normalisation step in cam_utils, wasting
        // gradient budget on parameters that don't move the profile.
        for (int i = 0; i < config::N_CTRL; ++i)
            x[i] = clamp_double(x[i], 0.0, config::STROKE);

        // valve_shift ∈ [-π, π]: symmetric range keeps optimum in the interior.
        // (Clamping to [0, 2π] previously trapped the optimizer at the 2π
        //  boundary where wrap_angle discontinuity killed finite-difference grads.)
        x[config::N_CTRL + 0] = clamp_double(x[config::N_CTRL + 0], -M_PI, M_PI);

        // valve_width ∈ [0.1, 0.9]: fraction of lobe cycle that supply port is open.
        x[config::N_CTRL + 1] = clamp_double(x[config::N_CTRL + 1], 0.1, 0.9);

        // valve_overlap ∈ [0, 0.2]: transition arc as fraction of lobe cycle.
        x[config::N_CTRL + 2] = clamp_double(x[config::N_CTRL + 2], 0.0, 0.2);
    }

    double penalty(const std::vector<double>& x) {
        const int expected = config::N_CTRL + 3;
        if ((int)x.size() != expected) {
            std::fprintf(stderr,
                "constraints::penalty: expected %d elements, got %d. Returning 1e6.\n",
                expected, (int)x.size());
            return 1e6;
        }
        // Smoothness penalty on cam Fourier coefficients (prefer smooth profiles).
        // With the Fourier parameterisation this also discourages sharp lobes
        // (high harmonic content) that produce high ripple.
        double s = 0.0;
        for (int i = 1; i < config::N_CTRL; ++i) {
            double diff = x[i] - x[i - 1];
            s += diff * diff;
        }
        return s / (double)std::max(1, config::N_CTRL - 1);
    }

} // namespace constraints
