#include "local_optimizer.h"
#include "gradient.h"
#include "objective.h"
#include "constraints.h"
#include "config.h"
#include <vector>
#include <cmath>
#include <algorithm>

std::vector<double> local_optimize(const std::vector<double>& x0) {
    std::vector<double> x = x0;

    // Step size tuned to the cumsum-relu parameterisation:
    // cam CPs are increments ~STROKE/N_CTRL ≈ 6e-4 m, so a step of 1e-4 is
    // ~1/6 of one increment — meaningful but not overshooting.
    // Valve params are O(0.1–1), step 1e-4 is very cautious there (fine).
    const double alpha = 1e-4;

    // 20 gradient steps: enough to find the local basin without burning
    // 50 × 23 × 2 = 2300 objective calls per candidate (was the bottleneck).
    // 20 × 23 × 2 = 920 objective calls — 2.5× faster local search.
    for (int iter = 0; iter < 20; ++iter) {
        auto g = numerical_gradient(x, evaluate_objective);
        for (size_t i = 0; i < x.size(); ++i)
            x[i] -= alpha * g[i];
        constraints::project(x);
    }
    return x;
}
