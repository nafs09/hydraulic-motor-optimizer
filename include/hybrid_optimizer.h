#pragma once

#include "gp_optimizer.h"
#include "snapshot.h"
#include <vector>

struct HybridConfig {
    int n_iterations = 60;
    int n_candidates = 400;
    double acq_kappa = 2.0;
    int local_refine_steps = 20;
};

std::vector<double> run_hybrid_optimisation(
    GPOptimizer& gp,
    const std::vector<double>& x0,
    const HybridConfig& cfg = HybridConfig(),
    Snapshot* snapshot_ptr = nullptr
);
