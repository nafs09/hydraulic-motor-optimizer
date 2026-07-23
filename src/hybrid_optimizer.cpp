#include "hybrid_optimizer.h"
#include "cam_utils.h"
#include "hydraulic_model.h"
#include "bspline.h"
#include "design_vector.h"
#include "constraints.h"
#include "objective.h"
#include "local_optimizer.h"
#include "common.h"
#include "snapshot.h"
#include "config.h"
#include <algorithm>
#include <limits>
#include <cmath>

// Mutation scale: cam CPs are relu cumsum increments, typical magnitude ~STROKE/half
// So noise should be on the same order, not the fixed 0.001 used before.
static constexpr double CAM_MUTATE_SIGMA  = config::STROKE / config::N_CTRL;
static constexpr double SHIFT_MUTATE_RANGE = 0.05;   // radians
static constexpr double WIDTH_MUTATE_RANGE = 0.02;   // fraction
static constexpr double OVERLAP_MUTATE_RANGE = 0.005;

static std::vector<double> mutate_candidate(const std::vector<double>& base) {
    std::vector<double> cand = base;
    // Mutate a random subset of cam CPs with appropriate scale
    for (int k = 0; k < 4; ++k) {
        int idx = static_cast<int>(common::rand_range(0, config::N_CTRL - 1));
        cand[idx] += common::rand_normal() * CAM_MUTATE_SIGMA;
    }
    cand[config::N_CTRL + 0] += common::rand_range(-SHIFT_MUTATE_RANGE,  SHIFT_MUTATE_RANGE);
    cand[config::N_CTRL + 1] += common::rand_range(-WIDTH_MUTATE_RANGE,  WIDTH_MUTATE_RANGE);
    cand[config::N_CTRL + 2] += common::rand_range(-OVERLAP_MUTATE_RANGE, OVERLAP_MUTATE_RANGE);
    constraints::project(cand);
    return cand;
}

std::vector<double> run_hybrid_optimisation(GPOptimizer& gp,
    const std::vector<double>& x0,
    const HybridConfig& cfg,
    Snapshot* snapshot_ptr)
{
    // One full shaft revolution for live snapshots — fast enough to not block,
    // accurate enough to show meaningful torque and ripple estimates.
    const int SNAP_STEPS = static_cast<int>(
        std::ceil(2.0 * M_PI / (config::OMEGA * config::DT)));

    std::vector<double> best_x = x0;
    double best_cost = evaluate_objective(best_x);
    gp.update(design_vector::unpack(best_x), best_cost);

    for (int iter = 0; iter < cfg.n_iterations; ++iter) {
        // Score candidates via GP acquisition function
        std::vector<std::pair<double, std::vector<double>>> scored;
        scored.reserve(cfg.n_candidates);
        for (int i = 0; i < cfg.n_candidates; ++i) {
            std::vector<double> cand = mutate_candidate(best_x);
            Design d = design_vector::unpack(cand);
            double acq = gp.acquisition(d, cfg.acq_kappa);
            scored.emplace_back(acq, std::move(cand));
        }

        // Take the most promising (lowest LCB) candidates for local refinement
        std::sort(scored.begin(), scored.end(),
            [](const auto& a, const auto& b) { return a.first < b.first; });

        int n_top = std::max(1, cfg.n_candidates / 20);
        for (int k = 0; k < n_top; ++k) {
            auto refined = local_optimize(scored[k].second);
            constraints::project(refined);

            double cost = evaluate_objective(refined);
            Design d    = design_vector::unpack(refined);
            gp.update(d, cost);

            if (snapshot_ptr) {
                snapshot_ptr->observations.fetch_add(1, std::memory_order_relaxed);
                double prev_best = snapshot_ptr->best_cost.load(std::memory_order_relaxed);
                if (cost < prev_best) {
                    snapshot_ptr->best_cost.store(cost, std::memory_order_relaxed);
                    std::lock_guard<std::mutex> lk(snapshot_ptr->mtx);
                    snapshot_ptr->best_design = d;

                    // FIX: was BSpline cam(d.cam) — raw CPs bypass cam_utils,
                    // giving near-zero torque and 20000%+ ripple in progress display.
                    BSpline cam = cam_utils::build_cam_from_design(d);
                    HydraulicModel model(cam, d.valve_shift, d.valve_width, d.valve_overlap);
                    snapshot_ptr->sim = model.run(SNAP_STEPS, config::DT);
                    snapshot_ptr->last_update = std::chrono::steady_clock::now();
                }
            }

            if (cost < best_cost) {
                best_cost = cost;
                best_x    = refined;
            }
        }
    }
    return best_x;
}
