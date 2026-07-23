#pragma once
#include "design.h"
#include <vector>
#include <limits>
#include <mutex>
#include <cmath>
#include <utility>

// One row of optimizer history logged after every update()
struct ConvergencePoint {
    int    observation;     // sequential index (1-based)
    double cost;            // cost of this evaluation
    double best_so_far;     // running minimum cost
    double mean_uncertainty;// average posterior std-dev across recent candidates
};

class GPOptimizer {
public:
    static constexpr int GP_MAX_POINTS = 200;

    GPOptimizer(
        double length_scale    = 1.0,
        double signal_variance = 1.0,
        double noise_variance  = 1e-3
    );

    void update(const Design& d, double cost);

    std::pair<double, double> predict(const Design& d) const;

    double acquisition(const Design& d, double kappa = 2.0) const;

    int           num_observations()  const;
    double        best_cost()         const;
    const Design& best_design()       const;

    // Full convergence history — one point per update() call.
    // Thread-safe copy.
    std::vector<ConvergencePoint> convergence_history() const;

private:
    static std::vector<double> to_features(const Design& d);
    double kernel(const std::vector<double>& a,
                  const std::vector<double>& b) const;
    void   rebuild_alpha();
    static void fwd_sub(const std::vector<double>& L, int n,
                        const std::vector<double>& b,
                        std::vector<double>& v);
    static void bck_sub(const std::vector<double>& L, int n,
                        const std::vector<double>& b,
                        std::vector<double>& v);

    double m_length_scale;
    double m_signal_var;
    double m_noise_var;

    double& L(int i, int j) { return m_L_mat[i * m_cap + j]; }
    double L(int i, int j) const { return m_L_mat[i * m_cap + j]; }

    int m_cap{ GP_MAX_POINTS };
    int m_n{ 0 };

    std::vector<std::vector<double>> m_X;
    std::vector<double>              m_y;
    std::vector<double>              m_L_mat;
    std::vector<double>              m_alpha;
    double m_y_mean { 0.0 };
    double m_y_std  { 1.0 };

    double m_best_cost { std::numeric_limits<double>::infinity() };
    Design m_best_design;

    std::vector<ConvergencePoint> m_history;

    mutable std::mutex m_mutex;
};
