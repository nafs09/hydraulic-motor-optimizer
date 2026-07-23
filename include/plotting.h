#pragma once
#include <string>
#include <vector>
#include "hydraulic_model.h"
#include "bspline.h"
#include "gp_optimizer.h"

struct PlotSeries {
    std::vector<double> x;
    std::vector<double> y;
    std::string label;
    std::string color;
};

namespace plot_svg {

    // Low-level generic plotter (unchanged API)
    void plot_svg(const std::string& filename, const std::string& title,
                  const std::string& x_label, const std::string& y_label,
                  const std::vector<PlotSeries>& series,
                  int width_px = 960, int height_px = 580);

    // ── Per-signal plots ──────────────────────────────────────────────────────

    // Torque vs shaft angle (degrees). Shows smoothed ripple peaks, mean torque
    // as a dashed reference, ±ripple shaded band, and target 2100 N·m line.
    void plot_torque_vs_angle(const SimulationResult& sim,
                               const std::string& filename = "plot_torque.svg");

    // Hydraulic power (T × ω) vs shaft angle in kW.
    void plot_power_vs_angle(const SimulationResult& sim,
                              const std::string& filename = "plot_power.svg");

    // Total volumetric flow rate vs shaft angle in L/min.
    void plot_flow_vs_angle(const SimulationResult& sim,
                             const std::string& filename = "plot_flow.svg");

    // Chamber pressure for all N_PISTONS vs shaft angle in bar.
    // Requires sim.piston_pressure to be populated.
    void plot_pressure_vs_angle(const SimulationResult& sim,
                                 const std::string& filename = "plot_pressure.svg");

    // FFT amplitude spectrum of the torque signal.
    // X = harmonic number (1 = shaft freq ≈ 10 Hz), shown up to 2×N_P×N_L = 84.
    // Vertical markers at key harmonics: N_LOBES, N_PISTONS, N_P×N_L.
    void plot_fft_spectrum(const SimulationResult& sim,
                            const std::string& filename = "plot_fft.svg");

    // Cam lift profile: lift in mm vs lobe angle in degrees (one full lobe, 0-360°).
    void plot_cam_profile(const BSpline& cam,
                           const std::string& filename = "plot_cam.svg");

    // Oil temperature vs simulation time in ms.
    void plot_temperature(const SimulationResult& sim,
                           const std::string& filename = "plot_temperature.svg");

    // ── Composite output ──────────────────────────────────────────────────────

    // Write a raw-data CSV (unchanged API)
    void write_metrics_csv(const SimulationResult& sim,
                            const std::string& filename = "metrics.csv");

    // Write full time-series CSV (every timestep: angle, torque, flow, pressure…)
    void write_timeseries_csv(const SimulationResult& sim,
                               const std::string& filename = "timeseries.csv");

    // Generate all 7 SVG plots + an HTML dashboard that tiles them.
    // All files land next to the binary (working directory).
    // Returns the path to the HTML dashboard file.
    std::string plot_all(const SimulationResult& sim, const BSpline& cam);

} // namespace plot_svg

// Add this include at top of your plotting.h (after existing includes)
// #include "gp_optimizer.h"  -- already needed for convergence plot

namespace plot_svg {

    // Optimizer convergence: best cost (log scale) + mean GP uncertainty vs
    // observation number. Dual Y-axis: left=cost, right=uncertainty (N*m).
    void plot_convergence(const std::vector<ConvergencePoint>& history,
                          const std::string& filename = "plot_convergence.svg");

} // namespace plot_svg (extension)
