#include "objective.h"
#include "design_vector.h"
#include "cam_utils.h"
#include "hydraulic_model.h"
#include "bspline.h"
#include "constraints.h"
#include "config.h"
#include <cmath>
#include <algorithm>

double evaluate_objective(const std::vector<double>& x) {
    std::vector<double> xx = x;
    constraints::project(xx);
    Design d = design_vector::unpack(xx);
    BSpline cam = cam_utils::build_cam_from_design(d);
    HydraulicModel model(cam, d.valve_shift, d.valve_width, d.valve_overlap);

    const int STEPS_PER_REV = static_cast<int>(
        std::ceil(2.0 * M_PI / (config::OMEGA * config::DT)));
    auto sim = model.run(STEPS_PER_REV, config::DT);

    if (std::isnan(sim.mean_torque) || std::isnan(sim.torque_ripple_pct)
            || std::abs(sim.max_pressure) > 1e12)
        return 1e18;

    double torque_error   = std::abs(sim.mean_torque - 2100.0);
    double ripple_penalty = std::max(0.0, sim.torque_ripple_pct - 25.0) * 2000.0;
    double vol_eff_penalty = 50000.0 * (1.0 - sim.volumetric_eff) * (1.0 - sim.volumetric_eff);

    // Light cavitation penalty — physical events only (numerical ones now eliminated)
    double cav_penalty  = (double)sim.cavitation_events * 50.0;

    double peak_penalty = 0.0;
    for (double t : sim.total_torque)
        if (t > sim.mean_torque * 2.0) peak_penalty += 500.0;

    return (torque_error * 20.0) + ripple_penalty + vol_eff_penalty
         + cav_penalty + peak_penalty;
}
