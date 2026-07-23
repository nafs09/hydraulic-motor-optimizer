#include "hydraulic_model.h"
#include "config.h"
#include "common.h"
#include "fluid_model.h"
#include <cmath>
#include <limits>
#include <algorithm>
#include <numeric>

HydraulicModel::HydraulicModel(const BSpline& cam,
                               double valve_shift,
                               double valve_width,
                               double valve_overlap)
    : m_cam(cam), m_valve_shift(valve_shift),
      m_valve_width(valve_width), m_valve_overlap(valve_overlap)
{
    m_pistons.resize(config::N_PISTONS);
    for (auto& p : m_pistons) { p.P=config::P_TANK; p.x=0; p.v=0; p.a=0; }
}

double HydraulicModel::cam_lift(double theta) const { return m_cam.evaluate(theta); }
double HydraulicModel::cam_lift_vel(double theta) const { return m_cam.evaluate_derivative(theta)*config::OMEGA; }
double HydraulicModel::cam_lift_acc(double theta) const { return m_cam.evaluate_second_derivative(theta)*config::OMEGA*config::OMEGA; }

double HydraulicModel::smooth_port_area(double theta_e, double phase_open) const {
    double phase = std::fmod(theta_e - phase_open + 4.0*M_PI, 2.0*M_PI);
    const double open_arc    = 2.0*M_PI*m_valve_width;
    const double overlap_arc = 2.0*M_PI*std::max(m_valve_overlap, 1e-6);
    if (phase < overlap_arc)             return config::A_PORT_MAX*(phase/overlap_arc);
    if (phase < open_arc - overlap_arc)  return config::A_PORT_MAX;
    if (phase < open_arc)                return config::A_PORT_MAX*(open_arc-phase)/overlap_arc;
    return 0.0;
}

double HydraulicModel::supply_port_area(double theta_e) const {
    return smooth_port_area(theta_e, common::wrap_angle(m_valve_shift));
}
double HydraulicModel::tank_port_area(double theta_e) const {
    return smooth_port_area(theta_e, common::wrap_angle(m_valve_shift + M_PI));
}

// Solve for the equilibrium chamber pressure where net flow = dV/dt.
// The hydraulic time constant τ = V/(β·G) ≈ 0.002 ms, far smaller than DT=0.02 ms,
// so pressure reaches equilibrium within every timestep — solving directly for P_eq
// is more accurate than any ODE integrator and eliminates all stiffness issues.
// Uses 24-iteration bisection → error < 3 Pa.
double HydraulicModel::solve_pressure_eq(double A_sup, double A_tank,
                                          double G_leak, double dV_dt) const {
    auto Q_net = [&](double P) -> double {
        double Qs = (P < config::P_SUP)
                  ? config::CD * A_sup  * std::sqrt(2.0*std::max(0.0, config::P_SUP-P) / config::RHO)
                  : 0.0;
        double Qt = (P > config::P_TANK)
                  ? config::CD * A_tank * std::sqrt(2.0*std::max(0.0, P-config::P_TANK) / config::RHO)
                  : 0.0;
        double Ql = G_leak * std::max(0.0, P - config::P_TANK);
        return Qs - Qt - Ql - dV_dt;
    };

    double lo = config::P_VAPOR;
    double hi = config::P_SUP * 1.05;
    if (Q_net(lo) <= 0.0) return lo;  // demand exceeds max supply — cavitation floor
    if (Q_net(hi) >= 0.0) return hi;  // excess supply — relief ceiling

    for (int i = 0; i < 24; ++i) {
        double mid = 0.5*(lo + hi);
        if (Q_net(mid) > 0.0) lo = mid; else hi = mid;
    }
    return 0.5*(lo + hi);
}

SimulationResult HydraulicModel::run(int steps, double dt) {
    SimulationResult res;
    res.max_pressure      = config::P_TANK;
    res.min_pressure      = config::P_SUP;
    res.mean_torque       = 0.0;
    res.torque_ripple_pct = 0.0;
    res.p_loss            = 0.0;
    res.total_energy      = 0.0;
    res.volumetric_eff    = 0.0;
    res.cavitation_events = 0;
    res.diverged          = false;

    // Allocate per-piston pressure storage
    res.piston_pressure.assign(config::N_PISTONS, std::vector<double>());
    for (auto& pp : res.piston_pressure) pp.reserve(steps);

    res.time.reserve(steps);
    res.total_torque.reserve(steps);
    res.total_flow.reserve(steps);
    res.omega.reserve(steps);
    res.theta.reserve(steps);
    res.temperature.reserve(steps);

    // Leakage conductance: Hagen-Poiseuille, annular clearance
    // G_leak such that Q_leak = G_leak * (P - P_TANK)
    const double G_leak = (M_PI * config::D_PISTON
                           * config::CLEARANCE * config::CLEARANCE * config::CLEARANCE)
                          / (12.0 * config::MU * config::L_LEAK);

    double theta      = 0.0;
    const double omega = config::OMEGA;
    double time        = 0.0;
    double p_loss_sum  = 0.0;
    double Q_sup_total = 0.0;
    double Q_th_total  = 0.0;

    for (int step = 0; step < steps; ++step) {
        double torque      = 0.0;
        double flow_step   = 0.0;
        double p_loss_step = 0.0;

        for (int i = 0; i < config::N_PISTONS; ++i) {
            double piston_offset = 2.0*M_PI*(double)i/(double)config::N_PISTONS;
            double theta_p = common::wrap_angle(theta + piston_offset);
            double theta_e = common::wrap_angle(theta_p*(double)config::N_LOBES);

            double lift  = m_cam.evaluate(theta_e);
            double slope = m_cam.evaluate_derivative(theta_e);
            double v_piston = slope * omega * (double)config::N_LOBES;
            double dV_dt    = config::A_P * v_piston;

            double A_sup  = supply_port_area(theta_e);
            double A_tank = tank_port_area(theta_e);

            // Equilibrium pressure this step (no ODE stiffness)
            double P = solve_pressure_eq(A_sup, A_tank, G_leak, dV_dt);

            if (P <= config::P_VAPOR) res.cavitation_events++;

            torque += P * config::A_P * slope * (double)config::N_LOBES;

            double Q_sup_i = (P < config::P_SUP)
                ? config::CD * A_sup * std::sqrt(2.0*std::max(0.0,config::P_SUP-P)/config::RHO)
                : 0.0;
            double Q_leak_i = G_leak * std::max(0.0, P - config::P_TANK);

            p_loss_step += std::abs(P - config::P_TANK) * Q_leak_i;
            flow_step   += Q_sup_i;
            Q_sup_total += Q_sup_i * dt;

            res.max_pressure = std::max(res.max_pressure, P);
            res.min_pressure = std::min(res.min_pressure, P);
        }

        Q_th_total += config::A_P * config::STROKE
                    * (double)config::N_PISTONS
                    * (double)config::N_LOBES
                    * omega / (2.0*M_PI) * dt;

        m_thermal.step(p_loss_step, dt);

        res.time.push_back(time);
        res.total_torque.push_back(torque);
        res.total_flow.push_back(flow_step);
        res.omega.push_back(omega);
        res.theta.push_back(theta);
        res.temperature.push_back(m_thermal.temperature());

        p_loss_sum       += p_loss_step;
        res.total_energy += std::abs(torque) * omega * dt;

        theta += omega * dt;
        time  += dt;
    }

    // Mean torque
    double sum_t = 0.0;
    for (double t : res.total_torque) sum_t += t;
    res.mean_torque = res.total_torque.empty() ? 0.0 : sum_t/(double)res.total_torque.size();

    // Ripple
    if (!res.total_torque.empty() && std::abs(res.mean_torque) > 1e-9) {
        double tmax = *std::max_element(res.total_torque.begin(), res.total_torque.end());
        double tmin = *std::min_element(res.total_torque.begin(), res.total_torque.end());
        res.torque_ripple_pct = (tmax - tmin) / std::abs(res.mean_torque) * 100.0;
    }

    // Vol eff: Q_displaced / Q_consumed  (motor definition: leakage draws extra flow)
    res.volumetric_eff = (Q_sup_total > 1e-12)
                       ? std::min(1.0, Q_th_total / Q_sup_total)
                       : 0.0;

    res.p_loss        = (steps > 0) ? p_loss_sum/(double)steps : 0.0;
    res.thermal_power = res.p_loss;

    return res;
}
