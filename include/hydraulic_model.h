#pragma once

#include "config.h"
#include "bspline.h"
#include "thermal_model.h"
#include <vector>
#include <limits>

struct PistonState {
    double P;
    double x;
    double v;
    double a;
};

struct SimulationResult {
    std::vector<double> time;
    std::vector<double> total_torque;
    std::vector<double> total_flow;
    std::vector<double> omega;
    std::vector<double> theta;

    std::vector<std::vector<double>> piston_pressure;
    std::vector<std::vector<double>> piston_x;
    std::vector<std::vector<double>> piston_v;
    std::vector<std::vector<double>> piston_a;
    std::vector<std::vector<double>> port_open_fraction;

    std::vector<double> temperature;

    double mean_torque = 0.0;
    double torque_ripple_pct = 0.0;
    double volumetric_eff = 0.0;
    double total_energy = 0.0;
    double p_loss = 0.0;
    double thermal_power = 0.0;
    double max_pressure = -std::numeric_limits<double>::infinity();
    double min_pressure = std::numeric_limits<double>::infinity();
    int cavitation_events = 0;
    bool diverged = false;
};

class HydraulicModel {
public:
    HydraulicModel(const BSpline& cam, double valve_shift, double valve_width, double valve_overlap);
    SimulationResult run(int steps = config::STEPS, double dt = config::DT);

private:
    BSpline m_cam;
    double m_valve_shift;
    double m_valve_width;
    double m_valve_overlap;
    ThermalModel m_thermal;
    std::vector<PistonState> m_pistons;

    // helpers
    double cam_lift(double theta) const;
    double cam_lift_vel(double theta) const;
    double cam_lift_acc(double theta) const;
    double supply_port_area(double theta) const;
    double tank_port_area(double theta) const;
    double smooth_port_area(double theta, double phase_open) const;
    double solve_pressure_eq(double A_sup, double A_tank, double G_leak, double dV_dt) const;
};
