#pragma once
#include <cmath>

namespace config {

    // BUILD / MODE
    inline constexpr bool RUN_HYBRID = true;
    inline constexpr unsigned int RNG_SEED = 42;

    // GEOMETRY
    inline constexpr int N_CTRL    = 20;   // Fourier harmonic coefficients in design vector
    inline constexpr int N_PISTONS = 7;
    inline constexpr int N_LOBES   = 6;
    inline constexpr int N_PORTS   = 12;
    inline constexpr double R_BASE = 0.065;
    inline constexpr double DXF_SCALE    = 1000.0;
    inline constexpr int    DXF_N_POINTS = 3600;   // 0.1° resolution — smooth in SolidWorks

    // CAM SAMPLING
    // Number of BSpline control points used to represent the cam profile.
    // Increased from N_CTRL=20 to 360 so that the piecewise-linear BSpline
    // is effectively smooth (0.019 mm max interpolation error vs 0.074 mm
    // for 20 CPs which produced the visible kinks in SolidWorks).
    // The design vector still has N_CTRL=20 Fourier coefficients; cam_utils
    // converts them to N_CAM_SAMPLE BSpline CPs.
    inline constexpr int N_CAM_SAMPLE = 360;

    // SIMULATION
    inline constexpr int    STEPS = 15003;   // 3 × STEPS_PER_REV @ OMEGA=62.83, DT=2e-5
    inline constexpr double DT    = 2.0e-5;
    inline constexpr double T_SIM = DT * STEPS;
    inline constexpr bool   LOG_EXTENDED = true;

    // FLUID PROPERTIES (ISO VG 46 hydraulic oil at 50 °C)
    inline constexpr double BULK_MODULUS = 1.4e9;    // Pa — effective (includes air entrainment)
    // BUG FIX: V_DEAD was 1e-5 m³ = 140 % of swept volume, which is physically absurd.
    // Typical hydraulic piston motor dead volume = 10–20 % of swept volume.
    // Swept volume per piston = A_P × STROKE = 5.94e-4 × 0.012 = 7.13e-6 m³
    // Dead volume = 15 % × 7.13e-6 = 1.07e-6 m³
    inline constexpr double V_DEAD = 1.07e-6;        // m³ — dead volume per piston chamber
    inline constexpr double CD     = 0.65;            // discharge coefficient
    inline constexpr double RHO    = 860.0;           // kg/m³
    inline constexpr double MU     = 0.035;           // Pa·s (dynamic viscosity @ 50 °C)

    // PRESSURE BOUNDARIES
    inline constexpr double P_SUP   = 4.5e7;   // Pa — supply pressure (450 bar)
    inline constexpr double P_TANK  = 1.5e5;   // Pa — return pressure (1.5 bar)
    inline constexpr double P_VAPOR = 2.0e3;   // Pa — cavitation threshold for mineral oil

    // PISTON / CYLINDER GEOMETRY
    inline constexpr double D_PISTON = 0.0275;
    inline constexpr double A_P      = 3.14159265358979323846 * D_PISTON * D_PISTON / 4.0;
    inline constexpr double STROKE   = 0.012;
    inline constexpr double M_P      = 0.8;
    inline constexpr double CLEARANCE = 1.5e-5;   // radial clearance (m)
    inline constexpr double L_LEAK    = 0.040;    // piston sealing land length (m)

    // SPRING RETURN
    inline constexpr double K_SPRING    = 15000.0;
    inline constexpr double F_SPRING_PRE = 100.0;
    inline constexpr double PRELOAD     = 0.0;
    inline constexpr double C_DAMP      = 500.0;

    // FRICTION MODEL
    inline constexpr double F_COULOMB   = 25.0;
    inline constexpr double B_VISCOUS   = 50.0;
    inline constexpr double V_STRIBECK  = 0.05;

    // VALVE / PORT GEOMETRY
    inline constexpr double A_PORT_MAX           = 3.5e-4;
    inline constexpr double PORT_HALF_WIDTH      = 0.01;
    inline constexpr double VALVE_WIDTH_DEFAULT  = 0.537 / 2 + 0.005;
    inline constexpr double VALVE_OVERLAP_DEFAULT = 0.01;

    // OPERATING SPEED
    inline constexpr double OMEGA = 62.83;   // rad/s (≈ 600 rpm)

    // THERMAL MODEL
    inline constexpr double T_AMBIENT      = 293.15;
    inline constexpr double C_P_OIL        = 1900.0;
    inline constexpr double M_OIL_THERMAL  = 0.8;
    inline constexpr double UA_HEAT        = 15.0;

    // OPTIMISATION
    inline constexpr int    N_INIT       = 15;
    inline constexpr int    N_BO_ITER    = 60;
    inline constexpr int    N_CANDIDATES = 800;
    inline constexpr double ACQ_KAPPA    = 2.0;

} // namespace config
