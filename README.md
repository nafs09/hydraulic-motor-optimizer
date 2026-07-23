# Hydraulic Motor Cam Profile Optimizer

A C++ framework for optimizing radial piston hydraulic motor cam profiles using **Gaussian Process Bayesian Optimization** with **hybrid local refinement**.

---

# Overview

This project implements a closed-loop design optimization system for hydraulic motor cam geometries. Given physical constraints (pressure, flow, torque ripple targets), it searches for optimal cam profiles that minimize torque ripple while maintaining volumetric efficiency.

---

# Key Features

- **Cubic B-Spline Parameterization**
  - Smooth, differentiable cam profiles with guaranteed base-circle continuity.
- **Gaussian Process Bayesian Optimization**
  - Sample-efficient global search with uncertainty quantification.
- **Hybrid Optimization Strategy**
  - Bayesian optimization for global exploration followed by Nelder-Mead local refinement.
- **Full Hydraulic Simulation**
  - Pressure dynamics, valve flow, piston motion, leakage, cavitation detection, and efficiency calculations.
- **Thermal Modeling**
  - Lumped-parameter oil temperature model with viscosity feedback.
- **SolidWorks Export**
  - Direct export to `.sldcrv` format for CAD integration.
- **SVG Visualization**
  - Automatic generation of performance plots, FFT spectra, convergence history, and dashboard.

---

# Physics Model

The simulator models a **7-piston, 6-lobe radial piston hydraulic motor**.

| Parameter | Value | Units |
|-----------|------:|------|
| Supply Pressure | 450 | bar |
| Piston Diameter | 27.5 | mm |
| Stroke | 12 | mm |
| Operating Speed | 600 | rpm |
| Working Fluid | ISO VG 46 | @ 50°C |
| Effective Bulk Modulus | 1.4 | GPa |

## Modeled Phenomena

- Compressible fluid dynamics (finite bulk modulus)
- Orifice flow through valve ports
- Piston leakage (annular gap flow)
- Coulomb, viscous, and Stribeck friction
- Spring return dynamics
- Cavitation detection with bulk modulus reduction
- Lumped thermal model with convective cooling

---

# Optimization

The optimizer searches a **23-dimensional design space**.

| Variable | Count |
|-----------|------:|
| Fourier sine-power coefficients | 20 |
| Valve timing shift | 1 |
| Valve width | 1 |
| Valve overlap | 1 |

## Objective Function

```text
cost =
    w₁ · torque_ripple(%)
  + w₂ · (1 − volumetric_efficiency)
  + w₃ · constraint_penalty
```

---

# Optimization Algorithm

1. Generate an initial **Latin Hypercube** sample set.
2. Evaluate each design using the full hydraulic simulator.
3. Fit a **Gaussian Process surrogate model**.
4. Optimize the **Lower Confidence Bound (LCB)** acquisition function.
5. Refine promising candidates using **Nelder-Mead**.
6. Evaluate refined designs.
7. Update the surrogate and repeat until convergence.

Each simulation evaluates:

- **15,003 timesteps**
- **3 shaft revolutions**
- Complete pressure, flow, thermal, and efficiency calculations

---

# Building

## Prerequisites

- C++17 compatible compiler
  - GCC 9+
  - Clang 10+
  - MSVC 2019+
- CMake 3.16+

## Build

```bash
cd "Hydraulic Motor Code"

mkdir build
cd build

cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . --parallel
```

---

# Build Options

| Option | Default | Description |
|---------|---------|-------------|
| `CMAKE_BUILD_TYPE` | `Release` | `Debug`, `Release`, `RelWithDebInfo` |
| `ENABLE_PLOTTING` | `ON` | Enable SVG plot generation |

---

# Running

```bash
./hydraulic_motor_optimizer
```

---

# Output Files

The optimizer generates the following files.

| File | Description |
|------|-------------|
| `cam_profile.sldcrv` | SolidWorks cam profile |
| `plot_torque.svg` | Torque vs shaft angle |
| `plot_pressure.svg` | Chamber pressures |
| `plot_flow.svg` | Volumetric flow |
| `plot_power.svg` | Hydraulic power |
| `plot_fft.svg` | Torque frequency spectrum |
| `plot_cam.svg` | Cam profile |
| `plot_temperature.svg` | Oil temperature history |
| `plot_convergence.svg` | Optimization convergence |
| `dashboard.html` | Combined interactive dashboard |
| `metrics.csv` | Summary performance metrics |
| `timeseries.csv` | Complete simulation output |

---

# SolidWorks Import

1. Open **SolidWorks**
2. Navigate to:

   ```
   Insert → Curve → Curve Through XYZ Points
   ```

3. Select:

   ```
   cam_profile.sldcrv
   ```

4. The curve is imported as a **closed profile in millimetres**.

---

# Project Structure

```text
Hydraulic Motor Code/
├── CMakeLists.txt
├── include/
│   ├── bspline.h           # Piecewise-linear curve representation
│   ├── cam_utils.h         # Fourier → cam conversion
│   ├── common.h            # Utilities, RNG, timing
│   ├── config.h            # Physical & optimization parameters
│   ├── constraints.h       # Design constraints
│   ├── csv_utils.h         # CSV output
│   ├── design.h            # Design definition
│   ├── design_vector.h     # Design ↔ vector conversion
│   ├── dxf_export.h        # SolidWorks export
│   ├── fft.h               # Frequency analysis
│   ├── fluid_model.h       # Compressible fluid model
│   ├── gp_optimizer.h      # Gaussian Process optimizer
│   ├── gradient.h          # Numerical differentiation
│   ├── hybrid_optimizer.h  # Bayesian + local optimizer
│   ├── hydraulic_model.h   # Hydraulic simulation
│   ├── local_optimizer.h   # Nelder-Mead optimizer
│   ├── objective.h         # Objective function
│   ├── plotting.h          # SVG plotting
│   ├── snapshot.h          # Live optimization state
│   └── thermal_model.h     # Oil temperature model
└── src/
    └── corresponding .cpp files
```

---

# Configuration

All configurable parameters are defined in:

```text
include/config.h
```

Categories include:

### Geometry

- `N_PISTONS`
- `N_LOBES`
- `D_PISTON`
- `STROKE`
- `R_BASE`

### Fluid

- `BULK_MODULUS`
- `RHO`
- `MU`
- `V_DEAD`

### Pressure

- `P_SUP`
- `P_TANK`
- `P_VAPOR`

### Valve

- `A_PORT_MAX`
- `PORT_HALF_WIDTH`
- `VALVE_WIDTH_DEFAULT`

### Simulation

- `STEPS`
- `DT`
- `OMEGA`

### Optimization

- `N_INIT`
- `N_BO_ITER`
- `N_CANDIDATES`
- `ACQ_KAPPA`

### Output

- `DXF_SCALE`
- `DXF_N_POINTS`

---

# Current Limitations

- Single-phase hydraulic flow only
- No air entrainment model
- Rigid piston assumption
- Simplified cavitation transition
- Thermal-viscosity coupling not fully integrated into friction model
- Single-objective optimization (no Pareto front)

---

# Future Work

Potential improvements include:

- Multi-objective Bayesian optimization
- Adaptive mesh simulation
- Fluid-structure interaction
- Elastohydrodynamic lubrication (EHL)
- Manufacturing constraint integration
- GPU-accelerated simulation
- Automatic sensitivity analysis

---

# License

This project is protected under the MIT license
