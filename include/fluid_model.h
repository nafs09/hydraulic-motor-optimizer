#pragma once

namespace fluid_model {

    struct FluidState {
        double beta_eff;
        double pressure;
        bool cavitating;
    };

    FluidState evaluate(double P);

} // namespace fluid_model
