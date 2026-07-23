#include "fluid_model.h"
#include "config.h"
#include <algorithm>

namespace fluid_model {

    FluidState evaluate(double P) {
        FluidState s;
        s.pressure = P;
        s.pressure = std::clamp(s.pressure, config::P_TANK, config::P_SUP);
        s.cavitating = (P < config::P_TANK + 1e3);
        s.beta_eff = config::BULK_MODULUS;
        if (s.cavitating) s.beta_eff *= 0.1;
        return s;
    }

} // namespace fluid_model
