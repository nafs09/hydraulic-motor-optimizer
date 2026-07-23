#include "design_vector.h"
#include "config.h"
#include <stdexcept>

namespace design_vector {

    std::vector<double> pack(const Design& d) {
        std::vector<double> x;
        x.reserve(config::N_CTRL + 3);
        for (int i = 0; i < config::N_CTRL; ++i) {
            double v = 0.0;
            if (i < (int)d.cam.size()) v = d.cam[i];
            x.push_back(v);
        }
        x.push_back(d.valve_shift);
        x.push_back(d.valve_width);
        x.push_back(d.valve_overlap);
        return x;
    }

    Design unpack(const std::vector<double>& x) {
        if ((int)x.size() != config::N_CTRL + 3) {
            throw std::runtime_error("design_vector::unpack: wrong vector length");
        }
        Design d;
        d.cam.resize(config::N_CTRL);
        for (int i = 0; i < config::N_CTRL; ++i) d.cam[i] = x[i];
        d.valve_shift = x[config::N_CTRL + 0];
        d.valve_width = x[config::N_CTRL + 1];
        d.valve_overlap = x[config::N_CTRL + 2];
        return d;
    }

} // namespace design_vector
