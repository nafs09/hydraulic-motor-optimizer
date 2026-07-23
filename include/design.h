#pragma once
#include <vector>
#include "config.h"

struct Design {
    std::vector<double> cam; // N_CTRL control points
    double valve_shift = 0.0;
    double valve_width = config::VALVE_WIDTH_DEFAULT;
    double valve_overlap = config::VALVE_OVERLAP_DEFAULT;
};
