#pragma once

#include "gp_optimizer.h"
#include "bspline.h"

namespace cam_utils {

    BSpline build_cam_from_design(const Design& d);

}
