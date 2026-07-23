#pragma once
#include "design.h"
#include <vector>

namespace design_vector {

	std::vector<double> pack(const Design& d);
	Design unpack(const std::vector<double>& x);

} // namespace design_vector
