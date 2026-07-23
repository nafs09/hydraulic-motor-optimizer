#pragma once
#include <vector>

namespace constraints {

	void project(std::vector<double>& x);   // hard bounds
	double penalty(const std::vector<double>& x); // soft penalties

} // namespace constraints
