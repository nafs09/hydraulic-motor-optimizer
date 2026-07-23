#pragma once
#include <vector>

std::vector<double> numerical_gradient(
    const std::vector<double>& x,
    double (*f)(const std::vector<double>&)
);
