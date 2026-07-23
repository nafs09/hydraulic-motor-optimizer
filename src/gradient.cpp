#include "gradient.h"
#include <vector>

std::vector<double> numerical_gradient(
    const std::vector<double>& x,
    double (*f)(const std::vector<double>&)
) {
    std::vector<double> g(x.size(), 0.0);
    const double h = 1e-6;
    for (size_t i = 0; i < x.size(); ++i) {
        std::vector<double> xp = x;
        std::vector<double> xm = x;
        xp[i] += h;
        xm[i] -= h;
        double fp = f(xp);
        double fm = f(xm);
        g[i] = (fp - fm) / (2.0 * h);
    }
    return g;
}
