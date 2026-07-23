#include "bspline.h"
#include <cmath>
#include <algorithm>

BSpline::BSpline(const std::vector<double>& control_points) : m_cp(control_points) {}

int BSpline::num_control_points() const { return (int)m_cp.size(); }
const std::vector<double>& BSpline::control_points() const { return m_cp; }
void BSpline::set_control_points(const std::vector<double>& cp) { m_cp = cp; }

bool BSpline::is_valid() const {
    if (m_cp.size() < 4) return false;
    double maxv = 0.0;
    for (double v : m_cp) maxv = std::max(maxv, std::abs(v));
    if (maxv < 1e-6) return false;
    for (size_t i = 1; i < m_cp.size(); ++i) {
        if (std::abs(m_cp[i] - m_cp[i - 1]) > 1.0) return false;
    }
    return true;
}

// Periodic linear interpolation across control points
double BSpline::evaluate(double theta) const {
    if (m_cp.empty()) return 0.0;
    const double two_pi = 2.0 * M_PI;
    double t = fmod(theta, two_pi);
    if (t < 0) t += two_pi;
    double u = t / two_pi * m_cp.size();
    double fu = floor(u);
    int i0 = static_cast<int>(fu) % m_cp.size();
    int i1 = (i0 + 1) % m_cp.size();
    double frac = u - fu;
    return (1.0 - frac) * m_cp[i0] + frac * m_cp[i1];
}

double BSpline::fd_derivative(double theta) const {
    const double h = 1e-6;
    return (evaluate(theta + h) - evaluate(theta - h)) / (2.0 * h);
}
double BSpline::fd_second_derivative(double theta) const {
    const double h = 1e-6;
    return (evaluate(theta + h) - 2.0 * evaluate(theta) + evaluate(theta - h)) / (h * h);
}

double BSpline::evaluate_derivative(double theta) const {
    return fd_derivative(theta);
}
double BSpline::evaluate_second_derivative(double theta) const {
    return fd_second_derivative(theta);
}
