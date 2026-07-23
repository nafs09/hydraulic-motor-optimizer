#pragma once
#include <vector>

class BSpline {
public:
    explicit BSpline(const std::vector<double>& control_points = {});
    double evaluate(double theta) const;
    double evaluate_derivative(double theta) const;
    double evaluate_second_derivative(double theta) const;
    int num_control_points() const;
    const std::vector<double>& control_points() const;
    bool is_valid() const;
    void set_control_points(const std::vector<double>& cp);

private:
    std::vector<double> m_cp;
    static constexpr int DEGREE = 3;
    double fd_derivative(double theta) const;
    double fd_second_derivative(double theta) const;
};
