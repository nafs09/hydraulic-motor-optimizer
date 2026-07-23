#pragma once

class ThermalModel {
public:
    ThermalModel();
    void step(double P_loss, double dt);
    double temperature() const;
    double viscosity() const;

    static constexpr double MU_REF = 0.035;
    static constexpr double T_REF = 323.15;
    static constexpr double ALPHA = 0.035;

private:
    double m_T;
};
