#include "thermal_model.h"
#include "config.h"
#include <cmath>

ThermalModel::ThermalModel() : m_T(config::T_AMBIENT) {}

void ThermalModel::step(double P_loss, double dt) {
    double mcp = config::M_OIL_THERMAL * config::C_P_OIL;
    double dT = (P_loss - config::UA_HEAT * (m_T - config::T_AMBIENT)) * dt / mcp;
    m_T += dT;
}

double ThermalModel::temperature() const { return m_T; }

double ThermalModel::viscosity() const {
    return MU_REF * std::exp(-ALPHA * (m_T - T_REF));
}
