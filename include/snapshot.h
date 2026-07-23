#pragma once
#include <atomic>
#include <mutex>
#include <chrono>
#include "design.h"
#include "hydraulic_model.h"

struct Snapshot {
    std::atomic<int> observations{ 0 };
    std::atomic<double> best_cost{ std::numeric_limits<double>::infinity() };
    mutable std::mutex mtx;
    SimulationResult sim;   // last full simulation result (updated only on new best)
    Design best_design;
    std::chrono::steady_clock::time_point last_update;
};