#ifdef _WIN32
#include <windows.h>
#endif

#include <iostream>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>
#include <thread>
#include <atomic>
#include <chrono>
#include <mutex>
#include <cmath>
#include <algorithm>
#include <clocale>

#include "config.h"
#include "design_vector.h"
#include "design.h"
#include "dxf_export.h"
#include "bspline.h"
#include "cam_utils.h"
#include "plotting.h"
#include "hydraulic_model.h"
#include "objective.h"
#include "common.h"
#include "constraints.h"
#include "gp_optimizer.h"
#include "hybrid_optimizer.h"
#include "snapshot.h"

// -- UTF-8 display macros (ASCII-safe source file) -----------------------------
#define UTF8_FULL_BLOCK   "\xE2\x96\x88"
#define UTF8_LIGHT_SHADE "\xE2\x96\x91"
#define UTF8_MID_DOT     "\xC2\xB7"
#define UTF8_ETA         "\xCE\xB7"
#define UTF8_OMEGA       "\xCF\x89"
#define UTF8_DELTA       "\xCE\x94"
#define UTF8_CHECK       "\xE2\x9C\x94"
#define UTF8_WARN        "\xE2\x9A\xA0"

// Box drawing
#define UTF8_BOX_TL      "\xE2\x95\x94"
#define UTF8_BOX_TR      "\xE2\x95\x97"
#define UTF8_BOX_BL      "\xE2\x95\x9A"
#define UTF8_BOX_BR      "\xE2\x95\x9D"
#define UTF8_BOX_H       "\xE2\x95\x90"
#define UTF8_BOX_V       "\xE2\x95\x91"

#define UTF8_ARROW  "\xE2\x86\x92" // ->

using clk = std::chrono::steady_clock;

// -- ANSI support --------------------------------------------------------------
static bool g_ansi = false;

static void init_ansi() {
#ifdef _WIN32
    HANDLE h = GetStdHandle(STD_OUTPUT_HANDLE);
    DWORD  m = 0;

    if (GetConsoleMode(h, &m)) {
        g_ansi = SetConsoleMode(
            h,
            m | ENABLE_VIRTUAL_TERMINAL_PROCESSING
        ) != 0;
    }

    SetConsoleOutputCP(CP_UTF8);
    SetConsoleCP(CP_UTF8);
#else
    g_ansi = true;
#endif
}

static constexpr int ROW_PROGRESS = 6;
static constexpr int ROW_SUMMARY = 7;
static constexpr int ROW_DIVIDER2 = 8;
static constexpr int SCROLL_START = 9;

// -- Colour macros -------------------------------------------------------------
#define A(s)  (g_ansi ? (s) : "")
#define RST   A("\033[0m")
#define BOLD  A("\033[1m")
#define DIM   A("\033[2m")
#define CYAN  A("\033[96m")
#define GREEN A("\033[92m")
#define YEL   A("\033[93m")
#define RED   A("\033[91m")
#define WHT   A("\033[97m")

// -- Helpers -------------------------------------------------------------------
static std::string fmt_dur(long long s) {
    if (s < 60)   return std::to_string(s) + "s";
    if (s < 3600) return std::to_string(s / 60) + "m " + std::to_string(s % 60) + "s";
    return std::to_string(s / 3600) + "h " + std::to_string((s % 3600) / 60) + "m";
}

// Overwrite a fixed header row without touching the scroll region.
// Saves cursor -> jumps to row -> clears line -> writes -> restores cursor.
static void overwrite_row(int row, const std::string& content) {
    if (!g_ansi) return;

    std::cout
        << "\033[s"
        << "\033[" << row << ";1H"
        << "\033[2K"
        << content
        << "\033[u"
        << std::flush;
}

// Build the animated progress bar string (fits in one terminal line)
static std::string make_progress_line(
    int done,
    int total,
    long long elapsed_s,
    int gp_n)
{
    const int W = 32;

    int fill = (total > 0)
        ? std::clamp(done * W / total, 0, W)
        : 0;

    double pct = (total > 0)
        ? 100.0 * done / total
        : 0.0;

    std::ostringstream s;

    s << " ";

    s << CYAN;

    s << "[";

    for (int i = 0; i < fill; ++i)
        s << UTF8_FULL_BLOCK;

    for (int i = fill; i < W; ++i)
        s << UTF8_LIGHT_SHADE;

    s << "]";

    s << RST;

    s << BOLD
        << " "
        << done
        << "/"
        << total
        << " ("
        << std::fixed
        << std::setprecision(1)
        << pct
        << "%)"
        << RST;

    s << DIM
        << " elapsed "
        << fmt_dur(elapsed_s)
        << " gp="
        << gp_n
        << RST;

    return s.str();
}

// Build the one-line best-cost summary for the fixed header
static std::string make_summary_line(double cost, const SimulationResult& s) {
    const char* tc = (std::abs(s.mean_torque - 2100) < 50) ? GREEN
        : (std::abs(s.mean_torque - 2100) < 200) ? YEL : RED;

    const char* rc = (s.torque_ripple_pct < 25) ? GREEN
        : (s.torque_ripple_pct < 60) ? YEL : RED;

    const char* cc = (s.cavitation_events == 0) ? GREEN : RED;

    std::ostringstream o;

    o << " " << DIM << "best " << RST
        << BOLD << YEL
        << std::fixed << std::setprecision(1)
        << cost << RST
        << DIM << " | " << RST
        << "T " << tc
        << std::setprecision(0)
        << s.mean_torque
        << " N" << UTF8_MID_DOT << "m"
        << RST
        << DIM << " | " << RST
        << "ripple " << rc
        << std::setprecision(1)
        << s.torque_ripple_pct << "%"
        << RST
        << DIM << " | " << RST
        << UTF8_ETA << "_v "
        << std::setprecision(1)
        << s.volumetric_eff * 100 << "%"
        << DIM << " | " << RST
        << "cav " << cc
        << s.cavitation_events
        << RST;

    return o.str();
}

// Detailed snapshot block printed INTO the scrolling region
static void print_snapshot_block(
    int obs,
    double cost,
    const Design& d,
    const SimulationResult& s)
{
    const char* tc = (std::abs(s.mean_torque - 2100) < 50) ? GREEN
        : (std::abs(s.mean_torque - 2100) < 200) ? YEL : RED;

    const char* rc = (s.torque_ripple_pct < 25) ? GREEN
        : (s.torque_ripple_pct < 60) ? YEL : RED;

    std::cout
        << "\n"
        << BOLD << CYAN
        << " > obs " << obs
        << RST
        << DIM
        << " cost "
        << RST
        << BOLD << YEL
        << std::fixed << std::setprecision(3)
        << cost
        << RST
        << "\n"

        << " torque "
        << tc << BOLD
        << std::setprecision(1)
        << s.mean_torque
        << " N" << UTF8_MID_DOT << "m"
        << RST

        << " ripple "
        << rc << BOLD
        << std::setprecision(1)
        << s.torque_ripple_pct
        << "%"
        << RST

        << " " << UTF8_ETA << "_v "
        << std::setprecision(1)
        << s.volumetric_eff * 100
        << "%"

        << " cav "
        << s.cavitation_events
        << "\n"

        << DIM
        << " valve: shift="
        << std::setprecision(4)
        << d.valve_shift
        << " rad width="
        << d.valve_width
        << " overlap="
        << d.valve_overlap
        << RST
        << "\n"

        << std::flush;
}

// -- Print the fixed header (called once at startup) ---------------------------
static void print_fixed_header() {
    if (g_ansi) std::cout << "\033[2J\033[H";

    std::cout
        << BOLD << CYAN
        << " "
        << UTF8_BOX_TL;

    for (int i = 0; i < 46; ++i)
        std::cout << UTF8_BOX_H;

    std::cout
        << UTF8_BOX_TR << "\n"
        << " "
        << UTF8_BOX_V
        << "     Hydraulic Cam Motor Optimizer v2.0       "
        << UTF8_BOX_V
        << "\n"
        << " "
        << UTF8_BOX_BL;

    for (int i = 0; i < 46; ++i)
        std::cout << UTF8_BOX_H;

    std::cout
        << UTF8_BOX_BR
        << "\n"
        << RST;

    std::cout
        << DIM
        << " Pistons: " << config::N_PISTONS
        << " Lobes: " << config::N_LOBES
        << " Speed: " << config::OMEGA << " rad/s"
        << " P_sup: " << config::P_SUP / 1e5 << " bar"
        << " Target: 2100 N" << UTF8_MID_DOT << "m\n"
        << RST
        << DIM << " ";

    for (int i = 0; i < 46; ++i) std::cout << '-';

    std::cout << RST << "\n";

    std::cout << make_progress_line(0, 1200, 0, 0) << "\n";
    std::cout << " " << DIM << "(waiting for first result...)" << RST << "\n";
    std::cout << DIM << " ";

    for (int i = 0; i < 46; ++i) std::cout << '-';

    std::cout << RST << "\n";

    // Lock rows 1-8 as the fixed region; everything from row 9 scrolls
    if (g_ansi)
        std::cout << "\033[" << SCROLL_START << ";r"
        << "\033[" << SCROLL_START << ";1H"
        << std::flush;
}

static void restore_terminal()
{
    if (!g_ansi)
        return;

    std::cout
        << "\033[r"
        << "\033[999;1H"
        << std::flush;
}

// -- Restore normal terminal state ---------------------------------------------
static void print_final_summary(const SimulationResult& res, const std::string& title)
{
    auto bar20 = [&](double frac, const char* col) -> std::string
        {
            int fill = std::clamp((int)(frac * 20), 0, 20);

            std::string b;
            b += col;
            b += "[";

            for (int i = 0; i < fill; ++i)
                b += "#";

            for (int i = fill; i < 20; ++i)
                b += ".";

            b += "]";
            b += RST;

            return b;
        };

    double pmax = res.max_pressure;
    double pmin = res.min_pressure;

    if (pmin > pmax)
        std::swap(pmax, pmin);

    const char* tc =
        (std::abs(res.mean_torque - 2100) < 50) ? GREEN :
        (std::abs(res.mean_torque - 2100) < 200) ? YEL :
        RED;

    const char* rc =
        (res.torque_ripple_pct < 25) ? GREEN :
        (res.torque_ripple_pct < 60) ? YEL :
        RED;

    const char* vc =
        (res.volumetric_eff > 0.97) ? GREEN :
        (res.volumetric_eff > 0.90) ? YEL :
        RED;

    std::cout
        << "\n"
        << BOLD
        << CYAN
        << " "
        << UTF8_BOX_TL;

    for (int i = 0; i < 49; ++i)
        std::cout << UTF8_BOX_H;

    std::cout
        << UTF8_BOX_TR
        << "\n"
        << " "
        << UTF8_BOX_V
        << " "
        << WHT
        << std::left
        << std::setw(44)
        << title
        << CYAN
        << UTF8_BOX_V
        << "\n"
        << " "
        << UTF8_BOX_BL;

    for (int i = 0; i < 49; ++i)
        std::cout << UTF8_BOX_H;

    std::cout
        << UTF8_BOX_BR
        << RST
        << "\n\n"

        << " "
        << DIM
        << std::setw(22)
        << std::left
        << "Mean torque"
        << RST
        << tc
        << BOLD
        << std::right
        << std::setw(10)
        << std::fixed
        << std::setprecision(1)
        << res.mean_torque
        << " N"
        << UTF8_MID_DOT
        << "m"
        << RST
        << DIM
        << " [target 2100 N"
        << UTF8_MID_DOT
        << "m]\n"
        << RST

        << "  "
        << DIM
        << std::setw(22)
        << std::left
        << "Torque ripple"
        << RST
        << rc
        << std::right
        << std::setw(10)
        << std::setprecision(2)
        << res.torque_ripple_pct
        << " %  "
        << RST
        << bar20(res.torque_ripple_pct / 100.0, rc)
        << "\n"

        << "  "
        << DIM
        << std::setw(22)
        << std::left
        << "Volumetric eff."
        << RST
        << vc
        << std::right
        << std::setw(10)
        << std::setprecision(2)
        << res.volumetric_eff * 100
        << " %  "
        << RST
        << bar20(res.volumetric_eff, vc)
        << "\n"

        << DIM
        << "  ";

    for (int i = 0; i < 46; ++i)
        std::cout << '-';

    std::cout
        << RST
        << "\n"

        << "  "
        << DIM
        << std::setw(22)
        << std::left
        << "Pressure range"
        << RST
        << CYAN
        << std::right
        << std::setw(7)
        << std::setprecision(1)
        << pmin / 1e5
        << " "
        << UTF8_ARROW
        << " "
        << std::setw(6)
        << pmax / 1e5
        << " bar"
        << RST
        << "\n"

        << "  "
        << DIM
        << std::setw(22)
        << std::left
        << "Cavitation events"
        << RST
        << (res.cavitation_events ? RED : GREEN)
        << BOLD
        << std::right
        << std::setw(10)
        << res.cavitation_events
        << RST
        << "\n"

        << DIM
        << "  ";

    for (int i = 0; i < 46; ++i)
        std::cout << '-';

    std::cout
        << RST
        << "\n"

        << "  "
        << DIM
        << std::setw(22)
        << std::left
        << "Total energy"
        << RST
        << std::right
        << std::setw(10)
        << std::setprecision(1)
        << res.total_energy
        << " J\n"

        << "  "
        << DIM
        << std::setw(22)
        << std::left
        << "Power loss"
        << RST
        << std::right
        << std::setw(10)
        << std::setprecision(2)
        << res.p_loss
        << " W\n"

        << "\n"
        << std::flush;
}
// -- Monitor thread ------------------------------------------------------------
static void monitor_loop(const GPOptimizer* gp,
                          Snapshot*            snapshot,
                          const std::atomic<bool>* done,
                          int total_obs,
                          int poll_ms = 800)
{
    auto start = clk::now();
    int    last_obs  = -1;
    double last_best = std::numeric_limits<double>::infinity();

    while (!done->load(std::memory_order_acquire)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(poll_ms));

        long long el  = std::chrono::duration_cast<std::chrono::seconds>(
                            clk::now() - start).count();
        int    obs    = snapshot ? snapshot->observations.load() : 0;
        double best   = snapshot ? snapshot->best_cost.load()    : std::numeric_limits<double>::infinity();
        int    gp_n   = gp ? gp->num_observations() : 0;

        // Always refresh the progress bar row
        overwrite_row(ROW_PROGRESS, make_progress_line(obs, total_obs, el, gp_n));

        // When there's a new best: update summary row AND print a snapshot block
        bool new_best = (best < last_best);
        if (new_best && snapshot) {
            std::lock_guard<std::mutex> lk(snapshot->mtx);
            if (!snapshot->sim.time.empty()) {
                overwrite_row(ROW_SUMMARY,
                    make_summary_line(best, snapshot->sim));
                print_snapshot_block(obs, best,
                    snapshot->best_design, snapshot->sim);
            }
            last_best = best;
        }
        last_obs = obs;
    }

    // Final progress bar update
    long long el = std::chrono::duration_cast<std::chrono::seconds>(
                       clk::now() - start).count();
    int obs      = snapshot ? snapshot->observations.load() : 0;
    int gp_n     = gp ? gp->num_observations() : 0;
    overwrite_row(ROW_PROGRESS, make_progress_line(obs, total_obs, el, gp_n));

    std::string done_line = std::string("  ") + BOLD + GREEN
        + (g_ansi ? "*" : "*") + " Complete ? "
        + std::to_string(obs) + " evaluations   elapsed " + fmt_dur(el) + RST;
    overwrite_row(ROW_SUMMARY, done_line);
}

// -- Final eval + export -------------------------------------------------------
static void run_final_eval(const Design& d) {
    BSpline cam = cam_utils::build_cam_from_design(d);
    HydraulicModel model(cam, d.valve_shift, d.valve_width, d.valve_overlap);
    const int SPR = static_cast<int>(std::ceil(2.0*M_PI/(config::OMEGA*config::DT)));
    auto res = model.run(3*SPR, config::DT);
    print_final_summary(res, "     Final Results - 3 Full Revolutions     ");
    dxf_export::export_cam(d);
    BSpline cam_for_plot = cam_utils::build_cam_from_design(d);
    plot_svg::plot_all(res, cam_for_plot);
}

// -- main ---------------------------------------------------------------------
int main() {
    init_ansi();
    print_fixed_header();

    const int TOTAL_OBS  = 1200;
    const double uni_inc = config::STROKE / (double)(config::N_CTRL / 2);

    std::vector<double> x(config::N_CTRL + 3, 0.0);
    for (int i = 0; i < config::N_CTRL; ++i)
        x[i] = uni_inc * common::rand_range(0.8, 1.2);
    x[config::N_CTRL + 0] = 0.0;
    x[config::N_CTRL + 1] = 0.5;
    x[config::N_CTRL + 2] = config::VALVE_OVERLAP_DEFAULT;
    constraints::project(x);

    Snapshot          snapshot;
    std::atomic<bool> done(false);

    if constexpr (config::RUN_HYBRID) {
        GPOptimizer  gp(1.0, 1.0, 1e-3);
        HybridConfig cfg;
        cfg.n_iterations       = 60;
        cfg.n_candidates       = 400;
        cfg.acq_kappa          = 2.0;
        cfg.local_refine_steps = 20;

        std::thread mon(&monitor_loop, &gp, &snapshot, &done, TOTAL_OBS, 800);
        auto best = run_hybrid_optimisation(gp, x, cfg, &snapshot);
        done.store(true, std::memory_order_release);
        if (mon.joinable()) mon.join();

        // Plot convergence history before final eval
        plot_svg::plot_convergence(gp.convergence_history(), "plot_convergence.svg");

        restore_terminal();
        run_final_eval(design_vector::unpack(best));
    } else {
        const int SPR = static_cast<int>(std::ceil(2.0*M_PI/(config::OMEGA*config::DT)));
        double best_cost = std::numeric_limits<double>::infinity();
        auto   best_x    = x;

        std::thread mon([&]{ monitor_loop(nullptr,&snapshot,&done,TOTAL_OBS,800); });

        for (int iter = 0; iter < TOTAL_OBS; ++iter) {
            std::vector<double> cand = best_x;
            for (int k=0;k<4;++k) {
                int idx = (int)common::rand_range(0, (double)(config::N_CTRL-1));
                cand[idx] += common::rand_normal() * uni_inc * 0.15;
            }
            cand[config::N_CTRL+0] += common::rand_range(-0.05,  0.05);
            cand[config::N_CTRL+1] += common::rand_range(-0.02,  0.02);
            cand[config::N_CTRL+2] += common::rand_range(-0.005, 0.005);
            constraints::project(cand);

            double c = evaluate_objective(cand);
            snapshot.observations.fetch_add(1);

            if (c < snapshot.best_cost.load()) {
                snapshot.best_cost.store(c);
                std::lock_guard<std::mutex> lk(snapshot.mtx);
                snapshot.best_design = design_vector::unpack(cand);
                BSpline cam = cam_utils::build_cam_from_design(snapshot.best_design);
                HydraulicModel model(cam, snapshot.best_design.valve_shift,
                                         snapshot.best_design.valve_width,
                                         snapshot.best_design.valve_overlap);
                snapshot.sim = model.run(SPR, config::DT);
                snapshot.last_update = clk::now();
            }
            if (c < best_cost) { best_cost = c; best_x = cand; }
        }

        done.store(true, std::memory_order_release);
        if (mon.joinable()) mon.join();

        restore_terminal();
        run_final_eval(design_vector::unpack(best_x));
    }

    return 0;
}
