#include "plotting.h"
#include "config.h"
#include <fstream>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <numeric>
#include <cmath>
#include <filesystem>
#include <iostream>

// ???????????????????????????????????????????????????????????????????????????????
// MATLAB default colour palette (7 colours, cycled)
// ???????????????????????????????????????????????????????????????????????????????
static const char* MATLAB_COLORS[] = {
    "#0072BD",  // C0 blue
    "#D95319",  // C1 orange
    "#EDB120",  // C2 yellow
    "#7E2F8E",  // C3 purple
    "#77AC30",  // C4 green
    "#4DBEEE",  // C5 light blue
    "#A2142F"   // C6 dark red
};
static constexpr int N_COLORS = 7;
static const char* mc(int i) { return MATLAB_COLORS[i % N_COLORS]; }

// ???????????????????????????????????????????????????????????????????????????????
// Axis computation ? "nice numbers" matching MATLAB's auto-scale
// ???????????????????????????????????????????????????????????????????????????????
struct AxisInfo {
    double min, max, step;
    std::vector<double> ticks;
};

static AxisInfo nice_axis(double data_min, double data_max, int target = 6) {
    double range = data_max - data_min;
    if (range < 1e-12) { range = 1.0; data_min -= 0.5; data_max += 0.5; }
    double raw   = range / target;
    double mag   = std::pow(10.0, std::floor(std::log10(raw)));
    double norm  = raw / mag;
    double step;
    if      (norm < 1.5) step = 1.0  * mag;
    else if (norm < 3.5) step = 2.0  * mag;
    else if (norm < 7.5) step = 5.0  * mag;
    else                 step = 10.0 * mag;

    AxisInfo ax;
    ax.step = step;
    ax.min  = std::floor(data_min / step) * step;
    ax.max  = std::ceil (data_max / step) * step;
    for (double v = ax.min; v <= ax.max + step * 0.01; v += step)
        ax.ticks.push_back(std::round(v / (step * 1e-9)) * (step * 1e-9));
    return ax;
}

// ???????????????????????????????????????????????????????????????????????????????
// Tick label formatter ? matches MATLAB's minimal decimal places
// ???????????????????????????????????????????????????????????????????????????????
static std::string fmt_tick(double v, double step) {
    if (step == 0) return "0";
    int decimals = std::max(0, (int)std::ceil(-std::log10(step)) + 1);
    decimals = std::min(decimals, 4);
    std::ostringstream s;
    s << std::fixed << std::setprecision(decimals) << v;
    // strip trailing zeros after decimal point
    std::string r = s.str();
    if (r.find('.') != std::string::npos) {
        r.erase(r.find_last_not_of('0') + 1);
        if (r.back() == '.') r.pop_back();
    }
    return r;
}

// ???????????????????????????????????????????????????????????????????????????????
// SVGCanvas ? the core rendering engine
// ???????????????????????????????????????????????????????????????????????????????
struct Layout {
    int W = 960, H = 580;
    int ml = 88, mr = 32, mt = 55, mb = 68;
    int pw() const { return W - ml - mr; }  // plot width
    int ph() const { return H - mt - mb; }  // plot height
    int x0() const { return ml; }           // plot left edge
    int y0() const { return mt; }           // plot top  edge
    int x1() const { return ml + pw(); }    // plot right edge
    int y1() const { return mt + ph(); }    // plot bottom edge
};

class SVGCanvas {
public:
    Layout L;
    AxisInfo ax, ay;
    std::ostringstream buf;

    SVGCanvas(int W, int H) { L.W=W; L.H=H; }

    // ?? coordinate transform ?????????????????????????????????????????????????
    double tx(double x) const {
        return L.x0() + (x - ax.min) / (ax.max - ax.min) * L.pw();
    }
    double ty(double y) const {
        return L.y1() - (y - ay.min) / (ay.max - ay.min) * L.ph();
    }
    // clamp to plot area
    bool in_range(double x, double y) const {
        return x >= ax.min - 1e-9 && x <= ax.max + 1e-9
            && y >= ay.min - 1e-9 && y <= ay.max + 1e-9;
    }

    // ?? header / background ??????????????????????????????????????????????????
    void begin(const std::string& title,
               const std::string& xlabel, const std::string& ylabel) {
        buf << "<?xml version='1.0' encoding='UTF-8'?>\n"
            << "<svg xmlns='http://www.w3.org/2000/svg'"
            << " width='" << L.W << "' height='" << L.H << "'>\n"
            << "<defs>"
            << "<clipPath id='plotclip'>"
            << "<rect x='" << L.x0() << "' y='" << L.y0()
            << "' width='" << L.pw() << "' height='" << L.ph() << "'/>"
            << "</clipPath></defs>\n";

        // Outer background
        buf << "<rect width='" << L.W << "' height='" << L.H
            << "' fill='#ffffff' stroke='none'/>\n";
        // Plot area background (MATLAB light-gray axes face)
        buf << "<rect x='" << L.x0() << "' y='" << L.y0()
            << "' width='" << L.pw() << "' height='" << L.ph()
            << "' fill='#f9f9f9' stroke='none'/>\n";

        // Title
        buf << "<text x='" << (L.W/2) << "' y='" << (L.mt/2 + 4)
            << "' text-anchor='middle' font-family='Arial,Helvetica,sans-serif'"
            << " font-size='15' font-weight='bold' fill='#222222'>"
            << title << "</text>\n";

        // X label
        buf << "<text x='" << (L.x0() + L.pw()/2) << "' y='" << (L.H - 8)
            << "' text-anchor='middle' font-family='Arial,Helvetica,sans-serif'"
            << " font-size='13' fill='#333333'>" << xlabel << "</text>\n";

        // Y label (rotated)
        buf << "<text transform='translate(" << (L.ml - 58) << ","
            << (L.y0() + L.ph()/2) << ") rotate(-90)'"
            << " text-anchor='middle' font-family='Arial,Helvetica,sans-serif'"
            << " font-size='13' fill='#333333'>" << ylabel << "</text>\n";
    }

    // ?? grid + axes ??????????????????????????????????????????????????????????
    void draw_grid() {
        // Major grid lines
        for (double v : ax.ticks) {
            double px = tx(v);
            if (px < L.x0()-0.5 || px > L.x1()+0.5) continue;
            buf << "<line x1='" << px << "' y1='" << L.y0()
                << "' x2='" << px << "' y2='" << L.y1()
                << "' stroke='#d8d8d8' stroke-width='0.8'/>\n";
        }
        for (double v : ay.ticks) {
            double py = ty(v);
            if (py < L.y0()-0.5 || py > L.y1()+0.5) continue;
            buf << "<line x1='" << L.x0() << "' y1='" << py
                << "' x2='" << L.x1() << "' y2='" << py
                << "' stroke='#d8d8d8' stroke-width='0.8'/>\n";
        }

        // Axes box
        buf << "<rect x='" << L.x0() << "' y='" << L.y0()
            << "' width='" << L.pw() << "' height='" << L.ph()
            << "' fill='none' stroke='#555555' stroke-width='1.2'/>\n";

        // X ticks + labels
        for (double v : ax.ticks) {
            double px = tx(v);
            if (px < L.x0()-0.5 || px > L.x1()+0.5) continue;
            buf << "<line x1='" << px << "' y1='" << L.y1()
                << "' x2='" << px << "' y2='" << (L.y1()+5)
                << "' stroke='#555555' stroke-width='1'/>\n";
            buf << "<text x='" << px << "' y='" << (L.y1()+17)
                << "' text-anchor='middle' font-family='Arial,sans-serif'"
                << " font-size='11' fill='#444444'>"
                << fmt_tick(v, ax.step) << "</text>\n";
        }

        // Y ticks + labels
        for (double v : ay.ticks) {
            double py = ty(v);
            if (py < L.y0()-0.5 || py > L.y1()+0.5) continue;
            buf << "<line x1='" << (L.x0()-5) << "' y1='" << py
                << "' x2='" << L.x0() << "' y2='" << py
                << "' stroke='#555555' stroke-width='1'/>\n";
            buf << "<text x='" << (L.x0()-9) << "' y='" << (py+4)
                << "' text-anchor='end' font-family='Arial,sans-serif'"
                << " font-size='11' fill='#444444'>"
                << fmt_tick(v, ay.step) << "</text>\n";
        }
    }

    // ?? data path ????????????????????????????????????????????????????????????
    // Builds a clipped <path> from (xs[i], ys[i]) pairs.
    void draw_line(const std::vector<double>& xs,
                   const std::vector<double>& ys,
                   const std::string& color,
                   double width = 1.8,
                   const std::string& dash = "") {
        if (xs.empty()) return;
        std::ostringstream path;
        bool pen_down = false;
        for (size_t i = 0; i < xs.size(); ++i) {
            double px = tx(xs[i]);
            double py = ty(ys[i]);
            // clamp to ?2px outside plot area for clean border clipping
            px = std::clamp(px, (double)L.x0()-2, (double)L.x1()+2);
            py = std::clamp(py, (double)L.y0()-2, (double)L.y1()+2);
            if (!pen_down) { path << "M " << px << "," << py; pen_down=true; }
            else           { path << " L " << px << "," << py; }
        }
        buf << "<path clip-path='url(#plotclip)' d='" << path.str()
            << "' fill='none' stroke='" << color
            << "' stroke-width='" << width << "'";
        if (!dash.empty()) buf << " stroke-dasharray='" << dash << "'";
        buf << "/>\n";
    }

    // Filled area between two y-series (for ripple band)
    void draw_fill_between(const std::vector<double>& xs,
                            const std::vector<double>& y_lo,
                            const std::vector<double>& y_hi,
                            const std::string& color,
                            double alpha = 0.15) {
        if (xs.empty()) return;
        std::ostringstream path;
        // forward pass (y_hi)
        for (size_t i = 0; i < xs.size(); ++i) {
            double px = tx(xs[i]);
            double py = std::clamp(ty(y_hi[i]), (double)L.y0(), (double)L.y1());
            if (i == 0) path << "M " << px << "," << py;
            else        path << " L " << px << "," << py;
        }
        // reverse pass (y_lo) to close
        for (int i = (int)xs.size()-1; i >= 0; --i) {
            double px = tx(xs[i]);
            double py = std::clamp(ty(y_lo[i]), (double)L.y0(), (double)L.y1());
            path << " L " << px << "," << py;
        }
        path << " Z";
        buf << "<path clip-path='url(#plotclip)' d='" << path.str()
            << "' fill='" << color << "' fill-opacity='" << alpha
            << "' stroke='none'/>\n";
    }

    // Vertical marker line (annotation)
    void draw_vline(double x_val, const std::string& color,
                     const std::string& dash = "4,3", double width = 1.2) {
        if (x_val < ax.min || x_val > ax.max) return;
        double px = tx(x_val);
        buf << "<line clip-path='url(#plotclip)' x1='" << px << "' y1='" << L.y0()
            << "' x2='" << px << "' y2='" << L.y1()
            << "' stroke='" << color << "' stroke-width='" << width
            << "' stroke-dasharray='" << dash << "'/>\n";
    }

    // Horizontal marker line
    void draw_hline(double y_val, const std::string& color,
                     const std::string& dash = "6,3", double width = 1.5) {
        if (y_val < ay.min || y_val > ay.max) return;
        double py = ty(y_val);
        buf << "<line clip-path='url(#plotclip)' x1='" << L.x0() << "' y1='" << py
            << "' x2='" << L.x1() << "' y2='" << py
            << "' stroke='" << color << "' stroke-width='" << width
            << "' stroke-dasharray='" << dash << "'/>\n";
    }

    // Bar chart (for FFT)
    void draw_bar(double x_center, double y_val, double bar_w,
                   const std::string& color) {
        double px   = tx(x_center);
        double py   = ty(y_val);
        double py0  = ty(ay.min);
        double bwpx = bar_w / (ax.max - ax.min) * L.pw();
        bwpx = std::max(bwpx, 1.5);
        double h = py0 - py;
        if (h <= 0) return;
        buf << "<rect clip-path='url(#plotclip)'"
            << " x='" << (px - bwpx/2) << "' y='" << py
            << "' width='" << bwpx << "' height='" << h
            << "' fill='" << color << "' fill-opacity='0.85'"
            << " stroke='none'/>\n";
    }

    // Legend box
    void draw_legend(const std::vector<std::string>& labels,
                      const std::vector<std::string>& colors,
                      const std::vector<std::string>& dashes = {}) {
        if (labels.empty()) return;
        int n   = (int)labels.size();
        int lw  = 120, lh = n * 18 + 10;
        int lx  = L.x1() - lw - 10;
        int ly  = L.y0() + 10;
        buf << "<rect x='" << lx << "' y='" << ly
            << "' width='" << lw << "' height='" << lh
            << "' fill='#ffffffcc' stroke='#aaaaaa' stroke-width='0.8'"
            << " rx='3'/>\n";
        for (int i = 0; i < n; ++i) {
            int row_y = ly + 10 + i * 18;
            std::string dash = (i < (int)dashes.size()) ? dashes[i] : "";
            buf << "<line x1='" << (lx+8) << "' y1='" << (row_y+4)
                << "' x2='" << (lx+28) << "' y2='" << (row_y+4)
                << "' stroke='" << colors[i] << "' stroke-width='2'";
            if (!dash.empty()) buf << " stroke-dasharray='" << dash << "'";
            buf << "/>\n";
            buf << "<text x='" << (lx+34) << "' y='" << (row_y+8)
                << "' font-family='Arial,sans-serif' font-size='11' fill='#333333'>"
                << labels[i] << "</text>\n";
        }
    }

    // Annotation text inside plot
    void annotate(double x, double y, const std::string& text,
                   const std::string& color = "#333333",
                   const std::string& anchor = "start") {
        buf << "<text x='" << tx(x) << "' y='" << (ty(y)-4)
            << "' text-anchor='" << anchor
            << "' font-family='Arial,sans-serif' font-size='10'"
            << " fill='" << color << "'>" << text << "</text>\n";
    }

    // ?? footer ???????????????????????????????????????????????????????????????
    std::string end() {
        buf << "</svg>\n";
        return buf.str();
    }

    bool save(const std::string& path) {
        std::ofstream f(path);
        if (!f) { std::cerr << "Cannot write " << path << "\n"; return false; }
        f << end();
        return true;
    }
};

// ???????????????????????????????????????????????????????????????????????????????
// DFT (harmonic analysis of torque signal)
// Computes the one-sided magnitude spectrum for harmonics 0..max_k.
// ???????????????????????????????????????????????????????????????????????????????
static std::vector<double> compute_dft(const std::vector<double>& sig, int max_k) {
    int n = (int)sig.size();
    std::vector<double> mags(max_k + 1, 0.0);
    for (int k = 0; k <= max_k; ++k) {
        double re = 0, im = 0;
        for (int j = 0; j < n; ++j) {
            double ang = 2.0 * M_PI * k * j / (double)n;
            re += sig[j] * std::cos(ang);
            im -= sig[j] * std::sin(ang);
        }
        mags[k] = (k == 0 ? 1.0 : 2.0) / n * std::sqrt(re*re + im*im);
    }
    return mags;
}

// ???????????????????????????????????????????????????????????????????????????????
// Extract one clean revolution from a multi-revolution simulation
// (use the last revolution to avoid startup transients)
// ???????????????????????????????????????????????????????????????????????????????
static int steps_per_rev() {
    return (int)std::ceil(2.0 * M_PI / (config::OMEGA * config::DT));
}

static std::vector<double> last_rev(const std::vector<double>& v) {
    int spr = steps_per_rev();
    if ((int)v.size() <= spr) return v;
    return std::vector<double>(v.end() - spr, v.end());
}

// Convert raw theta (radians, multi-rev) to shaft angle in degrees [0, 360)
static std::vector<double> theta_to_deg(const std::vector<double>& theta) {
    std::vector<double> deg(theta.size());
    for (size_t i = 0; i < theta.size(); ++i)
        deg[i] = std::fmod(theta[i] * 180.0 / M_PI, 360.0);
    return deg;
}

// ???????????????????????????????????????????????????????????????????????????????
// namespace plot_svg implementations
// ???????????????????????????????????????????????????????????????????????????????
namespace plot_svg {

// ?? Generic plotter (backwards-compatible) ???????????????????????????????????
void plot_svg(const std::string& filename, const std::string& title,
              const std::string& xlabel, const std::string& ylabel,
              const std::vector<PlotSeries>& series, int W, int H) {
    if (series.empty()) return;
    SVGCanvas c(W, H);
    double xmin= 1e18,xmax=-1e18,ymin=1e18,ymax=-1e18;
    for (auto& s : series) {
        for (double v : s.x) { xmin=std::min(xmin,v); xmax=std::max(xmax,v); }
        for (double v : s.y) { ymin=std::min(ymin,v); ymax=std::max(ymax,v); }
    }
    c.ax = nice_axis(xmin,xmax); c.ay = nice_axis(ymin,ymax);
    c.begin(title,xlabel,ylabel); c.draw_grid();
    for (size_t i=0;i<series.size();++i)
        c.draw_line(series[i].x, series[i].y,
                    series[i].color.empty() ? mc(i) : series[i].color);
    c.save(filename);
}

// ?? 1. Torque vs Shaft Angle ?????????????????????????????????????????????????
void plot_torque_vs_angle(const SimulationResult& sim, const std::string& fn) {
    auto t_rev   = last_rev(sim.total_torque);
    auto th_rev  = last_rev(sim.theta);
    auto deg     = theta_to_deg(th_rev);
    // Build monotone angle axis (0?360) for clean display
    for (auto& d : deg) if (d < deg[0]-1.0) d += 360.0;

    double mean_T = sim.mean_torque;
    double tmax = *std::max_element(t_rev.begin(),t_rev.end());
    double tmin = *std::min_element(t_rev.begin(),t_rev.end());

    SVGCanvas c(960, 580);
    c.ax = nice_axis(deg.front(), deg.back());
    // y range: give 8% headroom above/below
    double ymarg = (tmax-tmin)*0.08 + 50;
    c.ay = nice_axis(tmin - ymarg, tmax + ymarg);

    c.begin("Output Torque vs Shaft Angle", "Shaft Angle (deg)", "Torque (N?m)");
    c.draw_grid();

    // Ripple band (shaded between mean?half_ripple)
    double half = (tmax - tmin) * 0.5;
    std::vector<double> y_lo(deg.size(), mean_T - half);
    std::vector<double> y_hi(deg.size(), mean_T + half);
    c.draw_fill_between(deg, y_lo, y_hi, MATLAB_COLORS[0], 0.12);

    // Target 2100 N?m
    c.draw_hline(2100.0, "#999999", "4,3", 1.2);
    // Mean torque
    c.draw_hline(mean_T, MATLAB_COLORS[1], "8,4", 2.0);
    // Torque signal
    c.draw_line(deg, t_rev, MATLAB_COLORS[0], 1.8);

    // Annotations
    std::ostringstream ann;
    ann << std::fixed << std::setprecision(1) << mean_T << " N?m mean";
    c.annotate(deg.front() + 3, mean_T, ann.str(), MATLAB_COLORS[1]);
    double rip_pct = (std::abs(mean_T)>1e-3) ? (tmax-tmin)/std::abs(mean_T)*100 : 0;
    std::ostringstream rann;
    rann << "Ripple: " << std::fixed << std::setprecision(1) << rip_pct << "%";
    c.annotate(deg.back()*0.72, tmax, rann.str(), "#555555");

    c.draw_legend({"Torque","Mean torque","Target 2100 N?m"},
                  {MATLAB_COLORS[0],MATLAB_COLORS[1],"#999999"},
                  {"","8,4","4,3"});
    c.save(fn);
}

// ?? 2. Hydraulic Power vs Shaft Angle ????????????????????????????????????????
void plot_power_vs_angle(const SimulationResult& sim, const std::string& fn) {
    auto t_rev  = last_rev(sim.total_torque);
    auto th_rev = last_rev(sim.theta);
    auto deg    = theta_to_deg(th_rev);
    for (auto& d : deg) if (d < deg[0]-1.0) d += 360.0;

    std::vector<double> kW(t_rev.size());
    for (size_t i=0;i<t_rev.size();++i)
        kW[i] = t_rev[i] * config::OMEGA / 1000.0;
    double mean_kW = sim.mean_torque * config::OMEGA / 1000.0;
    double kmax = *std::max_element(kW.begin(),kW.end());
    double kmin = *std::min_element(kW.begin(),kW.end());

    SVGCanvas c(960,580);
    c.ax = nice_axis(deg.front(), deg.back());
    c.ay = nice_axis(kmin - std::abs(kmax-kmin)*0.08,
                      kmax + std::abs(kmax-kmin)*0.08);
    c.begin("Hydraulic Power vs Shaft Angle","Shaft Angle (deg)","Power (kW)");
    c.draw_grid();
    c.draw_hline(mean_kW, MATLAB_COLORS[1], "8,4", 2.0);
    c.draw_line(deg, kW, MATLAB_COLORS[2], 1.8);
    std::ostringstream a;
    a << std::fixed<<std::setprecision(1)<<mean_kW<<" kW mean";
    c.annotate(deg.front()+3, mean_kW, a.str(), MATLAB_COLORS[1]);
    c.draw_legend({"Power","Mean power"},{MATLAB_COLORS[2],MATLAB_COLORS[1]},{"","8,4"});
    c.save(fn);
}

// ?? 3. Flow Rate vs Shaft Angle ??????????????????????????????????????????????
void plot_flow_vs_angle(const SimulationResult& sim, const std::string& fn) {
    auto q_rev  = last_rev(sim.total_flow);
    auto th_rev = last_rev(sim.theta);
    auto deg    = theta_to_deg(th_rev);
    for (auto& d : deg) if (d < deg[0]-1.0) d += 360.0;

    // Convert m?/s ? L/min
    std::vector<double> Lmin(q_rev.size());
    for (size_t i=0;i<q_rev.size();++i) Lmin[i] = q_rev[i]*60000.0;
    double qmax=*std::max_element(Lmin.begin(),Lmin.end());
    double qmin=*std::min_element(Lmin.begin(),Lmin.end());
    double qmean = std::accumulate(Lmin.begin(),Lmin.end(),0.0)/Lmin.size();

    SVGCanvas c(960,580);
    c.ax = nice_axis(deg.front(), deg.back());
    double qr = (qmax-qmin)*0.1+1;
    c.ay = nice_axis(qmin-qr, qmax+qr);
    c.begin("Volumetric Flow Rate vs Shaft Angle","Shaft Angle (deg)","Flow Rate (L/min)");
    c.draw_grid();
    c.draw_hline(qmean, MATLAB_COLORS[1], "8,4", 2.0);
    c.draw_line(deg, Lmin, MATLAB_COLORS[5], 1.8);
    std::ostringstream a; a<<std::fixed<<std::setprecision(1)<<qmean<<" L/min mean";
    c.annotate(deg.front()+3, qmean, a.str(), MATLAB_COLORS[1]);
    c.draw_legend({"Flow rate","Mean flow"},{MATLAB_COLORS[5],MATLAB_COLORS[1]},{"","8,4"});
    c.save(fn);
}

// ?? 4. Chamber Pressure vs Shaft Angle ??????????????????????????????????????
void plot_pressure_vs_angle(const SimulationResult& sim, const std::string& fn) {
    if (sim.piston_pressure.empty()) {
        // Fallback: show max/min envelope
        auto pmax_rev = last_rev(sim.theta);
        std::vector<double> pmx(pmax_rev.size(), sim.max_pressure/1e5);
        std::vector<double> deg = theta_to_deg(pmax_rev);
        for (auto& d : deg) if (d < deg[0]-1.0) d += 360.0;
        SVGCanvas c(960,580);
        c.ax = nice_axis(deg.front(),deg.back());
        c.ay = nice_axis(0, sim.max_pressure/1e5 * 1.1);
        c.begin("Pressure (no per-piston data)","Shaft Angle (deg)","Pressure (bar)");
        c.draw_grid();
        c.draw_hline(config::P_SUP/1e5,  "#888888","4,2",1.0);
        c.draw_hline(config::P_TANK/1e5, "#888888","4,2",1.0);
        c.save(fn);
        return;
    }

    int n_pistons = (int)sim.piston_pressure.size();
    auto th_rev   = last_rev(sim.theta);
    auto deg      = theta_to_deg(th_rev);
    for (auto& d : deg) if (d < deg[0]-1.0) d += 360.0;
    int spr       = steps_per_rev();

    double pmin_all = config::P_TANK/1e5 * 0.9;
    double pmax_all = config::P_SUP/1e5  * 1.05;

    SVGCanvas c(960,580);
    c.ax = nice_axis(deg.front(), deg.back());
    c.ay = nice_axis(pmin_all, pmax_all);
    c.begin("Chamber Pressures vs Shaft Angle","Shaft Angle (deg)","Pressure (bar)");
    c.draw_grid();

    // Supply and tank reference lines
    c.draw_hline(config::P_SUP/1e5,  "#aaaaaa","3,2",1.0);
    c.draw_hline(config::P_TANK/1e5, "#aaaaaa","3,2",1.0);

    std::vector<std::string> labels, colors;
    for (int p = 0; p < n_pistons; ++p) {
        const auto& pp = sim.piston_pressure[p];
        std::vector<double> pbar(std::min((int)pp.size(), spr));
        int offset = std::max(0, (int)pp.size() - spr);
        for (int i = 0; i < (int)pbar.size(); ++i)
            pbar[i] = pp[offset + i] / 1e5;
        c.draw_line(deg, pbar, mc(p), 1.4);
        labels.push_back("P" + std::to_string(p+1));
        colors.push_back(mc(p));
    }
    c.draw_legend(labels, colors);
    c.save(fn);
}

// ?? 5. FFT Spectrum ??????????????????????????????????????????????????????????
void plot_fft_spectrum(const SimulationResult& sim, const std::string& fn) {
    auto t_rev = last_rev(sim.total_torque);
    int max_k  = 2 * config::N_PISTONS * config::N_LOBES + 4; // 88
    auto mags  = compute_dft(t_rev, max_k);

    SVGCanvas c(960,580);
    // x: harmonic 0..max_k, y: amplitude N?m
    c.ax = nice_axis(0, max_k);
    double ymax = *std::max_element(mags.begin()+1, mags.end()); // skip DC
    c.ay = nice_axis(0, ymax * 1.12);
    c.begin("Torque FFT Spectrum","Harmonic (? " +
            std::to_string((int)std::round(config::OMEGA/(2.0*M_PI))) + " Hz)",
            "Amplitude (N?m)");
    c.draw_grid();

    // Annotate key harmonics with vertical dashed lines
    auto vmark = [&](int k, const std::string& label, const std::string& col){
        if (k <= max_k) {
            c.draw_vline((double)k, col, "4,3", 1.3);
            // label above plot area
            double px = c.tx((double)k);
            c.buf << "<text x='" << px << "' y='" << (c.L.y0()-5)
                  << "' text-anchor='middle' font-family='Arial,sans-serif'"
                  << " font-size='10' fill='" << col << "'>" << label << "</text>\n";
        }
    };
    vmark(config::N_LOBES,   "N_L",    "#D95319");
    vmark(config::N_PISTONS, "N_P",    "#7E2F8E");
    vmark(config::N_PISTONS*config::N_LOBES, "N_P?N_L", "#A2142F");

    // Bars
    for (int k = 1; k <= max_k; ++k) {
        std::string col = "#0072BD";
        if (k == config::N_PISTONS*config::N_LOBES) col = "#A2142F";
        else if (k == config::N_LOBES)              col = "#D95319";
        else if (k == config::N_PISTONS)            col = "#7E2F8E";
        c.draw_bar((double)k, mags[k], 0.7, col);
    }
    c.save(fn);
}

// ?? 6. Cam Lift Profile ??????????????????????????????????????????????????????
void plot_cam_profile(const BSpline& cam, const std::string& fn) {
    const int N = 720;
    std::vector<double> ang(N), lift(N);
    double peak = 0;
    for (int i=0;i<N;++i) {
        ang[i]  = 360.0 * i / N;
        double theta_e = 2.0*M_PI*i/N;
        lift[i] = cam.evaluate(theta_e) * 1000.0; // mm
        peak = std::max(peak, lift[i]);
    }

    SVGCanvas c(960,580);
    c.ax = nice_axis(0, 360);
    c.ay = nice_axis(0, peak * 1.12);
    c.begin("Cam Lift Profile (Single Lobe)","Lobe Angle (deg)","Lift (mm)");
    c.draw_grid();

    // Base circle reference
    c.draw_hline(0, "#aaaaaa","2,2",0.8);
    // Peak lift line
    c.draw_hline(peak, "#D95319","6,3",1.2);
    // Profile
    c.draw_line(ang, lift, MATLAB_COLORS[0], 2.2);

    std::ostringstream a;
    a << "Peak " << std::fixed<<std::setprecision(2)<<peak<<" mm";
    c.annotate(185, peak, a.str(), MATLAB_COLORS[1], "middle");
    c.draw_legend({"Cam lift","Peak lift"},{MATLAB_COLORS[0],"#D95319"},{"","6,3"});
    c.save(fn);
}

// ?? 7. Oil Temperature vs Time ???????????????????????????????????????????????
void plot_temperature(const SimulationResult& sim, const std::string& fn) {
    if (sim.temperature.empty()) return;
    int n = (int)sim.temperature.size();
    std::vector<double> t_ms(n), T_C(n);
    for (int i=0;i<n;++i) {
        t_ms[i] = i * config::DT * 1000.0;
        T_C[i]  = sim.temperature[i] - 273.15;
    }
    double tmin = *std::min_element(T_C.begin(),T_C.end());
    double tmax = *std::max_element(T_C.begin(),T_C.end());

    SVGCanvas c(960,580);
    c.ax = nice_axis(t_ms.front(), t_ms.back());
    c.ay = nice_axis(tmin - 1.0, tmax + 1.0);
    c.begin("Oil Temperature vs Time","Time (ms)","Temperature (?C)");
    c.draw_grid();
    c.draw_hline(config::T_AMBIENT - 273.15, "#aaaaaa","4,2",0.8);
    c.draw_line(t_ms, T_C, MATLAB_COLORS[1], 2.0);
    std::ostringstream a;
    a << "Ambient " << std::fixed<<std::setprecision(0)<<(config::T_AMBIENT-273.15)<<" ?C";
    c.annotate(t_ms.front()+0.5, config::T_AMBIENT-273.15, a.str(), "#777777");
    c.draw_legend({"Oil temperature"},{MATLAB_COLORS[1]});
    c.save(fn);
}

// ?? CSV outputs ??????????????????????????????????????????????????????????????
void write_metrics_csv(const SimulationResult& sim, const std::string& fn) {
    std::ofstream f(fn);
    if (!f) return;
    f << "mean_torque,torque_ripple_pct,volumetric_eff,total_energy,"
      << "p_loss,thermal_power,max_pressure_bar,min_pressure_bar,cavitation_events\n"
      << sim.mean_torque     << ","
      << sim.torque_ripple_pct << ","
      << sim.volumetric_eff   << ","
      << sim.total_energy     << ","
      << sim.p_loss           << ","
      << sim.thermal_power    << ","
      << sim.max_pressure/1e5 << ","
      << sim.min_pressure/1e5 << ","
      << sim.cavitation_events << "\n";
}

void write_timeseries_csv(const SimulationResult& sim, const std::string& fn) {
    std::ofstream f(fn);
    if (!f) return;
    f << "step,theta_deg,torque_Nm,flow_Lmin,temperature_C\n";
    for (size_t i=0;i<sim.time.size();++i) {
        f << i << ","
          << std::fmod(sim.theta[i]*180.0/M_PI, 360.0) << ","
          << sim.total_torque[i] << ","
          << (sim.total_flow.size()>i ? sim.total_flow[i]*60000.0 : 0.0) << ","
          << (sim.temperature.size()>i ? sim.temperature[i]-273.15 : 0.0) << "\n";
    }
}

// ?? HTML Dashboard ???????????????????????????????????????????????????????????
std::string plot_all(const SimulationResult& sim, const BSpline& cam) {
    namespace fs = std::filesystem;
    auto cwd = fs::absolute(fs::current_path()).string();

    plot_torque_vs_angle   (sim,  "plot_torque.svg");
    plot_power_vs_angle    (sim,  "plot_power.svg");
    plot_flow_vs_angle     (sim,  "plot_flow.svg");
    plot_pressure_vs_angle (sim,  "plot_pressure.svg");
    plot_fft_spectrum      (sim,  "plot_fft.svg");
    plot_cam_profile       (cam,  "plot_cam.svg");
    plot_temperature       (sim,  "plot_temperature.svg");
    write_metrics_csv      (sim,  "metrics.csv");
    write_timeseries_csv   (sim,  "timeseries.csv");

    // HTML dashboard ? 2-column grid, opens directly in any browser
    static const char* PLOTS[][2] = {
        {"plot_torque.svg",      "Torque vs Angle"},
        {"plot_power.svg",       "Power vs Angle"},
        {"plot_flow.svg",        "Flow vs Angle"},
        {"plot_pressure.svg",    "Chamber Pressures"},
        {"plot_fft.svg",         "Torque FFT"},
        {"plot_cam.svg",         "Cam Profile"},
        {"plot_temperature.svg", "Temperature"},
        {"plot_convergence.svg", "GP Convergence"},
    };
    constexpr int N_PLOTS = 8;

    std::ofstream html("dashboard.html");
    html <<
R"(<!DOCTYPE html><html><head><meta charset='UTF-8'>
<title>Hydraulic Motor Optimizer ? Results</title>
<style>
  body{margin:0;background:#1a1a2e;font-family:Arial,sans-serif;color:#e0e0e0}
  h1{text-align:center;padding:20px 0 10px;font-size:22px;color:#7eb8f7;margin:0}
  .sub{text-align:center;font-size:13px;color:#888;margin-bottom:20px}
  .grid{display:grid;grid-template-columns:1fr 1fr;gap:16px;padding:0 20px 30px}
  .card{background:#0f3460;border-radius:8px;overflow:hidden;
        box-shadow:0 4px 12px rgba(0,0,0,0.5)}
  .card-title{font-size:12px;padding:8px 14px;background:#16213e;color:#7eb8f7;
              border-bottom:1px solid #0a3d62}
  .card img{width:100%;display:block}
  .wide{grid-column:1/-1}
  .footer{text-align:center;font-size:11px;color:#555;padding:10px;margin-bottom:10px}
</style></head><body>
<h1>&#9881; Hydraulic Cam Motor ? Optimisation Results</h1>
<div class='sub'>)" << config::N_PISTONS << " pistons &nbsp;?&nbsp; "
    << config::N_LOBES << " lobes &nbsp;?&nbsp; "
    << std::fixed << std::setprecision(1) << (config::OMEGA / (2*M_PI)*60) << " rpm &nbsp;?&nbsp; "
    << (int)(config::P_SUP/1e5) << " bar supply</div>\n"
    << "<div class='grid'>\n";

    for (int i=0;i<N_PLOTS;++i) {
        bool wide = (i == N_PLOTS-1 && N_PLOTS%2==1); // last odd card spans full
        html << "<div class='card" << (wide?" wide":"") << "'>\n"
             << "  <div class='card-title'>" << PLOTS[i][1] << "</div>\n"
             << "  <img src='" << PLOTS[i][0] << "' alt='" << PLOTS[i][1] << "'/>\n"
             << "</div>\n";
    }

    html << "</div>\n<div class='footer'>Generated by HydraulicMotorOptimizer &nbsp;?&nbsp; "
         << cwd << "</div>\n</body></html>\n";

    std::string dash = fs::absolute("dashboard.html").string();
    std::cout << "\n  Plots written to: " << cwd << "\n"
              << "  Open in browser:  " << dash << "\n\n";
    return dash;
}

} // namespace plot_svg

// ???????????????????????????????????????????????????????????????????????????????
// Convergence + Uncertainty plot
// ???????????????????????????????????????????????????????????????????????????????
#include "gp_optimizer.h"  // for ConvergencePoint

namespace plot_svg {

void plot_convergence(const std::vector<ConvergencePoint>& hist,
                      const std::string& fn) {
    if (hist.empty()) return;

    const int W = 960, H = 580;
    // Custom layout: right margin wider to fit second Y-axis
    struct Lay {
        int ml=88, mr=80, mt=55, mb=68;
        int W, H;
        Lay(int w,int h):W(w),H(h){}
        int pw()const{return W-ml-mr;}
        int ph()const{return H-mt-mb;}
        int x0()const{return ml;}
        int y0()const{return mt;}
        int x1()const{return ml+pw();}
        int y1()const{return mt+ph();}
    } L(W,H);

    int n = (int)hist.size();

    // Extract series
    std::vector<double> obs(n), cost_all(n), best(n), sigma(n);
    for (int i=0;i<n;++i) {
        obs[i]      = hist[i].observation;
        cost_all[i] = hist[i].cost;
        best[i]     = hist[i].best_so_far;
        sigma[i]    = hist[i].mean_uncertainty;
    }

    // Left axis: cost (log scale handled by log-transforming data)
    double bmax = *std::max_element(best.begin(),best.end());
    double bmin = *std::min_element(best.begin(),best.end());
    // scatter of all evaluations
    double cmax = *std::max_element(cost_all.begin(),cost_all.end());

    // Use log10 scale for cost axis
    auto safe_log = [](double v){ return std::log10(std::max(v,1.0)); };
    std::vector<double> log_cost(n), log_best(n);
    for (int i=0;i<n;++i) {
        log_cost[i] = safe_log(cost_all[i]);
        log_best[i] = safe_log(best[i]);
    }
    double lymin = safe_log(bmin*0.9);
    double lymax = safe_log(cmax*1.1);
    AxisInfo ax_left  = nice_axis(lymin, lymax, 6);
    AxisInfo ax_x     = nice_axis(1, n);

    // Right axis: uncertainty (linear)
    double smax = *std::max_element(sigma.begin(),sigma.end());
    double smin = *std::min_element(sigma.begin(),sigma.end());
    AxisInfo ax_right = nice_axis(0, smax*1.1, 5);

    // Map functions
    auto tx = [&](double x) -> double {
        return L.x0() + (x - ax_x.min)/(ax_x.max - ax_x.min) * L.pw();
    };
    auto ty_left = [&](double ly) -> double {
        return L.y1() - (ly - ax_left.min)/(ax_left.max - ax_left.min) * L.ph();
    };
    auto ty_right = [&](double s) -> double {
        return L.y1() - (s - ax_right.min)/(ax_right.max - ax_right.min) * L.ph();
    };

    std::ostringstream svg;
    svg << "<?xml version='1.0' encoding='UTF-8'?>\n"
        << "<svg xmlns='http://www.w3.org/2000/svg' width='" << W
        << "' height='" << H << "'>\n"
        << "<defs><clipPath id='cc'>"
        << "<rect x='" << L.x0() << "' y='" << L.y0()
        << "' width='" << L.pw() << "' height='" << L.ph() << "'/>"
        << "</clipPath></defs>\n"
        // Background
        << "<rect width='" << W << "' height='" << H << "' fill='#ffffff'/>\n"
        << "<rect x='" << L.x0() << "' y='" << L.y0()
        << "' width='" << L.pw() << "' height='" << L.ph()
        << "' fill='#f9f9f9' stroke='none'/>\n"
        // Title
        << "<text x='" << (W/2) << "' y='" << (L.mt/2+4)
        << "' text-anchor='middle' font-family='Arial,sans-serif'"
        << " font-size='15' font-weight='bold' fill='#222222'>"
        << "GP Convergence and Model Uncertainty</text>\n"
        // X label
        << "<text x='" << (L.x0()+L.pw()/2) << "' y='" << (H-8)
        << "' text-anchor='middle' font-family='Arial,sans-serif'"
        << " font-size='13' fill='#333333'>Observation Number</text>\n"
        // Left Y label
        << "<text transform='translate(" << (L.ml-58) << ","
        << (L.y0()+L.ph()/2) << ") rotate(-90)'"
        << " text-anchor='middle' font-family='Arial,sans-serif'"
        << " font-size='13' fill='#0072BD'>Cost (log" << (char)0x31 << "0)</text>\n"
        // Right Y label
        << "<text transform='translate(" << (W-18) << ","
        << (L.y0()+L.ph()/2) << ") rotate(90)'"
        << " text-anchor='middle' font-family='Arial,sans-serif'"
        << " font-size='13' fill='#D95319'>GP Uncertainty (cost units)</text>\n";

    // Grid (left axis)
    for (double v : ax_left.ticks) {
        double py = ty_left(v);
        if (py < L.y0()-0.5 || py > L.y1()+0.5) continue;
        svg << "<line x1='" << L.x0() << "' y1='" << py
            << "' x2='" << L.x1() << "' y2='" << py
            << "' stroke='#d8d8d8' stroke-width='0.8'/>\n";
    }
    for (double v : ax_x.ticks) {
        double px = tx(v);
        if (px < L.x0()-0.5 || px > L.x1()+0.5) continue;
        svg << "<line x1='" << px << "' y1='" << L.y0()
            << "' x2='" << px << "' y2='" << L.y1()
            << "' stroke='#d8d8d8' stroke-width='0.8'/>\n";
    }

    // Scatter: all cost evaluations (small dots)
    for (int i=0;i<n;++i) {
        double px = tx(obs[i]);
        double py = std::clamp(ty_left(log_cost[i]),
                               (double)L.y0(),(double)L.y1());
        svg << "<circle clip-path='url(#cc)' cx='" << px << "' cy='" << py
            << "' r='2.2' fill='#0072BD' fill-opacity='0.25' stroke='none'/>\n";
    }

    // Uncertainty band: sigma as filled area between 0 and sigma
    {
        std::ostringstream path;
        // top edge (sigma)
        for (int i=0;i<n;++i) {
            double px = tx(obs[i]);
            double py = std::clamp(ty_right(sigma[i]),(double)L.y0(),(double)L.y1());
            if (i==0) path<<"M "<<px<<","<<py;
            else      path<<" L "<<px<<","<<py;
        }
        // bottom edge (0)
        for (int i=n-1;i>=0;--i) {
            double px = tx(obs[i]);
            double py = ty_right(0);
            path<<" L "<<px<<","<<std::clamp(py,(double)L.y0(),(double)L.y1());
        }
        path<<" Z";
        svg << "<path clip-path='url(#cc)' d='" << path.str()
            << "' fill='#D95319' fill-opacity='0.15' stroke='none'/>\n";
    }

    // Uncertainty line
    {
        std::ostringstream path;
        for (int i=0;i<n;++i) {
            double px=tx(obs[i]);
            double py=std::clamp(ty_right(sigma[i]),(double)L.y0(),(double)L.y1());
            if (i==0) path<<"M "<<px<<","<<py;
            else      path<<" L "<<px<<","<<py;
        }
        svg << "<path clip-path='url(#cc)' d='" << path.str()
            << "' fill='none' stroke='#D95319' stroke-width='1.5'"
            << " stroke-dasharray='5,3'/>\n";
    }

    // Best-so-far line (thick, left axis, log scale)
    {
        std::ostringstream path;
        for (int i=0;i<n;++i) {
            double px=tx(obs[i]);
            double py=std::clamp(ty_left(log_best[i]),(double)L.y0(),(double)L.y1());
            if (i==0) path<<"M "<<px<<","<<py;
            else      path<<" L "<<px<<","<<py;
        }
        svg << "<path clip-path='url(#cc)' d='" << path.str()
            << "' fill='none' stroke='#0072BD' stroke-width='2.5'/>\n";
    }

    // Axes box
    svg << "<rect x='" << L.x0() << "' y='" << L.y0()
        << "' width='" << L.pw() << "' height='" << L.ph()
        << "' fill='none' stroke='#555555' stroke-width='1.2'/>\n";

    // Left axis ticks + labels (log scale: show as 10^x)
    for (double v : ax_left.ticks) {
        double py=ty_left(v);
        if (py<L.y0()-0.5||py>L.y1()+0.5) continue;
        svg << "<line x1='"<<(L.x0()-5)<<"' y1='"<<py
            <<"' x2='"<<L.x0()<<"' y2='"<<py
            <<"' stroke='#555555' stroke-width='1'/>\n";
        // Label: 10^v
        std::ostringstream lbl;
        int exp=(int)std::round(v);
        lbl<<"10"<<exp;
        svg << "<text x='"<<(L.x0()-9)<<"' y='"<<(py+4)
            <<"' text-anchor='end' font-family='Arial,sans-serif'"
            <<" font-size='11' fill='#0072BD'>"<<lbl.str()<<"</text>\n";
    }

    // Right axis ticks + labels
    for (double v : ax_right.ticks) {
        double py=ty_right(v);
        if (py<L.y0()-0.5||py>L.y1()+0.5) continue;
        svg << "<line x1='"<<L.x1()<<"' y1='"<<py
            <<"' x2='"<<(L.x1()+5)<<"' y2='"<<py
            <<"' stroke='#D95319' stroke-width='1'/>\n";
        svg << "<text x='"<<(L.x1()+9)<<"' y='"<<(py+4)
            <<"' text-anchor='start' font-family='Arial,sans-serif'"
            <<" font-size='11' fill='#D95319'>"
            <<fmt_tick(v,ax_right.step)<<"</text>\n";
    }

    // X axis ticks + labels
    for (double v : ax_x.ticks) {
        double px=tx(v);
        if (px<L.x0()-0.5||px>L.x1()+0.5) continue;
        svg<<"<line x1='"<<px<<"' y1='"<<L.y1()
           <<"' x2='"<<px<<"' y2='"<<(L.y1()+5)
           <<"' stroke='#555555' stroke-width='1'/>\n";
        svg<<"<text x='"<<px<<"' y='"<<(L.y1()+17)
           <<"' text-anchor='middle' font-family='Arial,sans-serif'"
           <<" font-size='11' fill='#444444'>"
           <<fmt_tick(v,ax_x.step)<<"</text>\n";
    }

    // Legend
    int lx=L.x0()+10, ly=L.y0()+10;
    svg<<"<rect x='"<<lx<<"' y='"<<ly
       <<"' width='180' height='52'"
       <<" fill='#ffffffcc' stroke='#aaaaaa' stroke-width='0.8' rx='3'/>\n";
    svg<<"<line x1='"<<(lx+8)<<"' y1='"<<(ly+14)
       <<"' x2='"<<(lx+28)<<"' y2='"<<(ly+14)
       <<"' stroke='#0072BD' stroke-width='2.5'/>\n";
    svg<<"<text x='"<<(lx+34)<<"' y='"<<(ly+18)
       <<"' font-family='Arial,sans-serif' font-size='11' fill='#333333'>"
       <<"Best cost (log10)</text>\n";
    svg<<"<circle cx='"<<(lx+18)<<"' cy='"<<(ly+33)
       <<"' r='4' fill='#0072BD' fill-opacity='0.3'/>\n";
    svg<<"<text x='"<<(lx+34)<<"' y='"<<(ly+37)
       <<"' font-family='Arial,sans-serif' font-size='11' fill='#333333'>"
       <<"All evaluations</text>\n";
    svg<<"<line x1='"<<(lx+8)<<"' y1='"<<(ly+14+26)
       <<"' x2='"<<(lx+28)<<"' y2='"<<(ly+14+26)
       <<"' stroke='#D95319' stroke-width='1.5' stroke-dasharray='5,3'/>\n";

    svg << "</svg>\n";

    std::ofstream f(fn);
    if (!f) { std::cerr << "Cannot write " << fn << "\n"; return; }
    f << svg.str();
}

} // namespace plot_svg (convergence extension)
