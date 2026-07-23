#include "dxf_export.h"
#include "cam_utils.h"
#include "bspline.h"
#include "config.h"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>

namespace dxf_export {

    std::string export_cam(const Design& d, const std::string& filename) {
        std::filesystem::path abs_path = std::filesystem::absolute(filename);

        // Always use cam_utils so the exported profile exactly matches
        // what the simulator ran — not the raw design vector CPs.
        BSpline cam = cam_utils::build_cam_from_design(d);

        // DXF_N_POINTS is now 3600 (0.1° per point) — smooth enough that
        // SolidWorks Curve Through XYZ won't introduce visible facets.
        const int    N     = config::DXF_N_POINTS;
        const double SCALE = config::DXF_SCALE;   // 1000 → m to mm

        std::ofstream f(abs_path);
        if (!f.is_open()) {
            std::cerr << "dxf_export::export_cam: cannot open:\n"
                      << "  " << abs_path << "\n";
            return "";
        }

        f << std::fixed << std::setprecision(6);

        for (int k = 0; k <= N; ++k) {        // <= N closes the loop
            double theta_shaft = 2.0 * M_PI * (double)(k % N) / (double)N;
            double theta_e     = std::fmod(theta_shaft * (double)config::N_LOBES,
                                           2.0 * M_PI);
            double r = (config::R_BASE + cam.evaluate(theta_e)) * SCALE;   // mm
            f << r * std::cos(theta_shaft) << "\t"
              << r * std::sin(theta_shaft) << "\t"
              << 0.0                        << "\n";
        }

        f.close();

        std::cout << "\n*** Cam profile written ***\n"
                  << "  " << abs_path.string() << "\n"
                  << "  " << (N + 1) << " points | "
                  << "R_base=" << config::R_BASE * SCALE << " mm | "
                  << "stroke=" << config::STROKE * SCALE << " mm | "
                  << config::N_LOBES << " lobes\n"
                  << "  SolidWorks: Insert → Curve → Curve Through XYZ Points\n\n";

        return abs_path.string();
    }

} // namespace dxf_export
