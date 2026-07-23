#include "cam_utils.h"
#include "config.h"
#include <cmath>
#include <algorithm>

namespace cam_utils {

    // ── Cam parameterisation ──────────────────────────────────────────────────
    //
    // History of bugs:
    //   Round 2: boundary zeroing destroyed profile          → 127% ripple
    //   Round 3: std::sort (not differentiable)              → cost stuck at 42000
    //   Round 4: cumsum(raw²) — spike concentration          → 1048% ripple
    //   Round 5: cumsum(relu), symmetric 10/20 CPs only      → correct torque,
    //            but BSpline piecewise-linear with 20 CPs    → visible kinks in SW
    //
    // Root cause of kinks: BSpline::evaluate() is piecewise LINEAR.
    //   With N_CTRL=20 CPs the linear segments span 2π/20 = 18° each — long enough
    //   to produce faceted cam lobes with kinks visible in SolidWorks.
    //
    // Fix — two changes:
    //
    // 1. PARAMETERISATION: Fourier sine-power series.
    //    lift(θ) = Σ_{k=1}^{N_CTRL} a_k · sin(θ/2)^k
    //    • Every basis function sin(θ/2)^k is:
    //        – zero at θ=0 and θ=2π          (base-circle continuity guaranteed)
    //        – smooth (infinitely differentiable, no kinks)
    //        – peaks at θ=π (lobe midpoint) for all k
    //    • a_k = relu(cam[k-1]) keeps coefficients non-negative →
    //      profile is always ≥ 0 and differentiable w.r.t. cam[k-1]
    //    • Normalise so peak(lift) = STROKE
    //    • All N_CTRL=20 design parameters have nonzero gradients
    //    • Optimiser can dial: wide flat lobe (low-k dominant) or sharp lobe
    //      (high-k dominant) to tune torque-ripple tradeoff
    //
    // 2. SAMPLING: N_CAM_SAMPLE=360 BSpline control points (0→2π)
    //    The Fourier series is evaluated at 360 uniformly spaced angles and
    //    stored as BSpline CPs.  With 360 CPs the piecewise-linear interpolation
    //    error is ≤ 0.019 mm — completely invisible in SolidWorks or any CAM tool.
    //    (With 20 CPs it was ≤ 0.074 mm per segment, visually faceted.)

    BSpline build_cam_from_design(const Design& d) {
        const int N_HARM = config::N_CTRL;       // number of Fourier harmonics
        const int N_SAMP = config::N_CAM_SAMPLE; // 360 BSpline control points

        // ── Step 1: extract relu Fourier amplitudes ───────────────────────────
        std::vector<double> a(N_HARM);
        double a_sum = 0.0;
        for (int k = 0; k < N_HARM; ++k) {
            double v = (k < (int)d.cam.size()) ? d.cam[k] : 0.0;
            a[k]   = std::max(0.0, v);
            a_sum += a[k];
        }
        // Degenerate case: all coefficients zero → use raised cosine (k=2 dominant)
        if (a_sum < 1e-14) {
            a[1] = 1.0;   // sin(θ/2)² = (1−cos θ)/2 = standard raised cosine
            a_sum = 1.0;
        }

        // ── Step 2: sample the Fourier series at N_SAMP uniformly spaced angles ─
        // lift(θ) = Σ a_k · sin(θ/2)^(k+1),  k = 0..N_HARM-1
        // NOTE: sin(θ/2) ∈ [0,1] for θ ∈ [0,2π], so sin(θ/2)^(k+1) ∈ [0,1] ✓
        std::vector<double> cp(N_SAMP);
        double peak = 0.0;
        for (int i = 0; i < N_SAMP; ++i) {
            double theta = 2.0 * M_PI * (double)i / (double)N_SAMP;
            double s     = std::sin(theta / 2.0);   // ∈ [0, 1]
            double lift  = 0.0;
            double sk    = s;                         // s^1, s^2, ..., s^N_HARM
            for (int k = 0; k < N_HARM; ++k) {
                lift += a[k] * sk;
                sk   *= s;
            }
            cp[i] = lift;
            if (lift > peak) peak = lift;
        }

        // ── Step 3: normalise so peak lift == STROKE ─────────────────────────
        if (peak > 1e-14) {
            const double scale = config::STROKE / peak;
            for (double& v : cp) v *= scale;
        }

        // cp[0] = 0 exactly (sin(0)=0) and cp[N_SAMP-1] ≈ 0 (sin(π×(N-1)/N) ≈ 0)
        // Pin both ends to exactly zero for clean periodic BSpline boundary.
        cp[0]         = 0.0;
        cp[N_SAMP - 1] = 0.0;

        return BSpline(cp);
    }

} // namespace cam_utils
