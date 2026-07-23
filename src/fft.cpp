#include "fft.h"
#include <complex>
#include <vector>
#include <cmath>

namespace fft_lib {

    int next_pow2(int n) {
        int p = 1;
        while (p < n) p <<= 1;
        return p;
    }

    std::vector<complex_t> fft_complex(const std::vector<complex_t>& input) {
        int n = (int)input.size();
        int N = next_pow2(n);
        std::vector<complex_t> a(N);
        for (int i = 0; i < n; ++i) a[i] = input[i];
        for (int i = n; i < N; ++i) a[i] = complex_t(0.0, 0.0);
        for (int len = 2; len <= N; len <<= 1) {
            double ang = -2.0 * M_PI / len;
            complex_t wlen(cos(ang), sin(ang));
            for (int i = 0; i < N; i += len) {
                complex_t w(1.0, 0.0);
                for (int j = 0; j < len / 2; ++j) {
                    complex_t u = a[i + j];
                    complex_t v = a[i + j + len / 2] * w;
                    a[i + j] = u + v;
                    a[i + j + len / 2] = u - v;
                    w *= wlen;
                }
            }
        }
        return a;
    }

    std::vector<double> compute_magnitude(const std::vector<complex_t>& spectrum) {
        int N = (int)spectrum.size();
        int half = N / 2 + 1;
        std::vector<double> mag(half);
        for (int i = 0; i < half; ++i) mag[i] = std::abs(spectrum[i]);
        return mag;
    }

    std::vector<double> fft_magnitude(const std::vector<double>& signal) {
        int n = (int)signal.size();
        std::vector<complex_t> in(n);
        for (int i = 0; i < n; ++i) in[i] = complex_t(signal[i], 0.0);
        auto spec = fft_complex(in);
        return compute_magnitude(spec);
    }

} // namespace fft_lib