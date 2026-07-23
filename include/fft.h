#pragma once

#include <vector>
#include <complex>

// Minimal FFT header for plotting helpers
namespace fft_lib {
	using complex_t = std::complex<double>;
	std::vector<double> fft_magnitude(const std::vector<double>& signal);
	std::vector<complex_t> fft_complex(const std::vector<complex_t>& input);
	std::vector<double> compute_magnitude(const std::vector<complex_t>& spectrum);
	int next_pow2(int n);
} // namespace fft_lib
