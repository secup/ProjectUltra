#include "ultra/dsp.hpp"
#include <cmath>
#include <stdexcept>

#ifdef ULTRA_HAS_FFTW
#include <fftw3.h>
#endif

namespace ultra {

struct FFT::Impl {
    size_t size;

#ifdef ULTRA_HAS_FFTW
    fftwf_plan forward_plan = nullptr;
    fftwf_plan inverse_plan = nullptr;
    fftwf_plan forward_real_plan = nullptr;
    fftwf_plan inverse_real_plan = nullptr;
    fftwf_complex* buffer = nullptr;
    float* real_buffer = nullptr;

    ~Impl() {
        if (forward_plan) fftwf_destroy_plan(forward_plan);
        if (inverse_plan) fftwf_destroy_plan(inverse_plan);
        if (forward_real_plan) fftwf_destroy_plan(forward_real_plan);
        if (inverse_real_plan) fftwf_destroy_plan(inverse_real_plan);
        if (buffer) fftwf_free(buffer);
        if (real_buffer) fftwf_free(real_buffer);
    }
#else
    // Fallback: precompute twiddle factors for Cooley-Tukey
    std::vector<Complex> twiddles;
    std::vector<Complex> work_buffer;
#endif
};

FFT::FFT(size_t size) : size_(size), impl_(std::make_unique<Impl>()) {
    if ((size & (size - 1)) != 0) {
        throw std::invalid_argument("FFT size must be power of 2");
    }

    impl_->size = size;

#ifdef ULTRA_HAS_FFTW
    impl_->buffer = fftwf_alloc_complex(size);
    impl_->real_buffer = fftwf_alloc_real(size);

    impl_->forward_plan = fftwf_plan_dft_1d(
        size,
        impl_->buffer, impl_->buffer,
        FFTW_FORWARD, FFTW_MEASURE
    );

    impl_->inverse_plan = fftwf_plan_dft_1d(
        size,
        impl_->buffer, impl_->buffer,
        FFTW_BACKWARD, FFTW_MEASURE
    );

    impl_->forward_real_plan = fftwf_plan_dft_r2c_1d(
        size,
        impl_->real_buffer,
        impl_->buffer,
        FFTW_MEASURE
    );

    impl_->inverse_real_plan = fftwf_plan_dft_c2r_1d(
        size,
        impl_->buffer,
        impl_->real_buffer,
        FFTW_MEASURE
    );
#else
    // Precompute twiddle factors
    impl_->twiddles.resize(size / 2);
    for (size_t k = 0; k < size / 2; ++k) {
        float angle = -2.0f * M_PI * k / size;
        impl_->twiddles[k] = Complex(std::cos(angle), std::sin(angle));
    }
    impl_->work_buffer.resize(size);
#endif
}

FFT::~FFT() = default;

#ifndef ULTRA_HAS_FFTW
// Iterative Cooley-Tukey FFT (fallback)
static void fft_impl(Complex* data, size_t size, const std::vector<Complex>& twiddles, bool inverse) {
    // Bit-reversal permutation
    size_t j = 0;
    for (size_t i = 0; i < size - 1; ++i) {
        if (i < j) std::swap(data[i], data[j]);
        size_t k = size / 2;
        while (k <= j) { j -= k; k /= 2; }
        j += k;
    }

    // Cooley-Tukey butterflies
    for (size_t len = 2; len <= size; len *= 2) {
        size_t half = len / 2;
        size_t step = size / len;
        for (size_t i = 0; i < size; i += len) {
            for (size_t k = 0; k < half; ++k) {
                Complex w = twiddles[k * step];
                if (inverse) w = std::conj(w);
                Complex t = w * data[i + k + half];
                data[i + k + half] = data[i + k] - t;
                data[i + k] = data[i + k] + t;
            }
        }
    }

    // Scale for inverse
    if (inverse) {
        float scale = 1.0f / size;
        for (size_t i = 0; i < size; ++i) {
            data[i] *= scale;
        }
    }
}
#endif

void FFT::forward(const Complex* in, Complex* out) {
#ifdef ULTRA_HAS_FFTW
    std::memcpy(impl_->buffer, in, size_ * sizeof(fftwf_complex));
    fftwf_execute(impl_->forward_plan);
    std::memcpy(out, impl_->buffer, size_ * sizeof(fftwf_complex));
#else
    std::memcpy(out, in, size_ * sizeof(Complex));
    fft_impl(out, size_, impl_->twiddles, false);
#endif
}

void FFT::forward(const std::vector<Complex>& in, std::vector<Complex>& out) {
    if (in.size() != size_) throw std::invalid_argument("Input size mismatch");
    out.resize(size_);
    forward(in.data(), out.data());
}

void FFT::inverse(const Complex* in, Complex* out) {
#ifdef ULTRA_HAS_FFTW
    std::memcpy(impl_->buffer, in, size_ * sizeof(fftwf_complex));
    fftwf_execute(impl_->inverse_plan);
    // FFTW doesn't normalize, so we do it
    float scale = 1.0f / size_;
    for (size_t i = 0; i < size_; ++i) {
        out[i] = Complex(
            impl_->buffer[i][0] * scale,
            impl_->buffer[i][1] * scale
        );
    }
#else
    std::memcpy(out, in, size_ * sizeof(Complex));
    fft_impl(out, size_, impl_->twiddles, true);
#endif
}

void FFT::inverse(const std::vector<Complex>& in, std::vector<Complex>& out) {
    if (in.size() != size_) throw std::invalid_argument("Input size mismatch");
    out.resize(size_);
    inverse(in.data(), out.data());
}

void FFT::forwardReal(const Sample* in, Complex* out) {
#ifdef ULTRA_HAS_FFTW
    std::memcpy(impl_->real_buffer, in, size_ * sizeof(float));
    fftwf_execute(impl_->forward_real_plan);
    for (size_t i = 0; i <= size_ / 2; ++i) {
        out[i] = Complex(impl_->buffer[i][0], impl_->buffer[i][1]);
    }
#else
    // Convert real to complex and use complex FFT
    for (size_t i = 0; i < size_; ++i) {
        impl_->work_buffer[i] = Complex(in[i], 0);
    }
    fft_impl(impl_->work_buffer.data(), size_, impl_->twiddles, false);
    for (size_t i = 0; i <= size_ / 2; ++i) {
        out[i] = impl_->work_buffer[i];
    }
#endif
}

void FFT::inverseReal(const Complex* in, Sample* out) {
#ifdef ULTRA_HAS_FFTW
    for (size_t i = 0; i <= size_ / 2; ++i) {
        impl_->buffer[i][0] = in[i].real();
        impl_->buffer[i][1] = in[i].imag();
    }
    fftwf_execute(impl_->inverse_real_plan);
    float scale = 1.0f / size_;
    for (size_t i = 0; i < size_; ++i) {
        out[i] = impl_->real_buffer[i] * scale;
    }
#else
    // Reconstruct full spectrum and use complex IFFT
    impl_->work_buffer[0] = in[0];
    for (size_t i = 1; i < size_ / 2; ++i) {
        impl_->work_buffer[i] = in[i];
        impl_->work_buffer[size_ - i] = std::conj(in[i]);
    }
    impl_->work_buffer[size_ / 2] = in[size_ / 2];
    fft_impl(impl_->work_buffer.data(), size_, impl_->twiddles, true);
    for (size_t i = 0; i < size_; ++i) {
        out[i] = impl_->work_buffer[i].real();
    }
#endif
}

} // namespace ultra
