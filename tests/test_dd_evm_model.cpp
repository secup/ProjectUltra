// Tests for the decision-directed EVM model (ofdm/dd_evm_model.hpp).
//
//   1. The closed forms equal a brute-force simulation of the receiver's estimator
//      (nearest-point decisions over the transmitter's constellation points, gain-corrected
//      S_dd / (S_dd - S_de^2 / S_ee)) for every modelled constellation and SNR.
//   2. The reading is monotone in the true SNR and the inversion round-trips.
//   3. Translation is the identity within a constellation and carries a reading between
//      constellations at the same true SNR; unmodelled constellations return NaN.

#include "ofdm/dd_evm_model.hpp"

#include <cmath>
#include <complex>
#include <iostream>
#include <random>
#include <vector>

using namespace ultra;
using Cx = std::complex<double>;

namespace {

int tests_run = 0;
int tests_failed = 0;

#define CHECK(cond, msg) \
    do { \
        ++tests_run; \
        if (!(cond)) { ++tests_failed; std::cout << "FAIL: " << msg << "\n"; } \
    } while (0)

// The transmitter's constellations (src/ofdm/modulator.cpp), written out independently.
std::vector<Cx> points(Modulation mod) {
    std::vector<Cx> p;
    auto rect = [&p](const std::vector<double>& li, const std::vector<double>& lq, double scale) {
        for (double i : li) for (double q : lq) p.emplace_back(i * scale, q * scale);
    };
    switch (mod) {
        case Modulation::BPSK: p = {Cx(-1, 0), Cx(1, 0)}; break;
        case Modulation::QPSK: rect({-1, 1}, {-1, 1}, 1.0 / std::sqrt(2.0)); break;
        case Modulation::QAM8:
            for (int k = 0; k < 8; ++k) p.push_back(std::polar(1.0, k * M_PI / 4.0 + M_PI / 8.0));
            break;
        case Modulation::QAM16: rect({-3, -1, 1, 3}, {-3, -1, 1, 3}, 1.0 / std::sqrt(10.0)); break;
        case Modulation::QAM32:
            rect({-3, -1, 1, 3}, {-7, -5, -3, -1, 1, 3, 5, 7}, 1.0 / std::sqrt(26.0));
            break;
        case Modulation::QAM64:
            rect({-7, -5, -3, -1, 1, 3, 5, 7}, {-7, -5, -3, -1, 1, 3, 5, 7}, 1.0 / std::sqrt(42.0));
            break;
        default: break;
    }
    return p;
}

double simulatedReadingDb(Modulation mod, double snr_db, int n, uint64_t seed) {
    const auto pts = points(mod);
    std::mt19937_64 rng(seed);
    std::uniform_int_distribution<size_t> pick(0, pts.size() - 1);
    std::normal_distribution<double> g(0.0, std::sqrt(0.5 * std::pow(10.0, -snr_db / 10.0)));
    double s_dd = 0.0, s_ee = 0.0, s_de = 0.0;
    for (int i = 0; i < n; ++i) {
        const Cx y = pts[pick(rng)] + Cx(g(rng), g(rng));
        Cx d = pts[0];
        double best = std::norm(y - d);
        for (const Cx& c : pts) {
            const double e = std::norm(y - c);
            if (e < best) { best = e; d = c; }
        }
        s_dd += std::norm(d);
        s_ee += std::norm(y);
        s_de += (d * std::conj(y)).real();
    }
    return 10.0 * std::log10(s_dd / (s_dd - s_de * s_de / s_ee));
}

const Modulation kModelled[] = {Modulation::BPSK, Modulation::QPSK, Modulation::QAM8,
                                Modulation::QAM16, Modulation::QAM32, Modulation::QAM64};

void test_closed_form_matches_receiver_estimator() {
    for (Modulation mod : kModelled) {
        for (double snr : {0.0, 4.0, 8.0, 12.0, 16.0, 20.0, 25.0}) {
            const double model = dd_evm::expectedReadingDb(mod, snr);
            const double sim = simulatedReadingDb(mod, snr, 200000, 7 + static_cast<int>(snr));
            CHECK(std::fabs(model - sim) < 0.06,
                  modulationToString(mod) << " @" << snr << " dB: model " << model
                                          << " vs simulated " << sim);
        }
    }
}

void test_known_points() {
    // The bias the selector corrects, at a true 12 dB (the 2026-11-10 measurement's numbers).
    CHECK(std::fabs(dd_evm::expectedReadingDb(Modulation::QPSK, 12.0) - 12.26) < 0.05,
          "QPSK reads ~12.3 dB at a true 12 dB");
    CHECK(std::fabs(dd_evm::expectedReadingDb(Modulation::QAM16, 12.0) - 13.04) < 0.05,
          "16QAM reads ~13.0 dB at a true 12 dB");
    CHECK(std::fabs(dd_evm::expectedReadingDb(Modulation::QAM32, 12.0) - 15.20) < 0.06,
          "32QAM reads ~15.2 dB at a true 12 dB");
    // With every decision right only the gain fit remains: 1 + SNR.
    CHECK(std::fabs(dd_evm::expectedReadingDb(Modulation::QAM16, 35.0) -
                    10.0 * std::log10(1.0 + std::pow(10.0, 3.5))) < 0.01,
          "at high SNR the reading is 1 + SNR");
}

void test_monotone_and_invertible() {
    for (Modulation mod : kModelled) {
        double prev = -1e9;
        bool monotone = true;
        for (double snr = dd_evm::kMinTrueSnrDb; snr <= dd_evm::kMaxTrueSnrDb; snr += 0.25) {
            const double r = dd_evm::expectedReadingDb(mod, snr);
            monotone = monotone && r > prev;
            prev = r;
        }
        CHECK(monotone, modulationToString(mod) << " reading rises with the true SNR");
        for (double snr : {-5.0, 3.0, 9.5, 14.0, 22.0, 38.0}) {
            const double back = dd_evm::trueSnrDb(mod, dd_evm::expectedReadingDb(mod, snr));
            CHECK(std::fabs(back - snr) < 1e-3,
                  modulationToString(mod) << " inverts " << snr << " dB (got " << back << ")");
        }
    }
}

void test_translation() {
    CHECK(dd_evm::translateReadingDb(Modulation::QAM16, 13.7, Modulation::QAM16) == 13.7,
          "same constellation: identity");
    CHECK(std::fabs(dd_evm::translateReadingDb(Modulation::QAM16, 13.04, Modulation::QPSK) -
                    12.26) < 0.06,
          "16QAM 13.0 dB and QPSK 12.3 dB are the same true 12 dB");
    // 32QAM's reading carried to 16QAM frames is lower: 32QAM frames read higher near their
    // own threshold than 16QAM frames on the same channel.
    CHECK(dd_evm::translateReadingDb(Modulation::QAM32, 15.5, Modulation::QAM16) < 14.5,
          "32QAM 15.5 dB is under 14.5 dB on 16QAM frames");
    CHECK(std::isnan(dd_evm::translateReadingDb(Modulation::DQPSK, 10.0, Modulation::QPSK)) &&
              std::isnan(dd_evm::expectedReadingDb(Modulation::D8PSK, 10.0)) &&
              !dd_evm::modelled(Modulation::DBPSK),
          "differential constellations are not modelled");
}

}  // namespace

int main() {
    std::cout << "=== Decision-directed EVM model ===\n";
    test_closed_form_matches_receiver_estimator();
    test_known_points();
    test_monotone_and_invertible();
    test_translation();
    std::cout << (tests_failed == 0 ? "PASS" : "FAIL") << ": "
              << (tests_run - tests_failed) << "/" << tests_run << " checks\n";
    return tests_failed == 0 ? 0 : 1;
}
