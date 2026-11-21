// peak_control_measure.cpp — Phase 0 of docs/PEAK_CONTROL_DESIGN_2026_11_21.md.
//
// Measures, on REAL production bursts (ModemEngine::transmitBurst, the GUI/TNC TX path), what an
// iterative envelope clip-and-filter does to every wideband OFDM rung, and predicts each rung's
// gain on a peak-limited transmitter. No production behaviour changes: the clipper below is a
// file-local prototype applied to a copy of the burst.
//
// Per rung and per (headroom h, pass schedule):
//   Pi(h)    burst PEP over the data cells' mean envelope power (dB) - what the peak-normalized TX
//            gives up;
//   alpha(h) Bussgang gain of the occupied cells (wanted component after / before), split into
//            data cells and pilot cells;
//   D(h)     data signal-to-distortion: |alpha Yu|^2 over |Yc - alpha Yu|^2 on data cells (dB);
//   alpha_LTS, LTS cells' gain (the channel reference the receiver seeds H from);
//   which segment class (data symbols / LTS / other: chirp, descriptor, gaps) holds the peak.
// Then for rung r with EVM threshold eta_r (evm_rate_selector.hpp, turbo table):
//   gain_r(h) = 10 log10[alpha^2 PEP(inf)/PEP(h)] + 10 log10(1 - eta_r / D(h))   (dB, PEP-fair)
// (design §3), and the best h per rung.
//
// Cells are measured exactly: the encoder's diagnostic symbol capture (genie_tx_capture.hpp)
// gives every data symbol's frequency-domain cells and absolute sample offset; the tool checks
// that alignment (correlation of the unclipped burst's FFT with the captured cells) and aborts
// if it is not exact.
//
// Usage: peak_control_measure [--frames N] [--edge-bins B] [--csv PATH]
//   --edge-bins: clip filter edge beyond the outermost carriers, in carrier spacings (default 3)

#include "gui/modem/modem_engine.hpp"
#include "ofdm/genie_tx_capture.hpp"
#include "ofdm/pilot_pattern.hpp"
#include "protocol/evm_rate_selector.hpp"
#include "protocol/frame_v2.hpp"
#include "protocol/waveform_selection.hpp"

#define POCKETFFT_CACHE_SIZE 16
#include "pocketfft/pocketfft_hdronly.h"

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <string>
#include <vector>

using ultra::Bytes;
using ultra::CodeRate;
using ultra::Modulation;
using ultra::gui::ModemEngine;
namespace protocol = ultra::protocol;
namespace v2 = ultra::protocol::v2;
using Cx = std::complex<double>;
template <typename T> long long LL(T v) { return static_cast<long long>(v); }

namespace {

size_t nextPow2(size_t n) {
    size_t p = 1;
    while (p < n) p <<= 1;
    return p;
}

void fft(std::vector<Cx>& v, bool forward) {
    const pocketfft::shape_t shape{v.size()};
    const pocketfft::stride_t stride{static_cast<std::ptrdiff_t>(sizeof(Cx))};
    pocketfft::c2c<double>(shape, stride, stride, {0}, forward, v.data(), v.data(),
                           forward ? 1.0 : 1.0 / static_cast<double>(v.size()));
}

// Analytic signal of a real burst, zero-padded to L (a power of two >= 2 * n).
std::vector<Cx> analytic(const std::vector<float>& x, size_t L) {
    std::vector<Cx> v(L, Cx(0, 0));
    for (size_t i = 0; i < x.size(); ++i) v[i] = Cx(x[i], 0);
    fft(v, true);
    for (size_t b = 1; b < L / 2; ++b) v[b] *= 2.0;
    for (size_t b = L / 2 + 1; b < L; ++b) v[b] = 0;
    fft(v, false);
    return v;
}

// Keep only positive-frequency bins inside [lo_hz, hi_hz] of an analytic signal (brick wall).
void bandLimit(std::vector<Cx>& z, double fs, double lo_hz, double hi_hz) {
    const size_t L = z.size();
    fft(z, true);
    for (size_t b = 0; b < L; ++b) {
        const double f = (b <= L / 2) ? fs * double(b) / double(L) : -1.0;
        if (!(f >= lo_hz && f <= hi_hz)) z[b] = 0;
    }
    fft(z, false);
}

struct Schedule {
    const char* name;
    std::vector<double> overshoot;  // one exponent per pass (scale^k), its length = pass count
};

// Iterative envelope clip-and-filter on the analytic burst (design §4.1). Threshold A is h dB above
// the RMS envelope of the data symbols (not the whole burst).
std::vector<Cx> clipFilter(const std::vector<Cx>& z0, double A, const Schedule& s, double fs,
                           double lo, double hi) {
    std::vector<Cx> z = z0;
    for (double k : s.overshoot) {
        for (auto& c : z) {
            const double m = std::abs(c);
            if (m > A) c *= std::pow(A / m, k);
        }
        bandLimit(z, fs, lo, hi);
    }
    return z;
}

struct Cells {
    // Per captured data symbol: the useful-window start sample and the passband bins of its
    // occupied cells, tagged data / pilot.
    std::vector<size_t> win_start;
    std::vector<std::vector<int>> data_bins, pilot_bins;
};

// FFT of the real part of z over [start, start + N) at the given bins.
void cellValues(const std::vector<Cx>& z, size_t start, size_t N, const std::vector<int>& bins,
                std::vector<Cx>& out, std::vector<Cx>& scratch) {
    scratch.assign(N, Cx(0, 0));
    for (size_t i = 0; i < N && start + i < z.size(); ++i) scratch[i] = Cx(z[start + i].real(), 0);
    fft(scratch, true);
    out.resize(bins.size());
    for (size_t i = 0; i < bins.size(); ++i) out[i] = scratch[static_cast<size_t>(bins[i])];
}

struct Stats {
    double alpha_data = 0, alpha_pilot = 0, d_data_db = 0, d_pilot_db = 0;
};

Stats cellStats(const std::vector<Cx>& zu, const std::vector<Cx>& zc, const Cells& cells, size_t N) {
    std::vector<Cx> yu, yc, scratch;
    Cx xd(0, 0), xp(0, 0);
    double pd = 0, pp = 0;
    // pass 1: alpha
    for (size_t s = 0; s < cells.win_start.size(); ++s) {
        cellValues(zu, cells.win_start[s], N, cells.data_bins[s], yu, scratch);
        cellValues(zc, cells.win_start[s], N, cells.data_bins[s], yc, scratch);
        for (size_t i = 0; i < yu.size(); ++i) { xd += yc[i] * std::conj(yu[i]); pd += std::norm(yu[i]); }
        cellValues(zu, cells.win_start[s], N, cells.pilot_bins[s], yu, scratch);
        cellValues(zc, cells.win_start[s], N, cells.pilot_bins[s], yc, scratch);
        for (size_t i = 0; i < yu.size(); ++i) { xp += yc[i] * std::conj(yu[i]); pp += std::norm(yu[i]); }
    }
    Stats st;
    const Cx ad = pd > 0 ? xd / pd : Cx(0, 0);
    const Cx ap = pp > 0 ? xp / pp : Cx(0, 0);
    st.alpha_data = std::abs(ad);
    st.alpha_pilot = std::abs(ap);
    // pass 2: distortion against the fitted gain
    double sd = 0, ed = 0, sp = 0, ep = 0;
    for (size_t s = 0; s < cells.win_start.size(); ++s) {
        cellValues(zu, cells.win_start[s], N, cells.data_bins[s], yu, scratch);
        cellValues(zc, cells.win_start[s], N, cells.data_bins[s], yc, scratch);
        for (size_t i = 0; i < yu.size(); ++i) { sd += std::norm(ad * yu[i]); ed += std::norm(yc[i] - ad * yu[i]); }
        cellValues(zu, cells.win_start[s], N, cells.pilot_bins[s], yu, scratch);
        cellValues(zc, cells.win_start[s], N, cells.pilot_bins[s], yc, scratch);
        for (size_t i = 0; i < yu.size(); ++i) { sp += std::norm(ap * yu[i]); ep += std::norm(yc[i] - ap * yu[i]); }
    }
    st.d_data_db = ed > 0 ? 10 * std::log10(sd / ed) : 99.0;
    st.d_pilot_db = ep > 0 ? 10 * std::log10(sp / ep) : 99.0;
    return st;
}

struct RungDef {
    Modulation mod;
    CodeRate rate;
    const char* name;
};

const RungDef kRungs[] = {
    {Modulation::QPSK, CodeRate::R1_4, "QPSK R1/4"},   {Modulation::QPSK, CodeRate::R1_2, "QPSK R1/2"},
    {Modulation::QPSK, CodeRate::R2_3, "QPSK R2/3"},   {Modulation::QPSK, CodeRate::R3_4, "QPSK R3/4"},
    {Modulation::QAM8, CodeRate::R2_3, "8PSK R2/3"},   {Modulation::QAM8, CodeRate::R3_4, "8PSK R3/4"},
    {Modulation::QAM16, CodeRate::R1_2, "16QAM R1/2"}, {Modulation::QAM16, CodeRate::R2_3, "16QAM R2/3"},
    {Modulation::QAM16, CodeRate::R3_4, "16QAM R3/4"}, {Modulation::QAM32, CodeRate::R2_3, "32QAM R2/3"},
};

}  // namespace

int main(int argc, char** argv) {
    int n_frames = 6;
    double edge_bins = 3.0;  // filter edge beyond the outermost carriers, in carrier spacings
    const char* csv_path = nullptr;
    for (int i = 1; i < argc; ++i) {
        if (!std::strcmp(argv[i], "--frames") && i + 1 < argc) n_frames = std::atoi(argv[++i]);
        else if (!std::strcmp(argv[i], "--csv") && i + 1 < argc) csv_path = argv[++i];
        else if (!std::strcmp(argv[i], "--edge-bins") && i + 1 < argc) edge_bins = std::atof(argv[++i]);
        else { std::fprintf(stderr, "usage: %s [--frames N] [--edge-bins B] [--csv PATH]\n", argv[0]); return 2; }
    }
    FILE* csv = csv_path ? std::fopen(csv_path, "w") : nullptr;
    if (csv) std::fprintf(csv, "rung,schedule,h_db,pi_db,pep_loss_db,alpha_data,alpha_pilot,alpha_lts,d_data_db,d_pilot_db,peak_class,gain_db\n");

    const std::vector<Schedule> schedules = {
        {"1pass", {1.0}},
        {"3pass", {1.0, 1.5, 2.0}},
        {"5pass", {1.0, 1.25, 1.5, 1.75, 2.0}},
    };
    std::vector<double> headrooms;
    for (double h = 0.0; h <= 12.0 + 1e-9; h += 1.0) headrooms.push_back(h);

    ultra::genie::txCapture().enabled = true;
    std::printf("Peak-control Phase 0 (production ModemEngine::transmitBurst, %d frames/burst, "
                "second burst of the session measured)\n", n_frames);
    std::printf("clip filter edge: %.1f carrier spacings beyond the outermost carriers\n", edge_bins);
    std::printf("gain = PEP-fair dB vs the unclipped burst at the rung's EVM threshold (turbo table)\n\n");

    for (const auto& r : kRungs) {
        ModemEngine tx;
        tx.setLogPrefix("PC0");
        tx.joinConnectedSession(protocol::WaveformMode::OFDM_CHIRP, r.mod, r.rate);
        tx.setMcDpskSessionAddresses(v2::hashCallsign("ALPHA"), v2::hashCallsign("BRAVO"));
        const int cw = tx.getFixedFrameCodewords();
        const size_t cap = v2::getFixedFramePayloadCapacity(r.rate, cw);
        std::mt19937 rng(1234);
        auto group = [&](uint16_t seq0) {
            std::vector<Bytes> g;
            for (int f = 0; f < n_frames; ++f) {
                Bytes p(cap);
                for (auto& b : p) b = static_cast<uint8_t>(rng());
                g.push_back(v2::makeFixedDataFrame("ALPHA", "BRAVO", uint16_t(seq0 + f), p, r.rate, cw).serialize());
            }
            return g;
        };
        (void)tx.transmitBurst(group(1), 0);  // first group of the session (anchor)
        ultra::genie::txCapture().reset();
        ultra::genie::txCapture().enabled = true;
        const std::vector<float> burst = tx.transmitBurst(group(100), 1);
        const auto& cap_syms = ultra::genie::txCapture().symbols;
        if (burst.empty() || cap_syms.empty()) {
            std::printf("%-11s  no burst / no captured symbols - skipped\n", r.name);
            continue;
        }

        const ultra::ModemConfig& cfg = tx.getConfig();
        const size_t N = cfg.fft_size;
        const size_t cp = cfg.getCyclicPrefix();
        const size_t symlen = cfg.getSymbolDuration();
        const double fs = cfg.sample_rate;
        const double df = fs / double(N);
        const int fc_bin = static_cast<int>(std::lround(cfg.center_freq / df));
        const double lo = ultra::ofdm_pilots::carrierFrequencyHzForLogical(cfg, 0) - edge_bins * df;
        const double hi = ultra::ofdm_pilots::carrierFrequencyHzForLogical(cfg, cfg.num_carriers - 1) + edge_bins * df;

        // Align the capture's frame offsets with the burst: one constant origin, found on the first
        // symbol and then verified on every symbol (normalized cell correlation >= 0.999).
        auto passbandBins = [&](const std::vector<std::complex<float>>& x, std::vector<int>& bins,
                                std::vector<Cx>& vals) {
            bins.clear(); vals.clear();
            for (size_t i = 0; i < x.size(); ++i) {
                if (std::norm(x[i]) == 0.0f) continue;
                const int k = (i < N / 2) ? int(i) : int(i) - int(N);
                bins.push_back(fc_bin + k);
                vals.push_back(Cx(x[i].real(), x[i].imag()));
            }
        };
        const size_t L = nextPow2(2 * burst.size() + 2 * N);
        const std::vector<Cx> zu = analytic(burst, L);
        std::vector<Cx> scratch, y;
        auto corrAt = [&](long long start, const std::vector<int>& bins, const std::vector<Cx>& vals) {
            if (start < 0 || size_t(start) + N > burst.size()) return 0.0;
            cellValues(zu, size_t(start), N, bins, y, scratch);
            Cx c(0, 0); double a = 0, b = 0;
            for (size_t i = 0; i < y.size(); ++i) { c += y[i] * std::conj(vals[i]); a += std::norm(y[i]); b += std::norm(vals[i]); }
            return (a > 0 && b > 0) ? std::norm(c) / (a * b) : 0.0;
        };
        std::vector<int> bins0; std::vector<Cx> vals0;
        passbandBins(cap_syms[0].x, bins0, vals0);
        long long best_origin = 0; double best = -1;
        const long long base0 = cap_syms[0].frame_base;
        for (long long d = 0; d < LL(burst.size()) && best < 0.9999; ++d) {
            for (long long o : {d, -d}) {
                const long long st = base0 + o + LL(cp);
                if (st < 0 || size_t(st) + N > burst.size()) continue;
                const double c = corrAt(st, bins0, vals0);
                if (c > best) { best = c; best_origin = o; }
            }
        }
        Cells cells;
        double worst = 1.0;
        std::vector<int> bins; std::vector<Cx> vals;
        for (const auto& s : cap_syms) {
            passbandBins(s.x, bins, vals);
            const long long st = s.frame_base + best_origin + LL(cp) + LL(s.sym_in_frame) * LL(symlen);
            const double c = corrAt(st, bins, vals);
            worst = std::min(worst, c);
            std::vector<int> db, pb;
            size_t real_pm1 = 0;
            for (size_t i = 0; i < vals.size(); ++i) {
                const bool pm1 = std::abs(std::abs(vals[i].real()) - 1.0) < 1e-6 && std::abs(vals[i].imag()) < 1e-6;
                real_pm1 += pm1;
            }
            const bool split_ok = (real_pm1 == s.n_pilot);
            for (size_t i = 0; i < vals.size(); ++i) {
                const bool pm1 = std::abs(std::abs(vals[i].real()) - 1.0) < 1e-6 && std::abs(vals[i].imag()) < 1e-6;
                (split_ok && pm1 ? pb : db).push_back(bins[i]);
            }
            cells.win_start.push_back(size_t(st));
            cells.data_bins.push_back(db);
            cells.pilot_bins.push_back(pb);
        }
        if (worst < 0.999) {
            std::printf("%-11s  ALIGNMENT FAILED (worst symbol correlation %.4f, origin %lld) - skipped\n",
                        r.name, worst, best_origin);
            continue;
        }

        // Sample classes: data-symbol spans, the LTS pair right before each frame's first data
        // symbol (verified: the two halves correlate), everything else.
        std::vector<uint8_t> cls(burst.size(), 2);  // 0 data, 1 LTS, 2 other
        for (size_t s = 0; s < cells.win_start.size(); ++s) {
            const size_t b = cells.win_start[s] - cp;
            for (size_t i = b; i < std::min(burst.size(), b + symlen); ++i) cls[i] = 0;
        }
        std::vector<size_t> lts_starts;
        for (size_t f = 0; f < ultra::genie::txCapture().frame_bases.size(); ++f) {
            const long long b = ultra::genie::txCapture().frame_bases[f] + best_origin;
            const long long l0 = b - 2 * LL(symlen);
            if (l0 < 0) continue;
            // the two LTS symbols are identical (the first may be negated as a group marker)
            double c = 0, e0 = 0, e1 = 0;
            for (size_t i = 0; i < symlen; ++i) {
                const double a0 = burst[size_t(l0) + i], a1 = burst[size_t(l0) + symlen + i];
                c += a0 * a1; e0 += a0 * a0; e1 += a1 * a1;
            }
            if (e0 > 0 && e1 > 0 && std::abs(c) / std::sqrt(e0 * e1) > 0.95) {
                lts_starts.push_back(size_t(l0));
                for (size_t i = size_t(l0); i < size_t(l0) + 2 * symlen; ++i) cls[i] = 1;
            }
        }

        double data_env = 0; size_t nd = 0;
        for (size_t i = 0; i < burst.size(); ++i) if (cls[i] == 0) { data_env += std::norm(zu[i]); ++nd; }
        data_env /= std::max<size_t>(1, nd);
        auto peakByClass = [&](const std::vector<Cx>& z, double pk[3]) {
            pk[0] = pk[1] = pk[2] = 0;
            for (size_t i = 0; i < burst.size(); ++i) pk[cls[i]] = std::max(pk[cls[i]], std::norm(z[i]));
        };
        double pk_u[3];
        peakByClass(zu, pk_u);
        const double pep_u = std::max({pk_u[0], pk_u[1], pk_u[2]});
        const double pi_u_db = 10 * std::log10(pep_u / data_env);
        const char* cls_name[3] = {"data", "LTS", "other"};
        const int pk_cls_u = int(std::max_element(pk_u, pk_u + 3) - pk_u);

        const uint8_t idx = protocol::coherentRungIndexFor(r.mod, r.rate);
        const double eta_db = protocol::evmDecodeThresholdTurboDb(idx);
        const double eta = std::pow(10.0, eta_db / 10.0);

        std::printf("== %s  cw=%d  symbols=%zu  LTS pairs=%zu  align=%.5f  eta=%.2f dB\n",
                    r.name, cw, cells.win_start.size(), lts_starts.size(), worst, eta_db);
        std::printf("   unclipped: Pi=%.2f dB  peaks (dB over data env RMS): data %.2f  LTS %.2f  other %.2f  -> set by %s\n",
                    pi_u_db, 10 * std::log10(pk_u[0] / data_env), pk_u[1] > 0 ? 10 * std::log10(pk_u[1] / data_env) : -99.0,
                    pk_u[2] > 0 ? 10 * std::log10(pk_u[2] / data_env) : -99.0, cls_name[pk_cls_u]);
        std::printf("   %-6s %5s %7s %7s %7s %7s %7s %7s %7s %-6s %7s\n", "sched", "h", "Pi", "PEPgain",
                    "a_data", "a_pilot", "a_LTS", "D_data", "D_pilot", "peak", "GAIN");

        double best_gain = 0, best_h = -1; const char* best_s = "none";
        for (const auto& sch : schedules) {
            for (double h : headrooms) {
                const double A = std::sqrt(data_env) * std::pow(10.0, h / 20.0);
                const std::vector<Cx> zc = clipFilter(zu, A, sch, fs, lo, hi);
                const Stats st = cellStats(zu, zc, cells, N);
                // LTS gain, time domain over the verified LTS spans
                Cx lx(0, 0); double lp = 0;
                for (size_t l0 : lts_starts)
                    for (size_t i = l0; i < l0 + 2 * symlen; ++i) { lx += zc[i] * std::conj(zu[i]); lp += std::norm(zu[i]); }
                const double a_lts = lp > 0 ? std::abs(lx / lp) : 0.0;
                double pk_c[3];
                peakByClass(zc, pk_c);
                double env_c = 0;
                for (size_t i = 0; i < burst.size(); ++i) if (cls[i] == 0) env_c += std::norm(zc[i]);
                env_c /= std::max<size_t>(1, nd);
                const double pep_c = std::max({pk_c[0], pk_c[1], pk_c[2]});
                const int pk_cls = int(std::max_element(pk_c, pk_c + 3) - pk_c);
                const double pi_db = 10 * std::log10(pep_c / env_c);
                // wanted data power per PEP, relative to unclipped
                const double pep_gain_db = 10 * std::log10(st.alpha_data * st.alpha_data * pep_u / pep_c);
                const double D = std::pow(10.0, st.d_data_db / 10.0);
                const double gain = (D > eta) ? pep_gain_db + 10 * std::log10(1.0 - eta / D) : -99.0;
                std::printf("   %-6s %5.1f %7.2f %7.2f %7.3f %7.3f %7.3f %7.2f %7.2f %-6s %7.2f\n", sch.name, h,
                            pi_db, pep_gain_db, st.alpha_data, st.alpha_pilot, a_lts, st.d_data_db, st.d_pilot_db,
                            cls_name[pk_cls], gain);
                if (csv)
                    std::fprintf(csv, "%s,%s,%.1f,%.3f,%.3f,%.4f,%.4f,%.4f,%.3f,%.3f,%s,%.3f\n", r.name, sch.name, h,
                                 pi_db, pep_gain_db, st.alpha_data, st.alpha_pilot, a_lts, st.d_data_db, st.d_pilot_db,
                                 cls_name[pk_cls], gain);
                if (gain > best_gain) { best_gain = gain; best_h = h; best_s = sch.name; }
            }
        }
        std::printf("   BEST %s: %s h=%.0f dB -> %+.2f dB PEP-fair\n\n", r.name, best_s, best_h, best_gain);
        std::fflush(stdout);
    }
    if (csv) std::fclose(csv);
    return 0;
}
