// Preamble-less structure acquisition (OFDMChirpWaveform::detectFrameStructure,
// ULTRA_BURST_ACQ=structure). Uses the PRODUCTION wideband geometry (presets::balanced():
// FFT 1024, 59 carriers, MEDIUM CP) and a real control-profile (QPSK R1/4) light frame —
// the burst descriptor shape — so the template is checked against the modulator itself.
//
//  1. detection + timing: a descriptor buried in noise at a random offset is found, with the
//     reported start inside the cyclic-prefix plateau [-CP-8, +8] (the detector reports the
//     plateau MIDDLE, centring the FFT window in the ISI-free region), at 0..20 dB in-band SNR;
//  2. false alarms: noise-only buffers essentially never trigger (chi ~ Gamma(2,1));
//  3. tone immunity: strong CW tones (the tone-burst ACK band) never trigger;
//  4. CFO tolerance: a +/-3 Hz unknown residual CFO is still detected;
//  5. regression (2026-11-08 GUI stall at 6%): structure mode must NOT touch detectDataSync —
//     the in-group warm light-LTS lock on a 4416-sample window still works with the knob on;
//  6. regression (2026-11-08 post-switch RTO loop): a GROUP-START frame carries the burst
//     marker (LTS1 negated); it must be detected AND reported as marked, and an unmarked frame
//     must not be (the first statistic cancelled itself on marked frames and locked frame 2);
//  7. settling lead-in (chirp-less burst start): the settling symbols alone never trigger, and
//     a descriptor that directly follows them is still found inside its CP plateau.
#include "env_compat.hpp"  // setenv on MSVC
#include "protocol/frame_v2.hpp"
#include "waveform/ofdm_chirp_waveform.hpp"
#include "ultra/types.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <vector>

using namespace ultra;

namespace {
int g_fail = 0;
void check(bool ok, const char* msg) {
    std::printf("%s %s\n", ok ? "ok:  " : "FAIL:", msg);
    if (!ok) ++g_fail;
}

// The production per-pass false-lock budget (StreamingDecoder::searchForSync: one pass per
// 4800-sample CORRELATION_STEP, one false lock per hour of continuous searching).
constexpr double kBudget = 4800.0 / (3600.0 * 48000.0);

ModemConfig controlConfig() {
    ModemConfig cfg = presets::balanced();
    cfg.modulation = Modulation::QPSK;
    cfg.code_rate = CodeRate::R1_4;
    return cfg;
}

// Real light descriptor: LTS pair + one R1/4 LDPC codeword of a 20-byte control frame.
Samples makeDescriptor(OFDMChirpWaveform& wf) {
    auto cf = protocol::v2::ControlFrame::makeAck("ALPHA", "BRAVO", 7);
    auto cws = protocol::v2::encodeFrameWithLDPC(cf.serialize(), CodeRate::R1_4);
    Bytes coded;
    for (auto& c : cws) coded.insert(coded.end(), c.begin(), c.end());
    Samples pre = wf.generateDataPreamble();
    Samples body = wf.modulate(coded);
    pre.insert(pre.end(), body.begin(), body.end());
    return pre;
}

double rms(const Samples& x) {
    double s = 0;
    for (float v : x) s += double(v) * v;
    return std::sqrt(s / std::max<size_t>(1, x.size()));
}

// In-band (50-2950 Hz of a 24 kHz Nyquist band) SNR -> white noise sigma.
double noiseSigmaFor(double sig_rms, double snr_db) {
    const double inband_fraction = 2900.0 / 24000.0;
    const double noise_inband_power = sig_rms * sig_rms / std::pow(10.0, snr_db / 10.0);
    return std::sqrt(noise_inband_power / inband_fraction);
}

// Apply a real-signal frequency shift via the analytic signal (FFT Hilbert).
Samples shiftHz(const Samples& x, double hz) {
    if (hz == 0.0) return x;
    const size_t n = x.size();
    Samples y(n);
    // Simple discrete Hilbert via direct DFT is too slow; use a long FIR Hilbert instead.
    const int taps = 255, half = taps / 2;
    std::vector<double> h(taps, 0.0);
    for (int k = -half; k <= half; ++k) {
        if (k % 2 != 0) {
            const double w = 0.54 + 0.46 * std::cos(M_PI * k / half);
            h[static_cast<size_t>(k + half)] = 2.0 / (M_PI * k) * w;
        }
    }
    for (size_t i = 0; i < n; ++i) {
        double q = 0.0;
        for (int k = -half; k <= half; ++k) {
            const long j = static_cast<long>(i) - k;
            if (j >= 0 && j < static_cast<long>(n)) q += h[static_cast<size_t>(k + half)] * x[static_cast<size_t>(j)];
        }
        const double ph = 2.0 * M_PI * hz * static_cast<double>(i) / 48000.0;
        y[i] = static_cast<float>(x[i] * std::cos(ph) - q * std::sin(ph));
    }
    return y;
}
}  // namespace

int main() {
    setenv("ULTRA_BURST_ACQ", "structure", 1);
    const ModemConfig cfg = controlConfig();
    OFDMChirpWaveform tx(cfg);
    tx.configure(Modulation::QPSK, CodeRate::R1_4);
    const Samples desc = makeDescriptor(tx);
    const double sig_rms = rms(desc);
    const int Ls = tx.getSamplesPerSymbol();
    std::printf("      descriptor: %zu samples (%.1f symbols of %d), rms %.3f\n", desc.size(),
                double(desc.size()) / Ls, Ls, sig_rms);

    std::mt19937_64 rng(1234);
    // 1. Detection + timing across SNR.
    for (double snr : {0.0, 4.0, 8.0, 20.0}) {
        int found = 0, timed = 0;
        const int trials = 20;
        for (int t = 0; t < trials; ++t) {
            const size_t lead = 4800 + static_cast<size_t>(rng() % 24000);
            Samples buf(lead + desc.size() + 9600, 0.0f);
            for (size_t i = 0; i < desc.size(); ++i) buf[lead + i] = desc[i];
            std::normal_distribution<double> nd(0.0, noiseSigmaFor(sig_rms, snr));
            for (auto& v : buf) v += static_cast<float>(nd(rng));
            OFDMChirpWaveform rx(cfg);
            rx.configure(Modulation::QPSK, CodeRate::R1_4);
            SyncResult r;
            if (rx.detectFrameStructure(SampleSpan(buf.data(), buf.size()), r, 0.0f, kBudget)) {
                ++found;
                const long off = static_cast<long>(r.start_sample) - static_cast<long>(lead);
                if (off >= -104 && off <= 8) ++timed;  // inside the CP plateau [-CP-8, +8]
                else if (std::getenv("STRUCT_DEBUG")) std::printf("        miss-timed: snr=%.0f off=%ld\n", snr, off);
            }
        }
        std::printf("      SNR %4.1f dB: detected %d/%d, timing inside CP plateau %d/%d\n", snr, found, trials,
                    timed, trials);
        char m[128];
        if (snr >= 4.0) {
            std::snprintf(m, sizeof(m), "SNR %.0f dB: 100%% detected, all inside the CP plateau", snr);
            check(found == trials && timed == found, m);
        } else {
            // 0 dB in-band is 8 dB below every OFDM rung floor (the R1/4 descriptor itself
            // would not decode); require useful acquisition, not perfection.
            std::snprintf(m, sizeof(m), "SNR %.0f dB (below every rung floor): >= 75%% detected, >= 90%% of those timed", snr);
            check(found * 4 >= trials * 3 && timed * 10 >= found * 9, m);
        }
    }

    // 2. False alarms on noise-only buffers (~0.75 s each, ~640 tests per buffer).
    {
        int alarms = 0;
        const int trials = 60;
        std::normal_distribution<double> nd(0.0, 0.3);
        for (int t = 0; t < trials; ++t) {
            Samples buf(36000);
            for (auto& v : buf) v = static_cast<float>(nd(rng));
            OFDMChirpWaveform rx(cfg);
            rx.configure(Modulation::QPSK, CodeRate::R1_4);
            SyncResult r;
            if (rx.detectFrameStructure(SampleSpan(buf.data(), buf.size()), r, 0.0f, kBudget)) ++alarms;
        }
        std::printf("      noise-only: %d false alarms in %d buffers\n", alarms, trials);
        check(alarms == 0, "no false alarm on noise-only buffers");
    }

    // 3. Tone immunity: CW tones in the ACK band at +30 dB over the noise.
    {
        int alarms = 0;
        const int trials = 20;
        std::normal_distribution<double> nd(0.0, 0.01);
        for (int t = 0; t < trials; ++t) {
            Samples buf(36000);
            const double f1 = 2400.0 + 75.0 * (t % 4), f2 = 2475.0 + 75.0 * ((t + 1) % 4);
            for (size_t i = 0; i < buf.size(); ++i) {
                const double tt = static_cast<double>(i) / 48000.0;
                buf[i] = static_cast<float>(0.3 * std::sin(2 * M_PI * f1 * tt) + 0.3 * std::sin(2 * M_PI * f2 * tt) + nd(rng));
            }
            OFDMChirpWaveform rx(cfg);
            rx.configure(Modulation::QPSK, CodeRate::R1_4);
            SyncResult r;
            if (rx.detectFrameStructure(SampleSpan(buf.data(), buf.size()), r, 0.0f, kBudget)) ++alarms;
        }
        check(alarms == 0, "two strong CW tones (ACK band) never trigger");
    }

    // 4. Residual CFO tolerance (unknown to the detector).
    for (double cfo : {-3.0, 3.0}) {
        int found = 0;
        const int trials = 8;
        for (int t = 0; t < trials; ++t) {
            const size_t lead = 6000 + static_cast<size_t>(rng() % 12000);
            Samples buf(lead + desc.size() + 9600, 0.0f);
            for (size_t i = 0; i < desc.size(); ++i) buf[lead + i] = desc[i];
            buf = shiftHz(buf, cfo);
            std::normal_distribution<double> nd(0.0, noiseSigmaFor(sig_rms, 10.0));
            for (auto& v : buf) v += static_cast<float>(nd(rng));
            OFDMChirpWaveform rx(cfg);
            rx.configure(Modulation::QPSK, CodeRate::R1_4);
            SyncResult r;
            if (rx.detectFrameStructure(SampleSpan(buf.data(), buf.size()), r, 0.0f, kBudget) &&
                static_cast<long>(r.start_sample) - static_cast<long>(lead) >= -104 &&
                static_cast<long>(r.start_sample) - static_cast<long>(lead) <= 8) {
                ++found;
            }
        }
        char m[80];
        std::snprintf(m, sizeof(m), "unknown CFO %+.0f Hz at 10 dB: detected and timed", cfo);
        check(found == trials, m);
    }

    // 6. Marked group-start frames (LTS1 negated, as markFirstLTSSymbolForBurstGroup does).
    {
        Samples marked = desc;
        for (int i = 0; i < Ls; ++i) marked[static_cast<size_t>(i)] = -marked[static_cast<size_t>(i)];
        for (double snr : {4.0, 8.0, 20.0}) {
            int found = 0, flagged = 0, unmarked_flagged = 0;
            const int trials = 12;
            for (int t = 0; t < trials; ++t) {
                for (int variant = 0; variant < 2; ++variant) {
                    const Samples& frame = variant == 0 ? marked : desc;
                    const size_t lead = 4800 + static_cast<size_t>(rng() % 12000);
                    Samples buf(lead + frame.size() + 9600, 0.0f);
                    for (size_t i = 0; i < frame.size(); ++i) buf[lead + i] = frame[i];
                    std::normal_distribution<double> nd(0.0, noiseSigmaFor(sig_rms, snr));
                    for (auto& v : buf) v += static_cast<float>(nd(rng));
                    OFDMChirpWaveform rx(cfg);
                    rx.configure(Modulation::QPSK, CodeRate::R1_4);
                    SyncResult r;
                    const bool ok = rx.detectFrameStructure(SampleSpan(buf.data(), buf.size()), r, 0.0f, kBudget);
                    const long off = static_cast<long>(r.start_sample) - static_cast<long>(lead);
                    if (variant == 0) {
                        if (ok && off >= -104 && off <= 8) ++found;
                        if (ok && rx.wasBurstInterleaved()) ++flagged;
                    } else if (ok && rx.wasBurstInterleaved()) {
                        ++unmarked_flagged;
                    }
                }
            }
            std::printf("      marked frame SNR %4.1f dB: detected+timed %d/%d, marker reported %d/%d, "
                        "unmarked mis-flagged %d/%d\n", snr, found, trials, flagged, trials, unmarked_flagged, trials);
            char m[128];
            std::snprintf(m, sizeof(m), "SNR %.0f dB: marked group start detected, timed and flagged; unmarked never flagged", snr);
            check(found == trials && flagged == trials && unmarked_flagged == 0, m);
        }
    }

    // 7. Settling lead-in fill.
    {
        const Samples settle = tx.generateSettlingSymbols(6);
        check(settle.size() == static_cast<size_t>(6 * Ls), "settling fill has exactly the requested symbols");
        int alarms = 0, found = 0;
        const int trials = 12;
        for (int t = 0; t < trials; ++t) {
            // (a) settle symbols alone, embedded in noise
            {
                Samples buf(4800 + settle.size() + 9600, 0.0f);
                for (size_t i = 0; i < settle.size(); ++i) buf[4800 + i] = settle[i];
                std::normal_distribution<double> nd(0.0, noiseSigmaFor(sig_rms, 20.0));
                for (auto& v : buf) v += static_cast<float>(nd(rng));
                OFDMChirpWaveform rx(cfg);
                rx.configure(Modulation::QPSK, CodeRate::R1_4);
                SyncResult r;
                if (rx.detectFrameStructure(SampleSpan(buf.data(), buf.size()), r, 0.0f, kBudget)) ++alarms;
            }
            // (b) settle symbols immediately followed by the descriptor
            {
                const size_t lead = 4800 + static_cast<size_t>(rng() % 4800);
                Samples buf(lead + settle.size() + desc.size() + 9600, 0.0f);
                for (size_t i = 0; i < settle.size(); ++i) buf[lead + i] = settle[i];
                const size_t at = lead + settle.size();
                for (size_t i = 0; i < desc.size(); ++i) buf[at + i] = desc[i];
                std::normal_distribution<double> nd(0.0, noiseSigmaFor(sig_rms, 12.0));
                for (auto& v : buf) v += static_cast<float>(nd(rng));
                OFDMChirpWaveform rx(cfg);
                rx.configure(Modulation::QPSK, CodeRate::R1_4);
                SyncResult r;
                if (rx.detectFrameStructure(SampleSpan(buf.data(), buf.size()), r, 0.0f, kBudget)) {
                    const long off = static_cast<long>(r.start_sample) - static_cast<long>(at);
                    if (off >= -104 && off <= 8) ++found;
                }
            }
        }
        std::printf("      settling fill: %d false triggers / %d; descriptor after fill found+timed %d/%d\n",
                    alarms, trials, found, trials);
        check(alarms == 0, "settling symbols never trigger the structure detector");
        check(found == trials, "descriptor directly after the settling fill is found inside its CP plateau");
    }

    // 5. detectDataSync is independent of the structure knob (set at the top of main): the
    //    in-group warm window (planWarmSearch narrow span, 4416 samples) still locks.
    {
        int locked = 0;
        const int trials = 8;
        for (int t = 0; t < trials; ++t) {
            const size_t lead = 960 + static_cast<size_t>(rng() % 200);
            Samples buf(4416, 0.0f);
            for (size_t i = 0; i + lead < buf.size() && i < desc.size(); ++i) buf[lead + i] = desc[i];
            std::normal_distribution<double> nd(0.0, noiseSigmaFor(sig_rms, 20.0));
            for (auto& v : buf) v += static_cast<float>(nd(rng));
            OFDMChirpWaveform rx(cfg);
            rx.configure(Modulation::QPSK, CodeRate::R1_4);
            SyncResult r;
            if (rx.detectDataSync(SampleSpan(buf.data(), buf.size()), r, 0.0f, 0.3f) && r.correlation > 0.5f &&
                std::abs(static_cast<long>(r.start_sample) - static_cast<long>(lead)) <= 48) {
                ++locked;
            }
        }
        std::printf("      warm-window light-LTS lock with structure mode on: %d/%d\n", locked, trials);
        check(locked == trials, "structure mode leaves the in-group detectDataSync lock intact");
    }

    if (g_fail == 0) {
        std::printf("PASS: frame structure detector\n");
        return 0;
    }
    std::printf("%d FAILURE(S)\n", g_fail);
    return 1;
}
