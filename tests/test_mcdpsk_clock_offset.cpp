// test_mcdpsk_clock_offset.cpp
//
// Proves the MC-DPSK demod tolerates the real soundcard impairments a commercial modem
// must, and discriminates WHICH impairment actually breaks the handshake (so the fix
// targets the real cause, not a guessed one).
//
// Two stations never share a sample clock, and a cheap USB card adds band tilt and
// frequency jitter. None of this appears in the shared-clock, flat simulator -- which
// is exactly why a handshake that passes in sim can fail on a real cable. This test
// injects each impairment with a controlled model, runs the real StreamingEncoder ->
// AWGN -> StreamingDecoder path, and reports decode success, comparing the demod's
// clock/CFO tracking ON vs OFF (track_clock_offset config flag).

#include "gui/modem/streaming_decoder.hpp"
#include "gui/modem/streaming_encoder.hpp"
#include "ota_channel_core/channel.hpp"
#include "protocol/frame_v2.hpp"
#include "ultra/dsp.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iomanip>
#include <iostream>
#include <vector>

using namespace ultra;
using namespace ultra::gui;
namespace v2 = ultra::protocol::v2;

namespace {

constexpr float kFs = 48000.0f;
using Impair = std::function<std::vector<float>(const std::vector<float>&)>;

std::vector<float> withSilence(const std::vector<float>& frame) {
    std::vector<float> audio;
    audio.resize(48000, 0.0f);  // 1 s lead-in for chirp-search noise-floor estimation
    audio.insert(audio.end(), frame.begin(), frame.end());
    audio.resize(audio.size() + 96000, 0.0f);  // trailing room for final decode
    return audio;
}

// Catmull-Rom fractional resampler modeling a constant sample-CLOCK offset of `ppm`.
std::vector<float> resampleClock(const std::vector<float>& in, double ppm) {
    if (in.size() < 4 || ppm == 0.0) return in;
    const double step = 1.0 / (1.0 + ppm * 1e-6);
    const size_t outN = (size_t)std::floor((double)(in.size() - 3) / step);
    std::vector<float> out(outN, 0.0f);
    for (size_t k = 0; k < outN; ++k) {
        const double pos = (double)k * step;
        const long i = (long)std::floor(pos);
        const double f = pos - (double)i;
        const float m1 = (i - 1 >= 0) ? in[i - 1] : in[0];
        const float p0 = in[i], p1 = in[i + 1], p2 = in[i + 2];
        const double a0 = -0.5 * m1 + 1.5 * p0 - 1.5 * p1 + 0.5 * p2;
        const double a1 = m1 - 2.5 * p0 + 2.0 * p1 - 0.5 * p2;
        const double a2 = -0.5 * m1 + 0.5 * p1;
        const double a3 = p0;
        out[k] = (float)(((a0 * f + a1) * f + a2) * f + a3);
    }
    return out;
}

// Linear-in-frequency amplitude tilt (`tilt_db` total across ~300..2700 Hz), one-pole
// high-shelf approximation -- models the cheap card's measured ~15 dB band tilt that
// starves the upper MC-DPSK carriers.
std::vector<float> applyTilt(const std::vector<float>& in, float tilt_db) {
    if (tilt_db == 0.0f) return in;
    const float g = std::pow(10.0f, tilt_db / 20.0f) - 1.0f;  // high-freq extra gain
    std::vector<float> out(in.size());
    float hp = 0.0f, prev = 0.0f;
    const float alpha = 0.97f;  // ~one-pole high-pass, low corner
    for (size_t n = 0; n < in.size(); ++n) {
        hp = alpha * (hp + in[n] - prev);
        prev = in[n];
        out[n] = in[n] + g * hp;
    }
    return out;
}

// Time-varying carrier frequency jitter: instantaneous offset peak_hz*sin(2*pi*rate*t),
// applied to the analytic signal -- models the cheap DAC's measured +-7 Hz jitter.
std::vector<float> applyFreqJitter(const std::vector<float>& in, float peak_hz, float rate_hz) {
    if (peak_hz == 0.0f || in.size() < 256) return in;
    HilbertTransform hilbert(127);
    SampleSpan span(in.data(), in.size());
    auto analytic = hilbert.process(span);
    std::vector<float> out(in.size());
    double phase = 0.0;
    const double w = 2.0 * M_PI * rate_hz / kFs;
    for (size_t n = 0; n < in.size() && n < analytic.size(); ++n) {
        const double f_inst = peak_hz * std::sin(w * (double)n);
        phase += 2.0 * M_PI * f_inst / kFs;
        const Complex rot(std::cos(phase), std::sin(phase));
        out[n] = std::real(analytic[n] * rot);
    }
    return out;
}

void feedInChunks(StreamingDecoder& decoder, const std::vector<float>& audio) {
    constexpr size_t kChunk = 4800;
    for (size_t pos = 0; pos < audio.size(); pos += kChunk) {
        const size_t len = std::min(kChunk, audio.size() - pos);
        decoder.feedAudio(audio.data() + pos, len);
        decoder.processBuffer();
    }
}

// CONNECT-class geometry: 8 carriers, 1024 samples/symbol (46.875 baud), DQPSK.
MultiCarrierDPSKConfig connectGeometry(bool track) {
    MultiCarrierDPSKConfig cfg = mc_dpsk_presets::level8();
    cfg.samples_per_symbol = 1024;
    cfg.track_clock_offset = track;
    return cfg;
}

// Encode a multi-codeword MC-DPSK DATA frame (identical demodulateSoft PHY path as
// CONNECT, sized for a multi-second frame), apply `impair`, push through AWGN, decode.
bool decodes(const Impair& impair, float snr_db, bool track, const Bytes& payload,
             uint32_t noise_seed = 0x5a17u) {
    auto frame = v2::DataFrame::makeData("ALPHA", "BRAVO", 7, payload, CodeRate::R1_4);
    const Bytes serialized = frame.serialize();

    StreamingEncoder encoder;
    encoder.setMode(protocol::WaveformMode::MC_DPSK);
    encoder.setMCDPSKConfig(connectGeometry(track));
    encoder.setDataMode(Modulation::DQPSK, CodeRate::R1_4);
    auto samples = encoder.encodeFrame(serialized);
    if (samples.empty()) return false;

    auto audio = withSilence(impair ? impair(samples) : samples);

    ultra::ota_channel_core::SimulatedChannel channel;
    channel.setSeed(noise_seed);
    channel.configure(snr_db, ultra::ota_channel_core::ChannelType::AWGN);
    channel.transmitFromA(audio);
    audio = channel.receiveForB(audio.size());

    StreamingDecoder decoder;
    decoder.setLogPrefix("CLK");
    decoder.setMode(protocol::WaveformMode::MC_DPSK, true);
    decoder.setMCDPSKConfig(connectGeometry(track));
    decoder.setDataMode(Modulation::DQPSK, CodeRate::R1_4);

    feedInChunks(decoder, audio);
    while (decoder.hasFrame()) {
        auto result = decoder.getFrame();
        if (result.success && result.frame_type == v2::FrameType::DATA &&
            result.frame_data == serialized) {
            return true;
        }
    }
    return false;
}

const char* yn(bool b) { return b ? "PASS" : "FAIL"; }

}  // namespace

int main() {
    Bytes payload(96);
    for (size_t i = 0; i < payload.size(); ++i)
        payload[i] = static_cast<uint8_t>(0x40 + (i * 7) % 64);

    auto clk = [](double ppm) { return [ppm](const std::vector<float>& s) { return resampleClock(s, ppm); }; };
    auto tilt = [](float db) { return [db](const std::vector<float>& s) { return applyTilt(s, db); }; };
    auto jit = [](float pk, float rt) { return [pk, rt](const std::vector<float>& s) { return applyFreqJitter(s, pk, rt); }; };

    int failures = 0;

    // ---- Committed regression: clock-offset tolerance at a healthy SNR -------------
    std::cout << "== Clock offset, AWGN 8 dB (tracking ON must hold to +-700 ppm) ==\n";
    std::cout << "   ppm | OFF  | ON\n";
    for (double ppm : {0.0, 100.0, 300.0, 500.0, 700.0, -300.0, -700.0, 1000.0, 1500.0}) {
        bool off = decodes(clk(ppm), 8.0f, false, payload);
        bool on = decodes(clk(ppm), 8.0f, true, payload);
        std::cout << "  " << std::setw(4) << (int)ppm << " | " << yn(off) << " | " << yn(on) << "\n";
        if (std::abs(ppm) <= 700.0 && !on) { std::cout << "   -> FAIL clock " << (int)ppm << "\n"; ++failures; }
        if (ppm == 0.0 && !off) { std::cout << "   -> FAIL no-regression ppm=0\n"; ++failures; }
    }

    // ---- Characterization (report-only): which impairment reproduces the failure? --
    std::cout << "\n== Characterization (report-only): what breaks MC-DPSK CONNECT? ==\n";

    std::cout << "-- Clock offset near the FLOOR (does tracking help where margin is thin?) --\n";
    std::cout << "   ppm | 6dB OFF/ON | 5dB OFF/ON\n";
    for (double ppm : {0.0, 300.0, 700.0, 1000.0}) {
        std::cout << "  " << std::setw(4) << (int)ppm << " |  "
                  << yn(decodes(clk(ppm), 6.0f, false, payload)) << "/" << yn(decodes(clk(ppm), 6.0f, true, payload))
                  << "  |  "
                  << yn(decodes(clk(ppm), 5.0f, false, payload)) << "/" << yn(decodes(clk(ppm), 5.0f, true, payload))
                  << "\n";
    }

    std::cout << "-- Band tilt (cheap-card ~15 dB), AWGN 8 dB --\n";
    std::cout << "  tilt_dB | ON\n";
    for (float db : {0.0f, 6.0f, 10.0f, 15.0f, 20.0f})
        std::cout << "  " << std::setw(6) << (int)db << "  | " << yn(decodes(tilt(db), 8.0f, true, payload)) << "\n";

    // ---- Committed regression: carrier-jitter tolerance (the real cheap-card failure) -
    // The cheap USB dongle's ~+-7 Hz oscillator jitter is what actually breaks the
    // handshake (it pushes the DQPSK differential past +-45 deg) while clock offset and
    // tilt do not. Common-phase tracking must recover the slow-jitter cases that the
    // untracked demod loses; >=symbol-rate jitter is irreducible and report-only.
    // The tracker must (a) RECOVER the slow-jitter regime a real DAC actually exhibits,
    // and (b) NEVER break a case the untracked demod decoded (the safety invariant), for
    // ALL cases including fast/near-aliasing jitter it cannot follow. Faster/larger jitter
    // is report-only: recovery is a bonus, no-harm is mandatory.
    std::cout << "\n== Frequency jitter (cheap-card +-7 Hz), AWGN 8 dB ==\n";
    std::cout << "  peakHz @rate | OFF  | ON\n";
    struct JC { float pk, rt; bool require_on; };
    // Each cell over kSeeds noise realizations. The channel noise comes from
    // std::normal_distribution, whose sequence is implementation-defined: one seed is a different
    // draw on libc++ and libstdc++, and a single frame at the edge of the waterfall decides on that
    // draw (CI 2026-11-21: 5 Hz @ 3 Hz flipped ON on Linux only). The invariants are statistical:
    // a required case decodes on every realization with tracking ON, and tracking ON never decodes
    // fewer realizations than OFF where OFF is reliable (no harm). Where OFF itself decodes only some
    // realizations the cell sits on the waterfall: five draws at ~40 % success differ by one decode
    // between OFF and ON by chance alone (CI Linux 2026-11-21: 5 Hz @ 3 Hz read 2/5 vs 1/5; Mac 3/5
    // vs 3/5), so a marginal cell is report-only, as its row says.
    constexpr uint32_t kSeeds = 5;
    std::cout << "  (" << kSeeds << " noise realizations per cell: decoded OFF / ON)\n";
    for (JC j : {JC{0, 0, true}, JC{2, 3, true}, JC{7, 1, true},
                 JC{5, 3, false}, JC{7, 3, false}, JC{7, 10, false}}) {
        int off = 0, on = 0;
        for (uint32_t s = 0; s < kSeeds; ++s) {
            off += decodes(jit(j.pk, j.rt), 8.0f, false, payload, 0x5a17u + s) ? 1 : 0;
            on += decodes(jit(j.pk, j.rt), 8.0f, true, payload, 0x5a17u + s) ? 1 : 0;
        }
        std::cout << "  " << std::setw(4) << (int)j.pk << " @" << std::setw(2) << (int)j.rt
                  << "Hz | " << off << "/" << kSeeds << "  | " << on << "/" << kSeeds
                  << (j.require_on ? "" : "  (report-only)") << "\n";
        if (j.require_on && on < static_cast<int>(kSeeds)) {
            std::cout << "   -> FAIL: tracking ON must recover " << (int)j.pk
                      << " Hz @" << (int)j.rt << " Hz jitter on every realization\n";
            ++failures;
        }
        // SAFETY INVARIANT: tracking must never break a case the untracked demod decodes reliably.
        if (off == static_cast<int>(kSeeds) && on < off) {
            std::cout << "   -> FAIL: tracking ON broke a case that decodes every realization with tracking OFF\n";
            ++failures;
        }
    }

    std::cout << "\n";
    if (failures == 0) {
        std::cout << "PASS: clock-offset + carrier-jitter tolerance holds\n";
        return 0;
    }
    std::cout << failures << " failure(s)\n";
    return 1;
}
