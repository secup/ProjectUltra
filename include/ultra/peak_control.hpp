#pragma once

// Peak control for OFDM data bursts (docs/PEAK_CONTROL_DESIGN_2026_11_21.md).
//
// A peak-limited transmitter (every real SSB rig: PEP, ALC; our TX peak-normalizes each burst) loses
// a burst's crest factor in average power. An iterative envelope clip-and-filter trades a
// signal-to-distortion floor D(h) for crest factor; the trade is worth it up to the rung's own
// decode threshold. The headroom h of a rung is DERIVED (design §3):
//
//     gain_r(h) = G(h) + 10 log10(1 - eta_r / D(h))         (dB, PEP-fair)
//     h_r*      = argmax_h gain_r(h)
//
// where G(h) is the wanted data power per unit PEP relative to the unclipped burst and D(h) the
// data-cell signal-to-distortion ratio, both MEASURED on production bursts by
// tools/peak_control_measure (5-pass schedule, filter edge 3 carrier spacings; the same for every
// constellation to ±0.3 dB in D, because after the IDFT every constellation is Gaussian), and eta_r
// is the rung's EVM decode threshold (protocol/evm_rate_selector.hpp). No per-modulation constant.
//
// Knob ULTRA_PEAK_CONTROL: ON by default since 2026-11-21; 0 = off. Same setting on both stations:
// the receiver's EVM rate selector carries each rung's clip floor (thresholdOnFramesOfRungDb). The
// decode itself needs no receiver change: the first data symbol's channel estimate comes from the
// comb pilots, which see the same Bussgang gain as the data (measured ±0.004).

#include <cstddef>
#include <vector>

namespace ultra::peak_control {

struct CurvePoint {
    float h_db;        // clip level over the burst's RMS envelope
    float gain_db;     // G(h): wanted data power per PEP vs unclipped (dB)
    float sdr_db;      // D(h): data-cell signal-to-distortion (dB)
};

// The measured curve (wideband 1024/59 geometry, 5 passes), h = 0..10 dB.
const std::vector<CurvePoint>& wideband5PassCurve();

// MC-DPSK session geometry (8 carriers 500-2500 Hz, 1024-sample symbols, chirp band 250-2750 Hz),
// production clipper, distortion measured INSIDE the tones' bands (+-1 symbol rate around each
// carrier, `peak_control_measure --mcdpsk`): evenly spaced tones put their intermodulation on
// other tones. h = 0..8 dB.
const std::vector<CurvePoint>& mcDpsk8x1024Curve();

// MC-DPSK decode threshold in the same tone-band domain: the measured in-band (3 kHz) AWGN floor
// (DBPSK R1/4 -6 dB, GUI 2026-11-11; DQPSK +2.5 dB over DBPSK, the differential BPSK->QPSK gap,
// connection_policy.hpp) moved into the tones' bands, 10 log10(3000 / (carriers * 2 * baud)).
float mcDpskToneBandThresholdDb(bool dqpsk, int carriers, int samples_per_symbol);

struct Choice;
Choice chooseHeadroomOnCurve(const std::vector<CurvePoint>& curve, float eta_db);

// The clip level of MC-DPSK control frames (CONNECT, CONNECT_ACK, control frames sent through
// ModemEngine::transmit): the largest h whose data-aided reading stays unbiased (modem_engine.cpp).
inline constexpr float kMcDpskControlHeadroomDb = 6.0f;

// G(h) on a measured curve, linear between its points (clamped to its ends).
float curveGainAtDb(const std::vector<CurvePoint>& curve, float h_db);

// How far above the control frames' reading scale a DBPSK DATA frame of the session reads: both are
// the same 8-carrier DPSK payload, the data clipped at its own chooseHeadroom choice and the control
// frame at kMcDpskControlHeadroomDb, so G(h_data) - G(h_control) on the MC-DPSK curve (+2.94 dB:
// 5.62 at h=0 vs 2.68 at h=6; GUI 2026-11-23 at AWGN 3 dB: DBPSK data read -0.2 where the handshake
// read -3.2). 0 with peak control off.
float mcDpskDbpskDataOverControlReadingDb();

// h_r* for a rung whose EVM decode threshold is eta_db; also returns its predicted gain and the
// distortion floor it transmits with. A rung whose threshold no clip level beats returns
// h_db < 0 (send unclipped).
struct Choice {
    float h_db = -1.0f;
    float gain_db = 0.0f;     // net predicted gain at eta (G + distortion penalty)
    float curve_g_db = 0.0f;  // G(h): wanted data power per PEP vs unclipped
    float sdr_db = 99.0f;     // D(h); 99 = unclipped
};
Choice chooseHeadroom(float eta_db);

// The EVM a frame of rung b would read, given a frame of rung a read evm_a_db, both rungs clipped
// at their own chooseHeadroom() choice (design §4.2.3): remove a's distortion floor, move the
// thermal SNR by the PEP-fair power difference G_b - G_a, add b's floor. Returns -99 when a's
// reading is at or below its own floor (no thermal SNR can be inferred) - callers treat that as
// "cannot climb on this frame".
float predictEvmAcrossRungsDb(float evm_a_db, const Choice& a, const Choice& b);

// The threshold, on frames of rung a, that corresponds to rung b's own threshold eta_b_db:
// 1/e = (G_b/G_a)(1/eta_b - 1/D_b) + 1/D_a. 99 when rung b cannot decode at any SNR (D_b <= eta_b).
float thresholdOnFramesOfRungDb(float eta_b_db, const Choice& a, const Choice& b);

struct Result {
    bool applied = false;
    float h_db = 0.0f;
    float pre_crest_db = 0.0f;   // envelope PEP over mean envelope power of the active span
    float post_crest_db = 0.0f;
};

// Clip-and-filter `samples` (real audio at fs) in place: envelope threshold h_db above the RMS
// envelope of the active (non-silent) span, 5 passes with overshoot 1, 1.25, 1.5, 1.75, 2, each
// followed by a band limit to [lo_hz, hi_hz]. The overall level is not renormalized (the TX
// peak-normalizes afterwards).
Result apply(std::vector<float>& samples, float h_db, float fs, float lo_hz, float hi_hz);

// ULTRA_PEAK_CONTROL (read once).
bool enabled();

}  // namespace ultra::peak_control
