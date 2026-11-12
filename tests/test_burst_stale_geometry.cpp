// test_burst_stale_geometry.cpp
//
// BUG-BURST-STALE-GEOMETRY (2026-07-28) — regression gate for BOTH defects on the
// burst group-start path. They are independent and this file pins each one:
//
//   (A) TRUNCATION / ALIGNMENT — the defect that produces the operator-visible
//       TIMEOUT. checkIfReadyToDecode releases the group-start frame once `available`
//       clears the requirement IT computed; if the requirement then GROWS before
//       decodeCurrentFrame runs (a deferred profile apply at the top of the next
//       processBuffer, a GUI-thread setFixedFrameCodewords, or a commanded-rung
//       change), `frame_len = std::min(frame_len, available)` silently TRUNCATES
//       frame 1. burst_next_pos_ AND burst_data_start_abs_ are both derived from that
//       length, so the whole group lands early; refreshBurstAirEnd() inherits the
//       error and the F176 half-duplex ack interlock opens INSIDE the sender's own
//       key-down. The ack is transmitted into the peer's transmission, is lost, and
//       the sender eats a full ARQ RTO (17.5 s measured; 28.2 s between useful acks).
//       FIX: never arm a burst group from a truncated frame — defer and retry.
//
//   (B) STALE GEOMETRY — the defect that DESTROYS the group (0/N CWs on every frame).
//       A missed BURST_HEADER descriptor after a rung switch leaves the receiver
//       slicing with the PREVIOUS rung's constellation. The receiver is the rate
//       AUTHORITY, so it already commanded the sender's next rung: slice with the
//       COMMANDED rung's learned geometry instead of the latched one.
//
// (A) is an unconditional correctness repair: both environment arms must hold and
// recover the truncated group start. Only (B), the commanded-geometry experiment,
// is controlled by ULTRA_COMMANDED_GEOMETRY (DEFAULT-OFF). Its two arms run in one
// process, which requires streaming_decode_policy::commandedGeometryEnabled() to
// read the environment fresh rather than latch it in a function-local static.
//
// Harness: real StreamingEncoder -> in-memory samples -> real StreamingDecoder, fed
// in 4800-sample chunks with one processBuffer() per chunk (single-threaded, no audio
// thread), copied from test_ofdm_snr_calibration.cpp. NO channel impairment: these
// are GEOMETRY defects, and a clean channel makes the fail/pass split structural
// rather than statistical.

#include "gui/modem/streaming_decoder.hpp"
#include "gui/modem/streaming_decode_policy.hpp"
#include "gui/modem/streaming_encoder.hpp"
#include "gui/modem/streaming_frame_policy.hpp"
#include "protocol/connection_policy.hpp"
#include "protocol/frame_v2.hpp"
#include "protocol/waveform_selection.hpp"
#include "ultra/logging.hpp"
#include "ultra/ofdm_link_adaptation.hpp"
#include "ultra/timing_profiler.hpp"
#include "ultra/types.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <thread>
#include <vector>

using namespace ultra;
using namespace ultra::gui;
namespace v2 = ultra::protocol::v2;

namespace {

using Bytes = std::vector<uint8_t>;

int g_failures = 0;

#define EXPECT(cond)                                                            \
    do {                                                                        \
        if (!(cond)) {                                                          \
            ++g_failures;                                                       \
            std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);\
        }                                                                       \
    } while (0)

// Production wideband OFDM geometry (mirrors measure_ack_fer::makeOFDMConfig and
// test_ofdm_snr_calibration::ofdmGeometry): FFT 1024, 59 carriers, LONG CP, pilots
// at the production adaptive spacing for the rung.
ModemConfig ofdmGeometry(Modulation mod, CodeRate rate) {
    ModemConfig cfg;
    cfg.fft_size = 1024;
    cfg.num_carriers = 59;
    cfg.sample_rate = 48000;
    cfg.center_freq = 1500.0f;
    cfg.cp_mode = CyclicPrefixMode::LONG;
    cfg.modulation = mod;
    cfg.code_rate = rate;
    cfg.use_pilots = true;
    cfg.pilot_spacing = ofdm_link_adaptation::recommendedPilotSpacing(mod, rate);
    return cfg;
}

void setKnob(bool on) {
#ifdef _WIN32
    _putenv_s("ULTRA_COMMANDED_GEOMETRY", on ? "1" : "0");
#else
    setenv("ULTRA_COMMANDED_GEOMETRY", on ? "1" : "0", /*overwrite=*/1);
#endif
}

struct BurstResult {
    bool fired = false;
    int unknown_outcomes = 0;
    int backstop_callbacks = 0;
    bool backstop_payload_seen = false;
    bool group_before_backstop = false;
    size_t backstop_arm_abs = 0;
    size_t backstop_fed = 0;
    uint16_t frame_mask = 0;
    bool all_ok = false;
    uint8_t group_size = 0;
    size_t samples_at_callback = 0;   // fed-sample offset when the group finalized
    uint64_t air_remaining_at_callback = 0;
    uint64_t air_gate_span_at_arm = 0;
    int frames_recovered = 0;
    uint32_t truncated_holds = 0;     // guard (A) fired this many times
    uint32_t commanded_arms = 0;      // guard (B) armed this many times
    bool truncation_hold_active = false;
    int active_lifting_z = 0;
    size_t active_erasure_soft_bits = 0;
    size_t burst_min_block_samples = 0;
    size_t wire_frame_samples = 0;
};

struct MarkerProvenanceResult {
    int callbacks = 0;
    int unknown_outcomes = 0;
    uint16_t frame_mask = 0;
    bool all_ok = false;
    bool geometry_proven = false;
    uint8_t group_size = 0;
    int frames_recovered = 0;
};

struct PhysicalTailGroupResult {
    int callbacks = 0;
    int unknown_outcomes = 0;
    uint16_t frame_mask = 0;
    bool all_ok = false;
    bool geometry_proven = false;
    uint8_t group_size = 0;
    int frames_recovered = 0;
    size_t samples_at_callback = 0;
    size_t physical_end_sample = 0;
    uint64_t fixed_decode_calls = 0;
};

struct StandaloneGateResult {
    int callbacks = 0;
    int backstop_callbacks = 0;
    int data_sync_accepts = 0;
    bool backstop_payload_seen = false;
    bool physical_complete = false;
    float last_data_sync_correlation = 0.0f;
    uint64_t air_remaining_at_callback = 0;
    uint64_t one_sample_before_deadline = 0;
    uint64_t at_deadline = 0;
    uint32_t classic_long_arms = 0;
    int active_lifting_z_after = 0;
};

struct FailedAnchorGateResult {
    int callbacks = 0;
    int backstop_callbacks = 0;
    uint64_t before_reset = 0;
    uint64_t after_reset = 0;
};

// Build the group's serialized DATA frames. cw MUST be stamped into the frame
// (byte 12) or the encoder sizes the physical frame from the header instead.
std::vector<Bytes> makeGroupFrames(int group_size, CodeRate rate, int cw,
                                   int lifting_z = 27) {
    std::vector<Bytes> frames;
    for (int f = 0; f < group_size; ++f) {
        Bytes payload(20, static_cast<uint8_t>(0x40 + f));
        auto frame = v2::makeFixedDataFrame("ALPHA", "BRAVO",
                                            static_cast<uint16_t>(f), payload, rate,
                                            cw, lifting_z);
        frames.push_back(frame.serialize());
    }
    return frames;
}

bool sameLogicalDataFrame(const Bytes& a, const Bytes& b) {
    auto normalize = [](const Bytes& bytes) {
        auto frame = v2::DataFrame::deserialize(bytes);
        if (!frame) return bytes;
        frame->flags = static_cast<uint8_t>(
            frame->flags & ~v2::Flags::PHYSICAL_BURST_END);
        return frame->serialize();
    };
    return normalize(a) == normalize(b);
}

std::vector<float> withSilence(const std::vector<float>& burst) {
    std::vector<float> audio(48000, 0.0f);   // 1 s lead-in for chirp search
    audio.insert(audio.end(), burst.begin(), burst.end());
    audio.resize(audio.size() + 192000, 0.0f);  // 4 s trailing room for the tail frame
    return audio;
}

void feedSilenceSamples(StreamingDecoder& dec, uint64_t samples) {
    std::vector<float> zeros(4800, 0.0f);
    while (samples > 0) {
        const size_t n = static_cast<size_t>(
            std::min<uint64_t>(samples, static_cast<uint64_t>(zeros.size())));
        dec.feedAudio(zeros.data(), n);
        samples -= n;
    }
}

void feedAndProcessSilenceSamples(StreamingDecoder& dec, uint64_t samples) {
    std::vector<float> zeros(4800, 0.0f);
    while (samples > 0) {
        const size_t n = static_cast<size_t>(
            std::min<uint64_t>(samples, static_cast<uint64_t>(zeros.size())));
        dec.feedAudio(zeros.data(), n);
        dec.processBuffer();
        samples -= n;
    }
}

constexpr uint64_t kAnchoredBackstopWindowSamples =
    48ull * (12000ull + 1410ull + 1200ull);

// ============================================================================
// TEST (A): never arm a burst group from a TRUNCATED group-start frame.
// ============================================================================
// Both stations are at the SAME rung (QPSK R3/4) — the geometry is CORRECT, so this
// isolates the alignment defect from the stale-constellation defect (B). The decoder
// starts LATCHED at a SHORT cw (so checkIfReadyToDecode releases the frame after only
// the short count) and the requirement is grown in the window between the readiness
// check and the decode — exactly what a deferred profile apply, a GUI-thread
// setFixedFrameCodewords, or (post-fix B) a commanded-rung change does in production,
// where the two resolves are separated by one processBuffer iteration.
BurstResult runTruncationTrial(bool knob_on) {
    setKnob(knob_on);

    // cw16 (the wide coherent rung) on the wire, cw4 latched at the receiver. The
    // gap has to be BIG for a reason that is itself informative: the full-anchor
    // search only declares sync once ~2.5 s of audio sits past the group start, so
    // `available` at the readiness check is already ~1.3 s. Only a frame LONGER than
    // that slack can be truncated — which is also why this defect shows up on the
    // widest rungs, exactly where a group crater costs the most airtime.
    constexpr int kGroup = 3;
    constexpr int kCw = 16;

    StreamingEncoder enc;
    enc.setMode(protocol::WaveformMode::OFDM_CHIRP);
    enc.setOFDMConfig(ofdmGeometry(Modulation::QPSK, CodeRate::R3_4));
    enc.setDataMode(Modulation::QPSK, CodeRate::R3_4);
    enc.setFixedFrameCodewords(kCw);
    enc.setBurstInterleave(false);
    enc.setBurstInterleaveGroupSize(kGroup);
    enc.setBurstDescriptorEnabled(false);

    const auto frames = makeGroupFrames(kGroup, CodeRate::R3_4, kCw);
    const auto audio = withSilence(enc.encodeBurstLight(
        frames, BurstAnchorOptions{/*force_full_group_start=*/true,
                                   /*keep_skip_streak=*/false}));

    StreamingDecoder dec;
    dec.setConnectedOFDMMode(protocol::WaveformMode::OFDM_CHIRP,
                             ofdmGeometry(Modulation::QPSK, CodeRate::R3_4),
                             Modulation::QPSK, CodeRate::R3_4);
    dec.setFixedFrameCodewords(4);   // STALE latch: readiness gates on the short count
    dec.setBurstInterleave(false);
    dec.setBurstInterleaveGroupSize(kGroup);
    dec.applyPendingConfigForTesting();
    // NOTE: deliberately NOT expectFullOFDMAnchorOnce(). That arms the 2.5 s
    // CHIRP_MAX_SEARCH window, so sync is only declared once ~2.5 s of audio sits
    // past the group start — `available` is then far larger than any frame and NO
    // requirement growth can truncate. The connected light-LTS DATA-sync fallback is
    // both the tight-`available` path AND the path this bug actually travels (the
    // receiver missed the full dual-chirp anchor and fell back to DATA sync).

    BurstResult r;
    size_t fed = 0;
    dec.setBurstGroupCallback([&](uint16_t, const std::vector<Bytes>& got, bool all_ok,
                                  float, uint16_t mask, bool, uint8_t gsize, bool) {
        if (r.fired) return;
        r.fired = true;
        r.frame_mask = mask;
        r.all_ok = all_ok;
        r.group_size = gsize;
        r.samples_at_callback = fed;
        r.air_remaining_at_callback = dec.burstAirSamplesRemaining();
        for (const auto& want : frames) {
            for (const auto& g : got) {
                if (sameLogicalDataFrame(g, want)) { ++r.frames_recovered; break; }
            }
        }
    });

    // Small chunks so `available` at the moment the readiness check passes is only
    // just above the STALE requirement — the tight window the guard must survive.
    constexpr size_t kChunk = 1200;
    bool bumped = false;
    for (size_t pos = 0; pos < audio.size() && !r.fired; pos += kChunk) {
        const size_t len = std::min(kChunk, audio.size() - pos);
        dec.feedAudio(audio.data() + pos, len);
        fed += len;
        dec.processBuffer();
        // THE STAGING: the readiness check has just passed at the cw4 length. Grow
        // the requirement before the decode runs. `available` is now ~cw4's frame
        // length while the decode will ask for cw8's (~0.8 s more audio), so
        // frame_len = min(frame_len, available) truncates unless the guard holds.
        if (!bumped && dec.stateForTesting() == DecoderState::DECODING) {
            dec.setFixedFrameCodewords(kCw);
            bumped = true;
        }
    }
    EXPECT(bumped);  // the staging must actually have happened
    r.truncated_holds = dec.truncatedBurstHoldsForTesting();
    r.commanded_arms = dec.commandedGeometryArmsForTesting();
    r.truncation_hold_active = dec.truncationHoldActiveForTesting();
    return r;
}

// ============================================================================
// TEST (B): a MISSED descriptor must be sliced with the COMMANDED rung.
// ============================================================================
// This mirrors the MEASURED rig failure: the receiver DEMOTED the sender (rig:
// 8PSK R2/3 -> QPSK R3/4), the sender obeyed immediately — a demote is never
// deferred, "a failing window never drains" (Connection::maybeObeyAuthorityCommand)
// — and the receiver then missed the BURST_HEADER anchor of the very first group at
// the new rung (setBurstDescriptorEnabled(false) is exactly "the descriptor was
// missed" from the decoder's point of view). It is still LATCHED at the previous,
// FASTER rung and slices the whole group with the wrong constellation.
//
// The rig's own 8PSK-R2/3-cw12 -> QPSK-R3/4-cw8 pair is airtime-IDENTICAL by design
// (cw is airtime-normalizing, connection_policy), which would hide the stride half of
// the defect. QPSK R3/4 cw8 (latched) -> QPSK R1/4 cw4 (commanded + on the wire) is
// the same DEMOTE direction with a genuinely different airtime — pilot spacing 8 vs 5
// AND cw 8 vs 4 — so the wrong-stride/air-end half is exercised too.
//
// PARAMETERIZED so the ADOPTION GATES get pinned by the same harness. "We commanded
// X" is NOT "the sender is at X" — Connection::maybeObeyAuthorityCommand declines a
// standing command on five distinct paths — and arming a rung the sender never took
// destroys a group that decodes perfectly today. Trials C and D below set the wire
// EQUAL to the latched rung (so a correct decoder recovers 5/5) and prove the
// resolver stays out of the way.
struct MissedDescriptorSpec {
    Modulation wire_mod;    CodeRate wire_rate;    int wire_cw;
    Modulation latch_mod;   CodeRate latch_rate;   int latch_cw;
    uint8_t    commanded_idx;                       // kRungIdxNone = no standing cmd
    Modulation learn_mod;   CodeRate learn_rate;   int learn_cw;
    // A descriptor decoded AFTER the command that announced a DIFFERENT rung — the
    // sender telling us on the wire that it has not taken the command.
    bool observed_declining_descriptor;
    int wire_lifting_z = 27;
    int learned_lifting_z = 27;
    int wire_group_size = 5;
    int receiver_group_size = 5;
    bool burst_interleave = false;
    bool corrupt_wire_tail = false;
    bool await_anchored_backstop = false;
};

BurstResult runMissedDescriptorTrial(bool knob_on, const MissedDescriptorSpec& spec) {
    setKnob(knob_on);

    StreamingEncoder enc;
    enc.setMode(protocol::WaveformMode::OFDM_CHIRP);
    enc.setOFDMConfig(ofdmGeometry(spec.wire_mod, spec.wire_rate));
    enc.setDataMode(spec.wire_mod, spec.wire_rate);
    enc.setFixedFrameCodewords(spec.wire_cw);
    enc.setLDPCLiftingZ(spec.wire_lifting_z);
    enc.setBurstInterleave(spec.burst_interleave);
    enc.setBurstInterleaveGroupSize(spec.wire_group_size);
    enc.setBurstDescriptorEnabled(false);   // <- THE MISSED DESCRIPTOR

    const auto frames = makeGroupFrames(spec.wire_group_size, spec.wire_rate,
                                        spec.wire_cw, spec.wire_lifting_z);
    auto burst = enc.encodeBurstLight(
        frames, BurstAnchorOptions{/*force_full_group_start=*/true,
                                   /*keep_skip_streak=*/false});
    if (spec.corrupt_wire_tail) {
        // Preserve the last member's LTS/acquisition energy but destroy its DATA.
        // The receiver reaches the right position without obtaining the
        // CRC-protected PHYSICAL_BURST_END proof.
        const size_t member_samples = static_cast<size_t>(
            enc.getWaveform()->getMinSamplesForCWCount(spec.wire_cw));
        const size_t light_preamble_samples =
            enc.getWaveform()->generateDataPreamble().size();
        EXPECT(burst.size() > member_samples);
        const size_t tail_data_start =
            burst.size() - member_samples + light_preamble_samples;
        EXPECT(tail_data_start < burst.size());
        uint32_t noise_state = 0xC0111DEu;
        for (size_t i = tail_data_start; i < burst.size(); ++i) {
            noise_state = noise_state * 1664525u + 1013904223u;
            burst[i] = (noise_state & 0x80000000u) ? 0.10f : -0.10f;
        }
    }
    const auto audio = withSilence(burst);

    StreamingDecoder dec;
    dec.setConnectedOFDMMode(protocol::WaveformMode::OFDM_CHIRP,
                             ofdmGeometry(spec.latch_mod, spec.latch_rate),
                             spec.latch_mod, spec.latch_rate);
    dec.setFixedFrameCodewords(spec.latch_cw);
    dec.setBurstInterleave(spec.burst_interleave);
    dec.setBurstInterleaveGroupSize(spec.receiver_group_size);
    dec.applyPendingConfigForTesting();
    dec.expectFullOFDMAnchorOnce();
    // What the receiver already knows: it decoded a descriptor for the commanded rung
    // earlier in the session, so its (cw, z) is wire truth in the learned table.
    dec.seedRungGeometryForTesting(spec.learn_mod, spec.learn_rate, spec.learn_cw,
                                   spec.learned_lifting_z);
    dec.setCommandedRungIndex(spec.commanded_idx);
    if (spec.observed_declining_descriptor) {
        dec.noteDescriptorRungForTesting(spec.latch_mod, spec.latch_rate);
    }

    BurstResult r;
    dec.setBurstOutcomeUnknownCallback([&]() { ++r.unknown_outcomes; });
    r.wire_frame_samples = static_cast<size_t>(
        enc.getWaveform()->getMinSamplesForCWCount(spec.wire_cw));
    size_t fed = 0;
    dec.setAnchoredBurstNoGroupCallback([&](bool payload_seen) {
        ++r.backstop_callbacks;
        r.backstop_payload_seen = payload_seen;
        r.group_before_backstop = r.fired;
        r.backstop_fed = fed;
    });
    dec.setBurstGroupCallback([&](uint16_t, const std::vector<Bytes>& got, bool all_ok,
                                  float, uint16_t mask, bool, uint8_t gsize, bool) {
        if (r.fired) return;
        r.fired = true;
        r.frame_mask = mask;
        r.all_ok = all_ok;
        r.group_size = gsize;
        r.samples_at_callback = fed;
        r.air_remaining_at_callback = dec.burstAirSamplesRemaining();
        r.active_lifting_z = dec.activeBurstLiftingZ();
        r.active_erasure_soft_bits = dec.burstErasureSoftBitsForTesting();
        r.burst_min_block_samples = dec.burstMinBlockSamplesForTesting();
        for (const auto& want : frames) {
            for (const auto& g : got) {
                if (sameLogicalDataFrame(g, want)) { ++r.frames_recovered; break; }
            }
        }
    });

    constexpr size_t kChunk = 4800;
    for (size_t pos = 0; pos < audio.size() && !r.fired; pos += kChunk) {
        const size_t len = std::min(kChunk, audio.size() - pos);
        dec.feedAudio(audio.data() + pos, len);
        fed += len;
        dec.processBuffer();
        if (dec.anchoredBurstBackstopArmedForTesting()) {
            r.backstop_arm_abs =
                dec.anchoredBurstBackstopArmAbsForTesting();
        }
    }
    if (spec.await_anchored_backstop && r.backstop_callbacks == 0) {
        // The fixture's normal trailing silence is usually enough, but derive an
        // explicit full-window extension so the regression cannot depend on burst
        // length or feed chunking.
        std::vector<float> zeros(4800, 0.0f);
        uint64_t remaining = kAnchoredBackstopWindowSamples + zeros.size();
        while (remaining > 0 && r.backstop_callbacks == 0) {
            const size_t n = static_cast<size_t>(
                std::min<uint64_t>(remaining, zeros.size()));
            dec.feedAudio(zeros.data(), n);
            fed += n;
            dec.processBuffer();
            remaining -= n;
            if (dec.anchoredBurstBackstopArmedForTesting()) {
                r.backstop_arm_abs =
                    dec.anchoredBurstBackstopArmAbsForTesting();
            }
        }
    }
    r.truncated_holds = dec.truncatedBurstHoldsForTesting();
    r.commanded_arms = dec.commandedGeometryArmsForTesting();
    r.air_gate_span_at_arm =
        dec.commandedGeometryAirGateSpanAtArmForTesting();
    r.truncation_hold_active = dec.truncationHoldActiveForTesting();
    return r;
}

// ============================================================================
// TEST (E): a sign-only marker is not enough to manufacture a burst group.
// ============================================================================
// A standalone repair can carry a full OFDM anchor. If noise rotates the LTS sign,
// it looks exactly like a group marker even though no descriptor preceded it and no
// continuation member follows it. The historical path filled the declared remainder
// from trailing silence and emitted a synthetic 0/N group callback. Keep the marked
// LTS but erase frame 1's data symbols to reproduce that exact 0/N shape. The paired
// arm sends the real second member, whose CRC-valid physical-tail flag proves the
// group's exact geometry. Process-valid soft grids alone are deliberately not proof:
// a stale lifting size can slice one long member into several plausible short ones.
MarkerProvenanceResult runMarkerProvenanceTrial(bool include_second_member,
                                                bool corrupt_tail = false) {
    constexpr int kGroup = 2;
    constexpr int kCw = 4;

    StreamingEncoder enc;
    enc.setMode(protocol::WaveformMode::OFDM_CHIRP);
    enc.setOFDMConfig(ofdmGeometry(Modulation::QPSK, CodeRate::R1_4));
    enc.setDataMode(Modulation::QPSK, CodeRate::R1_4);
    enc.setFixedFrameCodewords(kCw);
    enc.setBurstInterleave(false);
    enc.setBurstInterleaveGroupSize(kGroup);
    enc.setBurstDescriptorEnabled(false);  // marker has no descriptor provenance

    const auto frames = makeGroupFrames(kGroup, CodeRate::R1_4, kCw);
    auto burst = enc.encodeBurstLight(
        frames, BurstAnchorOptions{/*force_full_group_start=*/true,
                                   /*keep_skip_streak=*/false});
    EXPECT(!burst.empty());

    // The second member is exactly one light [LTS + data] block. Therefore the
    // prefix before it is the complete marked first member, including its full
    // chirp anchor. This derives the cut from production's own waveform geometry.
    const size_t member_samples = static_cast<size_t>(
        enc.getWaveform()->getMinSamplesForCWCount(kCw));
    EXPECT(burst.size() > member_samples);
    const size_t first_member_samples = burst.size() - member_samples;

    if (!include_second_member) {
        burst.resize(first_member_samples);
        // Preserve the full preamble (and its negated first LTS marker), but erase
        // all data symbols. Before the provenance gate this finalized as 0/2 once
        // the following silence was sliced as the missing member.
        const size_t full_preamble_samples = enc.getWaveform()->generatePreamble().size();
        EXPECT(full_preamble_samples < burst.size());
        std::fill(burst.begin() + static_cast<std::ptrdiff_t>(full_preamble_samples),
                  burst.end(), 0.0f);
    } else if (corrupt_tail) {
        // Preserve the second member's LTS and signal energy, but replace every
        // payload sample with deterministic broadband noise. Demodulation remains
        // substantive while LDPC/CRC rejects the DATA, so its tail flag is
        // unavailable. Reaching configured N=2 must abandon the still-unproven
        // candidate without publishing an ACK-driving callback.
        const size_t light_preamble_samples =
            enc.getWaveform()->generateDataPreamble().size();
        const size_t tail_data_start = first_member_samples + light_preamble_samples;
        EXPECT(tail_data_start < burst.size());
        uint32_t noise_state = 0xD15EA5Eu;
        for (size_t i = tail_data_start; i < burst.size(); ++i) {
            noise_state = noise_state * 1664525u + 1013904223u;
            burst[i] = (noise_state & 0x80000000u) ? 0.10f : -0.10f;
        }
    }

    const auto audio = withSilence(burst);

    StreamingDecoder dec;
    dec.setConnectedOFDMMode(protocol::WaveformMode::OFDM_CHIRP,
                             ofdmGeometry(Modulation::QPSK, CodeRate::R1_4),
                             Modulation::QPSK, CodeRate::R1_4);
    dec.setFixedFrameCodewords(kCw);
    dec.setBurstInterleave(false);
    dec.setBurstInterleaveGroupSize(kGroup);
    dec.applyPendingConfigForTesting();
    dec.expectFullOFDMAnchorOnce();

    MarkerProvenanceResult r;
    dec.setBurstOutcomeUnknownCallback([&]() { ++r.unknown_outcomes; });
    dec.setBurstGroupCallback([&](uint16_t, const std::vector<Bytes>& got, bool all_ok,
                                  float, uint16_t mask, bool, uint8_t gsize,
                                  bool geometry_proven) {
        ++r.callbacks;
        r.frame_mask = mask;
        r.all_ok = all_ok;
        r.geometry_proven = geometry_proven;
        r.group_size = gsize;
        for (const auto& want : frames) {
            for (const auto& actual : got) {
                if (sameLogicalDataFrame(actual, want)) {
                    ++r.frames_recovered;
                    break;
                }
            }
        }
    });

    constexpr size_t kChunk = 4800;
    for (size_t pos = 0; pos < audio.size(); pos += kChunk) {
        const size_t len = std::min(kChunk, audio.size() - pos);
        dec.feedAudio(audio.data() + pos, len);
        dec.processBuffer();
    }
    return r;
}

// Reproduce the live collision directly: the sender emits long-Z81 members while
// the receiver, having missed the descriptor, still slices with short Z27 geometry.
// One real member then contains several process-valid short slices. Those slices
// must never promote an unproven sign marker into an ACK-driving burst group.
MarkerProvenanceResult runWrongLiftingUnprovenMarkerTrial() {
    constexpr int kGroup = 5;
    constexpr int kCw = 3;
    constexpr int kTxZ = 81;

    StreamingEncoder enc;
    enc.setMode(protocol::WaveformMode::OFDM_CHIRP);
    enc.setOFDMConfig(ofdmGeometry(Modulation::QPSK, CodeRate::R3_4));
    enc.setDataMode(Modulation::QPSK, CodeRate::R3_4);
    enc.setFixedFrameCodewords(kCw);
    enc.setLDPCLiftingZ(kTxZ);
    enc.setBurstInterleave(false);
    enc.setBurstInterleaveGroupSize(kGroup);
    enc.setBurstDescriptorEnabled(false);

    std::vector<Bytes> frames;
    for (int f = 0; f < kGroup; ++f) {
        frames.push_back(v2::makeFixedDataFrame(
            "ALPHA", "BRAVO", static_cast<uint16_t>(f),
            Bytes(20, static_cast<uint8_t>(0x60 + f)), CodeRate::R3_4,
            kCw, kTxZ).serialize());
    }
    const auto burst = enc.encodeBurstLight(
        frames, BurstAnchorOptions{/*force_full_group_start=*/true,
                                   /*keep_skip_streak=*/false});
    EXPECT(!burst.empty());

    StreamingDecoder dec;
    dec.setConnectedOFDMMode(protocol::WaveformMode::OFDM_CHIRP,
                             ofdmGeometry(Modulation::QPSK, CodeRate::R3_4),
                             Modulation::QPSK, CodeRate::R3_4);
    dec.setFixedFrameCodewords(kCw);
    dec.setBurstInterleave(false);
    dec.setBurstInterleaveGroupSize(kGroup);
    dec.applyPendingConfigForTesting();
    dec.expectFullOFDMAnchorOnce();

    EXPECT(enc.getLDPCLiftingZ() == kTxZ);
    EXPECT(dec.activeBurstLiftingZ() == 27);
    EXPECT(static_cast<size_t>(enc.getWaveform()->getMinSamplesForCWCount(kCw)) >
           dec.waveformFrameSamplesForTesting(kCw));

    MarkerProvenanceResult r;
    dec.setBurstOutcomeUnknownCallback([&]() { ++r.unknown_outcomes; });
    dec.setBurstGroupCallback([&](uint16_t, const std::vector<Bytes>& got,
                                  bool all_ok, float, uint16_t mask, bool,
                                  uint8_t gsize, bool geometry_proven) {
        ++r.callbacks;
        r.frame_mask = mask;
        r.all_ok = all_ok;
        r.geometry_proven = geometry_proven;
        r.group_size = gsize;
        r.frames_recovered += static_cast<int>(got.size());
    });

    const auto audio = withSilence(burst);
    constexpr size_t kChunk = 4800;
    for (size_t pos = 0; pos < audio.size(); pos += kChunk) {
        const size_t len = std::min(kChunk, audio.size() - pos);
        dec.feedAudio(audio.data() + pos, len);
        dec.processBuffer();
    }
    return r;
}

// A missed descriptor leaves the receiver on its configured fallback group size.
// The wire may legitimately be shorter (the sender only had four ARQ frames left),
// and the last independently-decodable DATA frame carries a CRC-protected physical
// tail marker. Production fixed_default_04 demonstrated the pre-fix failure exactly:
// all four real frames decoded, then the accumulator sliced two blocks of silence to
// satisfy stale fallback N=6 and reported a delayed 4/6 partial group. The tail is
// stronger evidence than guessed geometry, so it must finalize the collected 4/4
// immediately. The decode counter also pins the early-decode cache alignment: frames
// 2..4 are decoded as they arrive and must not be decoded a second time at finalize.
PhysicalTailGroupResult runMissedDescriptorPhysicalTailTrial(bool corrupt_tail = false) {
    constexpr int kWireGroup = 4;
    constexpr int kFallbackGroup = 6;
    constexpr int kCw = 4;

    StreamingEncoder enc;
    enc.setMode(protocol::WaveformMode::OFDM_CHIRP);
    enc.setOFDMConfig(ofdmGeometry(Modulation::QPSK, CodeRate::R1_4));
    enc.setDataMode(Modulation::QPSK, CodeRate::R1_4);
    enc.setFixedFrameCodewords(kCw);
    enc.setBurstInterleave(false);
    enc.setBurstInterleaveGroupSize(kWireGroup);
    enc.setBurstDescriptorEnabled(false);  // receiver must use stale fallback geometry

    const auto frames = makeGroupFrames(kWireGroup, CodeRate::R1_4, kCw);
    auto burst = enc.encodeBurstLight(
        frames, BurstAnchorOptions{/*force_full_group_start=*/true,
                                   /*keep_skip_streak=*/false});
    EXPECT(!burst.empty());
    if (corrupt_tail) {
        const size_t member_samples = static_cast<size_t>(
            enc.getWaveform()->getMinSamplesForCWCount(kCw));
        const size_t light_preamble_samples =
            enc.getWaveform()->generateDataPreamble().size();
        EXPECT(burst.size() > member_samples);
        const size_t tail_data_start =
            burst.size() - member_samples + light_preamble_samples;
        EXPECT(tail_data_start < burst.size());
        uint32_t noise_state = 0x51A1E123u;
        for (size_t i = tail_data_start; i < burst.size(); ++i) {
            noise_state = noise_state * 1664525u + 1013904223u;
            burst[i] = (noise_state & 0x80000000u) ? 0.10f : -0.10f;
        }
    }
    const auto audio = withSilence(burst);

    StreamingDecoder dec;
    dec.setConnectedOFDMMode(protocol::WaveformMode::OFDM_CHIRP,
                             ofdmGeometry(Modulation::QPSK, CodeRate::R1_4),
                             Modulation::QPSK, CodeRate::R1_4);
    dec.setFixedFrameCodewords(kCw);
    dec.setBurstInterleave(false);
    dec.setBurstInterleaveGroupSize(kFallbackGroup);
    dec.applyPendingConfigForTesting();
    dec.expectFullOFDMAnchorOnce();

    auto& profile = ultra::timing::globalDecoderProfile();
    profile.reset();

    PhysicalTailGroupResult r;
    r.physical_end_sample = 48000 + burst.size();  // withSilence() lead-in
    size_t fed = 0;
    dec.setBurstOutcomeUnknownCallback([&]() { ++r.unknown_outcomes; });
    dec.setBurstGroupCallback([&](uint16_t, const std::vector<Bytes>& got, bool all_ok,
                                  float, uint16_t mask, bool, uint8_t gsize,
                                  bool geometry_proven) {
        ++r.callbacks;
        r.frame_mask = mask;
        r.all_ok = all_ok;
        r.geometry_proven = geometry_proven;
        r.group_size = gsize;
        r.samples_at_callback = fed;
        for (const auto& want : frames) {
            for (const auto& actual : got) {
                if (sameLogicalDataFrame(actual, want)) {
                    ++r.frames_recovered;
                    break;
                }
            }
        }
    });

    constexpr size_t kChunk = 4800;
    for (size_t pos = 0; pos < audio.size() && r.callbacks == 0; pos += kChunk) {
        const size_t len = std::min(kChunk, audio.size() - pos);
        dec.feedAudio(audio.data() + pos, len);
        fed += len;
        dec.processBuffer();
    }
    r.fixed_decode_calls =
        profile.decode_fixed_frame_total.count.load(std::memory_order_relaxed);
    return r;
}

// A full anchor is also the normal wire shape for a singleton repair. It need
// not carry FINAL, and its ARQ timeout is shorter than the conservative unknown-
// burst ceiling. A successful classic decode at that same anchor must therefore
// clear the provisional gate before the protocol callback runs.
StandaloneGateResult runStandaloneNonFinalGateTrial() {
    constexpr int kCw = 4;

    auto frame = v2::makeFixedDataFrame(
        "ALPHA", "BRAVO", 41, Bytes(20, 0x5A), CodeRate::R1_4, kCw);
    EXPECT((frame.flags & v2::Flags::FINAL) == 0);
    const Bytes serialized = frame.serialize();

    StreamingEncoder enc;
    enc.setMode(protocol::WaveformMode::OFDM_CHIRP);
    enc.setOFDMConfig(ofdmGeometry(Modulation::QPSK, CodeRate::R1_4));
    enc.setDataMode(Modulation::QPSK, CodeRate::R1_4);
    enc.setFixedFrameCodewords(kCw);
    const auto audio = withSilence(enc.encodeFrame(serialized));

    StreamingDecoder dec;
    dec.setConnectedOFDMMode(protocol::WaveformMode::OFDM_CHIRP,
                             ofdmGeometry(Modulation::QPSK, CodeRate::R1_4),
                             Modulation::QPSK, CodeRate::R1_4);
    dec.setFixedFrameCodewords(kCw);
    dec.applyPendingConfigForTesting();
    dec.expectFullOFDMAnchorOnce();

    StandaloneGateResult r;
    dec.setAnchoredBurstNoGroupCallback([&](bool payload_seen) {
        ++r.backstop_callbacks;
        r.backstop_payload_seen = payload_seen;
    });
    dec.setFrameCallback([&](const DecodeResult& got) {
        if (!got.success || got.frame_data != serialized) return;
        ++r.callbacks;
        r.physical_complete = got.physical_turn_complete;
        r.air_remaining_at_callback = dec.burstAirSamplesRemaining();
    });

    constexpr size_t kChunk = 4800;
    for (size_t pos = 0; pos < audio.size() && r.callbacks == 0; pos += kChunk) {
        const size_t len = std::min(kChunk, audio.size() - pos);
        dec.feedAudio(audio.data() + pos, len);
        dec.processBuffer();
    }
    // A successful singleton is the completed physical turn.  Its accepted
    // anchor must not manufacture an ANCHORED-BURST backstop callback later.
    feedAndProcessSilenceSamples(
        dec, kAnchoredBackstopWindowSamples + 4800);
    return r;
}

// Reproduce the collision's decoder shape without relying on ARQ: accept one
// expected full anchor, make that frame's body unreadable, then append a valid
// light DATA member. The later callback must still see the original air gate.
StandaloneGateResult runLaterLightFallbackGateTrial(bool physical_tail) {
    constexpr int kCw = 4;
    auto head = v2::makeFixedDataFrame(
        "ALPHA", "BRAVO", 50, Bytes(20, 0x71), CodeRate::R1_4, kCw);
    auto later = v2::makeFixedDataFrame(
        "ALPHA", "BRAVO", 51, Bytes(20, 0x72), CodeRate::R1_4, kCw);
    if (physical_tail) {
        later.flags |= v2::Flags::PHYSICAL_BURST_END;
    }
    const Bytes later_serialized = later.serialize();

    StreamingEncoder enc;
    enc.setMode(protocol::WaveformMode::OFDM_CHIRP);
    enc.setOFDMConfig(ofdmGeometry(Modulation::QPSK, CodeRate::R1_4));
    enc.setDataMode(Modulation::QPSK, CodeRate::R1_4);
    enc.setFixedFrameCodewords(kCw);
    auto failed_head = enc.encodeFrame(head.serialize());
    const size_t preamble_samples = enc.getWaveform()->generatePreamble().size();
    EXPECT(preamble_samples < failed_head.size());
    std::fill(failed_head.begin() + static_cast<std::ptrdiff_t>(preamble_samples),
              failed_head.end(), 0.0f);
    const auto light_later = enc.encodeFrameLight(later_serialized);
    failed_head.insert(failed_head.end(), light_later.begin(), light_later.end());
    const auto audio = withSilence(failed_head);

    StreamingDecoder dec;
    dec.setConnectedOFDMMode(protocol::WaveformMode::OFDM_CHIRP,
                             ofdmGeometry(Modulation::QPSK, CodeRate::R1_4),
                             Modulation::QPSK, CodeRate::R1_4);
    dec.setFixedFrameCodewords(kCw);
    dec.applyPendingConfigForTesting();
    dec.expectFullOFDMAnchorOnce();

    StandaloneGateResult r;
    dec.setAnchoredBurstNoGroupCallback([&](bool payload_seen) {
        ++r.backstop_callbacks;
        r.backstop_payload_seen = payload_seen;
    });
    dec.setFrameCallback([&](const DecodeResult& got) {
        if (!got.success || got.frame_data != later_serialized) return;
        ++r.callbacks;
        r.physical_complete = got.physical_turn_complete;
        r.air_remaining_at_callback = dec.burstAirSamplesRemaining();
    });
    constexpr size_t kChunk = 4800;
    for (size_t pos = 0; pos < audio.size() && r.callbacks == 0; pos += kChunk) {
        const size_t len = std::min(kChunk, audio.size() - pos);
        dec.feedAudio(audio.data() + pos, len);
        dec.processBuffer();
    }
    if (r.air_remaining_at_callback > 0) {
        feedSilenceSamples(dec, r.air_remaining_at_callback - 1);
        r.one_sample_before_deadline = dec.burstAirSamplesRemaining();
        feedSilenceSamples(dec, 1);
        r.at_deadline = dec.burstAirSamplesRemaining();
    }
    // An unmarked later member retains the recovery backstop. A CRC-protected
    // physical tail clears it and authorizes the immediate cumulative ACK path.
    feedAndProcessSilenceSamples(
        dec, kAnchoredBackstopWindowSamples + 4800);
    r.classic_long_arms = dec.classicLongGeometryArmsForTesting();
    r.active_lifting_z_after = dec.activeBurstLiftingZ();
    return r;
}

// Live MPG@20 residual: the accepted full anchor's descriptor/body is unreadable,
// the marked DATA head is never accepted, but a later ordinary light-LTS member is
// strong. The decoder begins at stale Z27 even though an earlier descriptor taught
// it this exact 8PSK R2/3 cw4/Z81 rung. After control-first rejects the candidate,
// it must wait for/decode one full long member through the classic callback path,
// use PHYSICAL_BURST_END as the only turn proof, and restore Z27 afterward.
StandaloneGateResult runClassicLongFallbackGateTrial() {
    constexpr int kCw = 4;
    constexpr int kZ = 81;
    constexpr Modulation kMod = Modulation::QAM8;
    constexpr CodeRate kRate = CodeRate::R2_3;

    auto head = v2::makeFixedDataFrame(
        "ALPHA", "BRAVO", 60, Bytes(40, 0x81), kRate, kCw, kZ);
    auto later = v2::makeFixedDataFrame(
        "ALPHA", "BRAVO", 61, Bytes(40, 0x82), kRate, kCw, kZ);
    later.flags |= v2::Flags::PHYSICAL_BURST_END;
    const Bytes later_serialized = later.serialize();

    StreamingEncoder enc;
    enc.setMode(protocol::WaveformMode::OFDM_CHIRP);
    enc.setOFDMConfig(ofdmGeometry(kMod, kRate));
    enc.setDataMode(kMod, kRate);
    enc.setFixedFrameCodewords(kCw);
    enc.setLDPCLiftingZ(kZ);
    enc.setBurstInterleave(false);

    auto failed_head = enc.encodeFrame(head.serialize());
    const size_t preamble_samples = enc.getWaveform()->generatePreamble().size();
    EXPECT(preamble_samples < failed_head.size());
    std::fill(failed_head.begin() + static_cast<std::ptrdiff_t>(preamble_samples),
              failed_head.end(), 0.0f);
    const auto light_later = enc.encodeFrameLight(later_serialized);
    failed_head.insert(failed_head.end(), light_later.begin(), light_later.end());
    const auto audio = withSilence(failed_head);

    StreamingDecoder dec;
    dec.setConnectedOFDMMode(protocol::WaveformMode::OFDM_CHIRP,
                             ofdmGeometry(kMod, kRate), kMod, kRate);
    dec.setFixedFrameCodewords(kCw);
    dec.setBurstInterleave(false);
    dec.applyPendingConfigForTesting();
    dec.seedRungGeometryForTesting(kMod, kRate, kCw, kZ);
    dec.setCommandedRungIndex(protocol::kRungIdxQam8R23);
    dec.expectFullOFDMAnchorOnce();
    EXPECT(dec.activeBurstLiftingZ() == 27);

    StandaloneGateResult r;
    dec.setDataSyncAcceptedCallback([&](float correlation) {
        ++r.data_sync_accepts;
        r.last_data_sync_correlation =
            std::max(r.last_data_sync_correlation, correlation);
    });
    dec.setAnchoredBurstNoGroupCallback([&](bool payload_seen) {
        ++r.backstop_callbacks;
        r.backstop_payload_seen = payload_seen;
    });
    dec.setFrameCallback([&](const DecodeResult& got) {
        if (!got.success || got.frame_data != later_serialized) return;
        ++r.callbacks;
        r.physical_complete = got.physical_turn_complete;
        r.air_remaining_at_callback = dec.burstAirSamplesRemaining();
    });

    constexpr size_t kChunk = 4800;
    for (size_t pos = 0; pos < audio.size() && r.callbacks == 0; pos += kChunk) {
        const size_t len = std::min(kChunk, audio.size() - pos);
        dec.feedAudio(audio.data() + pos, len);
        dec.processBuffer();
    }
    r.classic_long_arms = dec.classicLongGeometryArmsForTesting();
    r.active_lifting_z_after = dec.activeBurstLiftingZ();

    // A CRC-valid physical tail must retire the accepted-anchor backstop; no
    // timer-driven substitute ACK should appear after the real callback.
    feedAndProcessSilenceSamples(dec, kAnchoredBackstopWindowSamples + 4800);
    return r;
}

bool classicLongEligibilityTrial(bool burst_interleave,
                                 bool seed_geometry,
                                 int learned_z,
                                 float correlation,
                                 uint64_t prior_group_start_abs = 0,
                                 uint64_t anchor_abs = 0,
                                 uint64_t candidate_abs = 1) {
    constexpr int kCw = 4;
    constexpr Modulation kMod = Modulation::QAM8;
    constexpr CodeRate kRate = CodeRate::R2_3;

    StreamingDecoder dec;
    dec.setConnectedOFDMMode(protocol::WaveformMode::OFDM_CHIRP,
                             ofdmGeometry(kMod, kRate), kMod, kRate);
    dec.setFixedFrameCodewords(kCw);
    dec.setBurstInterleave(burst_interleave);
    dec.applyPendingConfigForTesting();
    if (seed_geometry) {
        dec.seedRungGeometryForTesting(kMod, kRate, kCw, learned_z);
    }
    dec.setCommandedRungIndex(protocol::kRungIdxQam8R23);
    return dec.classicLongGeometryEligibleForTesting(
        correlation, /*burst_latched=*/false,
        prior_group_start_abs, anchor_abs, candidate_abs);
}

bool classicLongSessionStateClearedTrial() {
    constexpr int kCw = 4;
    constexpr Modulation kMod = Modulation::QAM8;
    constexpr CodeRate kRate = CodeRate::R2_3;

    StreamingDecoder dec;
    dec.setConnectedOFDMMode(protocol::WaveformMode::OFDM_CHIRP,
                             ofdmGeometry(kMod, kRate), kMod, kRate);
    dec.setFixedFrameCodewords(kCw);
    dec.setBurstInterleave(false);
    dec.applyPendingConfigForTesting();
    dec.seedRungGeometryForTesting(kMod, kRate, kCw, /*lifting_z=*/81);
    dec.setCommandedRungIndex(protocol::kRungIdxQam8R23);
    EXPECT(dec.classicLongGeometryEligibleForTesting(
        /*correlation=*/0.95f, /*burst_latched=*/false));

    // This is the true peer/session boundary. reset() alone is intentionally not
    // used as the discriminator because connected TX echo-clear calls it every
    // turnaround and must retain this session's learned table.
    dec.setMode(protocol::WaveformMode::MC_DPSK, /*connected=*/false);
    EXPECT(dec.commandedRungIndexForTesting() == protocol::kRungIdxNone);

    dec.setConnectedOFDMMode(protocol::WaveformMode::OFDM_CHIRP,
                             ofdmGeometry(kMod, kRate), kMod, kRate);
    dec.setFixedFrameCodewords(kCw);
    dec.setBurstInterleave(false);
    dec.applyPendingConfigForTesting();

    // Re-issue the same rung in the new session so this final predicate does not
    // pass merely because the old commanded-rung atomic was cleared. A fresh
    // command must still have no Z81 geometry until this peer has announced it on
    // the wire; otherwise the first missed Z27 descriptor is sliced three-times
    // long from the previous peer/session's learned table.
    dec.setCommandedRungIndex(protocol::kRungIdxQam8R23);
    EXPECT(dec.commandedRungIndexForTesting() == protocol::kRungIdxQam8R23);
    return !dec.classicLongGeometryEligibleForTesting(
        /*correlation=*/0.95f, /*burst_latched=*/false);
}

bool learnedRungGeometrySessionBoundarySynchronizationTrial() {
    constexpr Modulation kMod = Modulation::QAM8;
    constexpr CodeRate kRate = CodeRate::R2_3;
    constexpr std::array<int, 2> kEmpty{0, 27};
    constexpr std::array<int, 2> kLearned{4, 81};

    StreamingDecoder dec;
    dec.setMode(protocol::WaveformMode::MC_DPSK, /*connected=*/true);
    dec.seedRungGeometryForTesting(kMod, kRate, kLearned[0], kLearned[1]);
    if (dec.learnedRungGeometryForTesting(kMod, kRate) != kLearned) {
        return false;
    }

    // reset() is also the connected TX echo-clear path. It is deliberately not a
    // peer/session boundary and must retain geometry learned from this peer.
    const uint64_t epoch_before_echo_reset =
        dec.learnedRungGeometrySessionEpochForTesting();
    dec.reset(/*reset_doppler_coherence=*/false);
    if (dec.learnedRungGeometryForTesting(kMod, kRate) != kLearned ||
        dec.learnedRungGeometrySessionEpochForTesting() !=
            epoch_before_echo_reset) {
        return false;
    }

    // An old decode can finish after the protocol thread clears the session. The
    // mutex removes the data race; the epoch also prevents that late descriptor
    // from repopulating the just-cleared table for the next peer.
    dec.setMode(protocol::WaveformMode::MC_DPSK, /*connected=*/false);
    if (dec.seedRungGeometryForSessionForTesting(
            kMod, kRate, kLearned[0], kLearned[1], epoch_before_echo_reset) ||
        dec.learnedRungGeometryForTesting(kMod, kRate) != kEmpty) {
        return false;
    }
    dec.setMode(protocol::WaveformMode::MC_DPSK, /*connected=*/true);
    dec.seedRungGeometryForTesting(kMod, kRate, kLearned[0], kLearned[1]);

    // Exercise the real connected->disconnected lifecycle transition against a
    // synchronized reader. The value may be either the old session's complete
    // tuple or the fully cleared tuple, never a torn cw/Z combination. This is a
    // meaningful ThreadSanitizer gate because setMode() and the snapshot execute
    // on distinct threads through the same production mutex.
    std::atomic<bool> keep_reading{true};
    std::atomic<bool> reader_started{false};
    std::atomic<uint64_t> snapshots{0};
    std::atomic<uint64_t> invalid_snapshots{0};
    std::thread reader([&]() {
        reader_started.store(true, std::memory_order_release);
        while (keep_reading.load(std::memory_order_acquire)) {
            const auto geometry = dec.learnedRungGeometryForTesting(kMod, kRate);
            if (geometry != kEmpty && geometry != kLearned) {
                invalid_snapshots.fetch_add(1, std::memory_order_relaxed);
            }
            snapshots.fetch_add(1, std::memory_order_release);
        }
    });
    while (!reader_started.load(std::memory_order_acquire)) {
        std::this_thread::yield();
    }
    while (snapshots.load(std::memory_order_acquire) == 0) {
        std::this_thread::yield();
    }

    for (int session = 0; session < 256; ++session) {
        dec.setMode(protocol::WaveformMode::MC_DPSK, /*connected=*/false);
        dec.setMode(protocol::WaveformMode::MC_DPSK, /*connected=*/true);
        dec.seedRungGeometryForTesting(kMod, kRate, kLearned[0], kLearned[1]);
    }
    keep_reading.store(false, std::memory_order_release);
    reader.join();

    dec.setMode(protocol::WaveformMode::MC_DPSK, /*connected=*/false);
    return snapshots.load(std::memory_order_relaxed) > 0 &&
           invalid_snapshots.load(std::memory_order_relaxed) == 0 &&
           dec.learnedRungGeometryForTesting(kMod, kRate) == kEmpty;
}

// A detected expected anchor with an unreadable body is the precondition for
// later light duplicates taking the classic callback path. Pin both the initial
// provisional arm and its reset lifecycle with a real chirp/LTS encode.
// Frame-structure acquisition (ULTRA_BURST_ACQ=structure, the production default) locks on ANY
// frame's LTS pair: after a fade takes a group's head, on a member in mid-burst. Only a dual chirp
// proves a STANDALONE frame at the anchor (a single OFDM frame always goes out behind one), so a
// member decoded at a structure anchor must keep the physical air gate - no immediate ACK - and the
// burst's last frame, stamped PHYSICAL_BURST_END, ends the turn. GUI ITU Good 5 dB seed 44: the lock
// landed on the third of four frames and its ACK keyed 2.3 s before the sender's key-down.
struct StructureMemberResult {
    int member_callbacks = 0;
    bool member_turn_complete = true;
    uint64_t member_air_remaining = 0;
    int tail_callbacks = 0;
    bool tail_turn_complete = false;
};

StructureMemberResult runStructureMemberAnchorTrial() {
    constexpr int kGroup = 4;
    constexpr int kCw = 4;
    ModemConfig geom = presets::balanced();  // production: MEDIUM CP
    geom.modulation = Modulation::QPSK;
    geom.code_rate = CodeRate::R1_2;
    geom.use_pilots = true;
    geom.pilot_spacing =
        ofdm_link_adaptation::recommendedPilotSpacing(Modulation::QPSK, CodeRate::R1_2);

    StreamingEncoder enc;
    enc.setMode(protocol::WaveformMode::OFDM_CHIRP);
    enc.setOFDMConfig(geom);
    enc.setDataMode(Modulation::QPSK, CodeRate::R1_2);
    enc.setFixedFrameCodewords(kCw);
    enc.setBurstInterleave(false);
    const auto frames = makeGroupFrames(kGroup, CodeRate::R1_2, kCw);
    auto burst = enc.encodeBurstLight(
        frames, BurstAnchorOptions{/*force_full_group_start=*/false,
                                   /*keep_skip_streak=*/false});
    const size_t member_samples = static_cast<size_t>(
        enc.getWaveform()->getMinSamplesForCWCount(kCw));
    EXPECT(burst.size() > 2 * member_samples);
    // The fade: the descriptor and the first two members never arrive.
    std::fill(burst.begin(), burst.end() - static_cast<std::ptrdiff_t>(2 * member_samples), 0.0f);
    auto audio = withSilence(burst);
    // A receiver's own noise floor, 30 dB under the burst (deterministic).
    double power = 0.0;
    for (size_t i = audio.size() - 2 * member_samples - 192000; i < audio.size() - 192000; ++i) {
        power += static_cast<double>(audio[i]) * audio[i];
    }
    const float sigma = static_cast<float>(
        std::sqrt(power / static_cast<double>(2 * member_samples)) * 0.0316);
    uint64_t state = 0x5EEDu;
    for (auto& x : audio) {
        float sum = 0.0f;  // Irwin-Hall(12) - 6: unit variance
        for (int k = 0; k < 12; ++k) {
            state = state * 6364136223846793005ull + 1442695040888963407ull;
            sum += static_cast<float>(state >> 40) / 16777216.0f;
        }
        x += sigma * (sum - 6.0f);
    }

    StreamingDecoder dec;
    dec.setConnectedOFDMMode(protocol::WaveformMode::OFDM_CHIRP, geom, Modulation::QPSK,
                             CodeRate::R1_2);
    dec.setFixedFrameCodewords(kCw);
    dec.setBurstInterleave(false);
    dec.applyPendingConfigForTesting();
    dec.expectFullOFDMAnchorOnce();

    StructureMemberResult r;
    dec.setFrameCallback([&](const DecodeResult& got) {
        if (!got.success) return;
        if (sameLogicalDataFrame(got.frame_data, frames[2])) {
            ++r.member_callbacks;
            r.member_turn_complete = got.physical_turn_complete;
            r.member_air_remaining = dec.burstAirSamplesRemaining();
        } else if (sameLogicalDataFrame(got.frame_data, frames[3])) {
            ++r.tail_callbacks;
            r.tail_turn_complete = got.physical_turn_complete;
        }
    });
    constexpr size_t kChunk = 4800;
    for (size_t pos = 0; pos < audio.size(); pos += kChunk) {
        const size_t len = std::min(kChunk, audio.size() - pos);
        dec.feedAudio(audio.data() + pos, len);
        dec.processBuffer();
    }
    return r;
}

FailedAnchorGateResult runFailedAnchorResetGateTrial() {
    constexpr int kCw = 4;
    auto frame = v2::makeFixedDataFrame(
        "ALPHA", "BRAVO", 42, Bytes(20, 0x6B), CodeRate::R1_4, kCw);

    StreamingEncoder enc;
    enc.setMode(protocol::WaveformMode::OFDM_CHIRP);
    enc.setOFDMConfig(ofdmGeometry(Modulation::QPSK, CodeRate::R1_4));
    enc.setDataMode(Modulation::QPSK, CodeRate::R1_4);
    enc.setFixedFrameCodewords(kCw);
    auto signal = enc.encodeFrame(frame.serialize());
    const size_t preamble_samples = enc.getWaveform()->generatePreamble().size();
    EXPECT(preamble_samples < signal.size());
    std::fill(signal.begin() + static_cast<std::ptrdiff_t>(preamble_samples),
              signal.end(), 0.0f);
    const auto audio = withSilence(signal);

    StreamingDecoder dec;
    dec.setConnectedOFDMMode(protocol::WaveformMode::OFDM_CHIRP,
                             ofdmGeometry(Modulation::QPSK, CodeRate::R1_4),
                             Modulation::QPSK, CodeRate::R1_4);
    dec.setFixedFrameCodewords(kCw);
    dec.applyPendingConfigForTesting();
    dec.expectFullOFDMAnchorOnce();

    FailedAnchorGateResult r;
    dec.setAnchoredBurstNoGroupCallback([&](bool) { ++r.backstop_callbacks; });
    dec.setFrameCallback([&](const DecodeResult&) { ++r.callbacks; });
    constexpr size_t kChunk = 4800;
    for (size_t pos = 0; pos < audio.size(); pos += kChunk) {
        const size_t len = std::min(kChunk, audio.size() - pos);
        dec.feedAudio(audio.data() + pos, len);
        dec.processBuffer();
    }
    r.before_reset = dec.burstAirSamplesRemaining();
    feedAndProcessSilenceSamples(
        dec, kAnchoredBackstopWindowSamples + 4800);
    dec.reset(/*reset_doppler_coherence=*/false);
    r.after_reset = dec.burstAirSamplesRemaining();
    return r;
}

void checkProvisionalCeilingCoversWorstPhysicalVector() {
    constexpr Modulation kMod = Modulation::QPSK;
    constexpr CodeRate kRate = CodeRate::R2_3;
    constexpr int kCw = 8;
    size_t group = 1;
    while (group < protocol::connection_policy::kToneBurstAckWindowCapFrames &&
           protocol::connection_policy::wideOFDMBurstAirtimeMs(
               kMod, kRate, group + 1, kCw) <=
               protocol::connection_policy::kMaxBurstAirtimeCeilingMs) {
        ++group;
    }

    StreamingEncoder enc;
    enc.setMode(protocol::WaveformMode::OFDM_CHIRP);
    enc.setOFDMConfig(ofdmGeometry(kMod, kRate));
    enc.setDataMode(kMod, kRate);
    enc.setFixedFrameCodewords(kCw);
    enc.setBurstInterleave(false);
    enc.setBurstInterleaveGroupSize(static_cast<int>(group));
    enc.setBurstDescriptorEnabled(true);
    enc.setBurstDescriptorIdentity("ALPHA", "BRAVO");
    // Worst wire shape: a full BURST_HEADER descriptor followed by another full
    // reliability anchor at the marked DATA group start.
    const auto physical = enc.encodeBurstLight(
        makeGroupFrames(static_cast<int>(group), kRate, kCw),
        BurstAnchorOptions{/*force_full_group_start=*/true,
                           /*keep_skip_streak=*/false});

    const uint64_t training_offset =
        protocol::connection_policy::kWideOFDMFullAnchorExtraSamples;
    EXPECT(physical.size() > training_offset);
    const uint64_t remaining_from_descriptor_training =
        static_cast<uint64_t>(physical.size()) - training_offset;
    const uint64_t provisional_ceiling_samples =
        static_cast<uint64_t>(
            protocol::connection_policy::maxWideOFDMPhysicalTurnAfterAnchorTrainingMs()) *
        protocol::connection_policy::kOFDMSampleRate / 1000u;
    EXPECT(remaining_from_descriptor_training <= provisional_ceiling_samples);
}

void report(const char* name, const BurstResult& r) {
    std::fprintf(stderr,
                 "  %-28s fired=%d mask=0x%02X all_ok=%d group=%u recovered=%d "
                 "samples_at_cb=%zu air_rem=%llu trunc_holds=%u cmd_arms=%u "
                 "arm_air_span=%llu z=%d soft_bits=%zu hold_active=%d unknown=%d\n",
                 name, r.fired ? 1 : 0, r.frame_mask, r.all_ok ? 1 : 0,
                 static_cast<unsigned>(r.group_size), r.frames_recovered,
                 r.samples_at_callback,
                 static_cast<unsigned long long>(r.air_remaining_at_callback),
                 r.truncated_holds, r.commanded_arms,
                 static_cast<unsigned long long>(
                     r.air_gate_span_at_arm),
                 r.active_lifting_z,
                 r.active_erasure_soft_bits,
                 r.truncation_hold_active ? 1 : 0, r.unknown_outcomes);
}

}  // namespace

int main(int argc, char** argv) {
    LogLevel level = LogLevel::ERROR;
    if (const char* env = std::getenv("ULTRA_LOG_LEVEL")) {
        parseLogLevel(env, level);
    }
    setLogLevel(level);

    // Own CTest entry (BurstStructureMemberAnchor) with ULTRA_BURST_ACQ=structure: the mode is
    // read once per process, and every other trial here runs the chirp-per-group path.
    if (argc > 1 && std::strcmp(argv[1], "--structure-member-only") == 0) {
        const StructureMemberResult sm = runStructureMemberAnchorTrial();
        std::fprintf(stderr,
                     "structure member anchor: member callbacks=%d turn_complete=%d "
                     "air_remaining=%llu | tail callbacks=%d turn_complete=%d\n",
                     sm.member_callbacks, sm.member_turn_complete ? 1 : 0,
                     static_cast<unsigned long long>(sm.member_air_remaining),
                     sm.tail_callbacks, sm.tail_turn_complete ? 1 : 0);
        EXPECT(sm.member_callbacks == 1);
        EXPECT(!sm.member_turn_complete);
        EXPECT(sm.member_air_remaining > 0);
        EXPECT(sm.tail_callbacks == 1);
        EXPECT(sm.tail_turn_complete);
        std::fprintf(stderr, "%s\n", g_failures == 0 ? "PASS" : "FAIL");
        return g_failures == 0 ? 0 : 1;
    }

    // Both environment arms run IN ONE PROCESS. For (A), both must exercise the
    // unconditional alignment guard. For (B), the off/on pair remains live
    // fail-before evidence for the separately gated commanded-geometry experiment.
    std::fprintf(stderr, "== (A) truncated group-start frame ==\n");
    const BurstResult a_knob_off = runTruncationTrial(/*knob_on=*/false);
    report("A knob=0 (guard on)", a_knob_off);
    const BurstResult a_knob_on = runTruncationTrial(/*knob_on=*/true);
    report("A knob=1 (guard on)", a_knob_on);

    // (B) The measured failure: a DEMOTE the sender took immediately, whose
    // descriptor we then missed. Wire = commanded = QPSK R1/4 cw4; latched = the
    // previous, faster QPSK R3/4 cw8.
    const MissedDescriptorSpec kAdopted{
        Modulation::QPSK, CodeRate::R1_4, 4,
        Modulation::QPSK, CodeRate::R3_4, 8,
        protocol::kRungIdxQpskR14,
        Modulation::QPSK, CodeRate::R1_4, 4,
        /*observed_declining_descriptor=*/false};
    std::fprintf(stderr, "== (B) missed descriptor / commanded geometry ==\n");
    const BurstResult b_broken = runMissedDescriptorTrial(/*knob_on=*/false, kAdopted);
    report("B knob=0 (pre-fix)", b_broken);
    const BurstResult b_fixed = runMissedDescriptorTrial(/*knob_on=*/true, kAdopted);
    report("B knob=1 (fixed)", b_fixed);

    // (C) ADOPTION GATE — DECLINED. Same demote command, but a descriptor decoded
    // since then announced the OLD rung: direct wire evidence the sender has not
    // taken it (legacy MODE_CHANGE round trip in flight, ladder snap, ULTRA_LOCK_RATE
    // on the far end...). The wire IS the latched rung, so the group decodes 5/5 —
    // unless the resolver arms the command anyway, which is the "destroys a healthy
    // group" regression this gate exists to prevent.
    const MissedDescriptorSpec kDeclined{
        Modulation::QPSK, CodeRate::R3_4, 8,   // wire == latched: a healthy group
        Modulation::QPSK, CodeRate::R3_4, 8,
        protocol::kRungIdxQpskR14,             // ...but a demote is standing
        Modulation::QPSK, CodeRate::R1_4, 4,
        /*observed_declining_descriptor=*/true};
    std::fprintf(stderr, "== (C) command DECLINED by the sender (must not arm) ==\n");
    const BurstResult c_fixed = runMissedDescriptorTrial(/*knob_on=*/true, kDeclined);
    report("C knob=1 declined", c_fixed);

    // (D) ADOPTION GATE — CLIMB. An UP command is DEFERRED by the sender while its TX
    // window drains, and the receiver re-stamps it on every ACK, so it can stand
    // un-adopted for many groups. Same shape as (C): wire == latched, healthy group.
    const MissedDescriptorSpec kDeferredClimb{
        Modulation::QPSK, CodeRate::R2_3, 8,
        Modulation::QPSK, CodeRate::R2_3, 8,
        protocol::kRungIdxQpskR34,             // a climb the sender is holding
        Modulation::QPSK, CodeRate::R3_4, 8,
        /*observed_declining_descriptor=*/false};
    std::fprintf(stderr, "== (D) climb command still deferred (must not arm) ==\n");
    const BurstResult d_fixed = runMissedDescriptorTrial(/*knob_on=*/true, kDeferredClimb);
    report("D knob=1 deferred climb", d_fixed);

    // (B2) SAME-RUNG Z RESTORATION. The live MPG@20 loss kept 8PSK R2/3 cw4
    // unchanged while the sender lifted Z27 -> Z81. A lost BURST_HEADER therefore
    // left the receiver at Z27. Mod/rate/cw equality is not a no-op when lifting Z
    // differs: the receiver would otherwise collect 4*648=2592 LLRs for a
    // 4*1944=7776-LLR member and mis-stride the whole group.
    const MissedDescriptorSpec kSameRungZ81{
        Modulation::QAM8, CodeRate::R2_3, 4,
        Modulation::QAM8, CodeRate::R2_3, 4,
        protocol::kRungIdxQam8R23,
        Modulation::QAM8, CodeRate::R2_3, 4,
        /*observed_declining_descriptor=*/false,
        /*wire_lifting_z=*/81,
        /*learned_lifting_z=*/81};
    std::fprintf(stderr, "== (B2) missed descriptor / same-rung Z81 restore ==\n");
    const BurstResult e_same_rung_z81 =
        runMissedDescriptorTrial(/*knob_on=*/false, kSameRungZ81);
    report("B2 knob=0 safe Z81 restore", e_same_rung_z81);

    // The transfer-scoped QPSK R3/4 long profile has the same authority contract:
    // its logical cw8/Z27 rung is announced and learned on wire as physical
    // cw3/Z81. Losing the next descriptor while mod/rate/cw remain physical cw3
    // must not make the receiver consume only 3*648=1944 of 3*1944=5832 LLRs.
    const MissedDescriptorSpec kSameRungQpskR34Z81{
        Modulation::QPSK, CodeRate::R3_4, 3,
        Modulation::QPSK, CodeRate::R3_4, 3,
        protocol::kRungIdxQpskR34,
        Modulation::QPSK, CodeRate::R3_4, 3,
        /*observed_declining_descriptor=*/false,
        /*wire_lifting_z=*/81,
        /*learned_lifting_z=*/81};
    std::fprintf(stderr,
                 "== (B3) missed descriptor / QPSK R3/4 cw3/Z81 restore ==\n");
    const BurstResult e_qpsk_r34_z81 =
        runMissedDescriptorTrial(/*knob_on=*/false, kSameRungQpskR34Z81);
    report("B3 knob=0 QPSK Z81 restore", e_qpsk_r34_z81);

    // (B4) BI1 + VARIABLE N. This is the exact live falsifier for the narrow
    // recovery path. The receiver previously learned the right cw4/Z81 profile and
    // is configured for the N8 maximum, but the sender has only N7 members left and
    // this group's descriptor is lost. With interleaving enabled, the receiver
    // cannot inspect PHYSICAL_BURST_END until after deinterleaving, while
    // deinterleaving itself requires N. It must therefore fail closed: restoring Z81
    // and inventing N8 publishes a premature 0/8 callback/ACK into live inbound air.
    const MissedDescriptorSpec kSameRungZ81Bi1VariableN{
        Modulation::QAM8, CodeRate::R2_3, 4,
        Modulation::QAM8, CodeRate::R2_3, 4,
        protocol::kRungIdxQam8R23,
        Modulation::QAM8, CodeRate::R2_3, 4,
        /*observed_declining_descriptor=*/false,
        /*wire_lifting_z=*/81,
        /*learned_lifting_z=*/81,
        /*wire_group_size=*/7,
        /*receiver_group_size=*/8,
        /*burst_interleave=*/true};
    std::fprintf(stderr,
                 "== (B4) BI1 missed descriptor / variable N must fail closed ==\n");
    const BurstResult e_bi1_variable_n =
        runMissedDescriptorTrial(/*knob_on=*/false, kSameRungZ81Bi1VariableN);
    report("B4 knob=0 BI1 N7/N8 fail closed", e_bi1_variable_n);

    // (B5) BI0 + VARIABLE N, inverse direction. A stale configured N7 must not
    // finalize while the sender's eighth member is still on air. BI0 can keep
    // collecting until member 8's CRC-protected tail proves exact N8.
    const MissedDescriptorSpec kSameRungZ81Bi0GrowN{
        Modulation::QAM8, CodeRate::R2_3, 4,
        Modulation::QAM8, CodeRate::R2_3, 4,
        protocol::kRungIdxQam8R23,
        Modulation::QAM8, CodeRate::R2_3, 4,
        /*observed_declining_descriptor=*/false,
        /*wire_lifting_z=*/81,
        /*learned_lifting_z=*/81,
        /*wire_group_size=*/8,
        /*receiver_group_size=*/7,
        /*burst_interleave=*/false,
        /*corrupt_wire_tail=*/false};
    std::fprintf(stderr,
                 "== (B5) BI0 missed descriptor / N7 must grow to proven N8 ==\n");
    const BurstResult e_bi0_grow_n =
        runMissedDescriptorTrial(/*knob_on=*/false, kSameRungZ81Bi0GrowN);
    report("B5 knob=0 BI0 N8/N7 tail", e_bi0_grow_n);

    auto kSameRungZ81Bi0BadTail = kSameRungZ81Bi0GrowN;
    kSameRungZ81Bi0BadTail.corrupt_wire_tail = true;
    kSameRungZ81Bi0BadTail.await_anchored_backstop = true;
    std::fprintf(stderr,
                 "== (B6) BI0 unknown N / corrupt tail must fail closed ==\n");
    const BurstResult e_bi0_bad_tail =
        runMissedDescriptorTrial(/*knob_on=*/false, kSameRungZ81Bi0BadTail);
    report("B6 knob=0 BI0 bad tail", e_bi0_bad_tail);

    std::fprintf(stderr, "== (E) sign-only marker provenance ==\n");
    const MarkerProvenanceResult e_orphan =
        runMarkerProvenanceTrial(/*include_second_member=*/false);
    const MarkerProvenanceResult e_confirmed =
        runMarkerProvenanceTrial(/*include_second_member=*/true);
    const MarkerProvenanceResult e_stale_fallback =
        runMarkerProvenanceTrial(/*include_second_member=*/true,
                                 /*corrupt_tail=*/true);
    const MarkerProvenanceResult e_wrong_z =
        runWrongLiftingUnprovenMarkerTrial();
    const StandaloneGateResult f_singleton = runStandaloneNonFinalGateTrial();
    const StandaloneGateResult f_later_light =
        runLaterLightFallbackGateTrial(/*physical_tail=*/false);
    const StandaloneGateResult f_later_tail =
        runLaterLightFallbackGateTrial(/*physical_tail=*/true);
    std::fprintf(stderr,
                 "== (F2) classic BI0 long-Z fallback after descriptor/head loss ==\n");
    const StandaloneGateResult f_classic_long =
        runClassicLongFallbackGateTrial();
    const bool f_classic_eligible = classicLongEligibilityTrial(
        /*burst_interleave=*/false, /*seed_geometry=*/true,
        /*learned_z=*/81, /*correlation=*/0.95f);
    const bool f_classic_bi1_rejected = !classicLongEligibilityTrial(
        /*burst_interleave=*/true, /*seed_geometry=*/true,
        /*learned_z=*/81, /*correlation=*/0.95f);
    const bool f_classic_unlearned_rejected = !classicLongEligibilityTrial(
        /*burst_interleave=*/false, /*seed_geometry=*/false,
        /*learned_z=*/81, /*correlation=*/0.95f);
    const bool f_classic_low_corr_rejected = !classicLongEligibilityTrial(
        /*burst_interleave=*/false, /*seed_geometry=*/true,
        /*learned_z=*/81,
        /*correlation=*/streaming_frame_policy::kMinSyncRecoveryCorrelation - 0.01f);
    const bool f_classic_z27_rejected = !classicLongEligibilityTrial(
        /*burst_interleave=*/false, /*seed_geometry=*/true,
        /*learned_z=*/27, /*correlation=*/0.95f);
    constexpr uint64_t kPriorGroupStart = 48000;
    constexpr uint64_t kSteadyAnchor =
        kPriorGroupStart + 10ull * 48000ull;
    constexpr uint64_t kLateMember =
        kPriorGroupStart + 16ull * 48000ull;
    const bool f_classic_late_member_cadence = classicLongEligibilityTrial(
        /*burst_interleave=*/false, /*seed_geometry=*/true,
        /*learned_z=*/81, /*correlation=*/0.95f,
        kPriorGroupStart, kSteadyAnchor, kLateMember);
    const bool f_classic_rto_anchor_rejected = !classicLongEligibilityTrial(
        /*burst_interleave=*/false, /*seed_geometry=*/true,
        /*learned_z=*/81, /*correlation=*/0.95f,
        kPriorGroupStart,
        kPriorGroupStart + 16ull * 48000ull,
        kPriorGroupStart + 17ull * 48000ull);
    const bool f_classic_session_state_cleared =
        classicLongSessionStateClearedTrial();
    const bool f_learned_geometry_synchronized =
        learnedRungGeometrySessionBoundarySynchronizationTrial();
    std::fprintf(stderr,
                 "  F2 callbacks=%d backstop=%d sync_accepts=%d corr=%.3f "
                 "physical_complete=%d air_rem=%llu classic_arms=%u z_after=%d "
                 "elig=%d bi1_reject=%d unlearned_reject=%d lowcorr_reject=%d "
                 "z27_reject=%d late_member=%d rto_anchor_reject=%d "
                 "session_clear=%d table_sync=%d\n",
                 f_classic_long.callbacks, f_classic_long.backstop_callbacks,
                 f_classic_long.data_sync_accepts,
                 f_classic_long.last_data_sync_correlation,
                 f_classic_long.physical_complete ? 1 : 0,
                 static_cast<unsigned long long>(
                     f_classic_long.air_remaining_at_callback),
                 f_classic_long.classic_long_arms,
                 f_classic_long.active_lifting_z_after,
                 f_classic_eligible ? 1 : 0,
                 f_classic_bi1_rejected ? 1 : 0,
                 f_classic_unlearned_rejected ? 1 : 0,
                 f_classic_low_corr_rejected ? 1 : 0,
                 f_classic_z27_rejected ? 1 : 0,
                 f_classic_late_member_cadence ? 1 : 0,
                 f_classic_rto_anchor_rejected ? 1 : 0,
                 f_classic_session_state_cleared ? 1 : 0,
                 f_learned_geometry_synchronized ? 1 : 0);
    const FailedAnchorGateResult f_failed_anchor = runFailedAnchorResetGateTrial();
    std::fprintf(stderr, "== (G) missed descriptor / CRC physical tail ==\n");
    const PhysicalTailGroupResult g_physical_tail =
        runMissedDescriptorPhysicalTailTrial();
    const PhysicalTailGroupResult g_stale_fallback =
        runMissedDescriptorPhysicalTailTrial(/*corrupt_tail=*/true);
    std::fprintf(stderr,
                 "  G callbacks=%d group=%u mask=0x%02X recovered=%d all_ok=%d "
                 "callback_sample=%zu physical_end=%zu fixed_decodes=%llu\n",
                 g_physical_tail.callbacks,
                 static_cast<unsigned>(g_physical_tail.group_size),
                 g_physical_tail.frame_mask, g_physical_tail.frames_recovered,
                 g_physical_tail.all_ok ? 1 : 0,
                 g_physical_tail.samples_at_callback,
                 g_physical_tail.physical_end_sample,
                 static_cast<unsigned long long>(g_physical_tail.fixed_decode_calls));
    std::fprintf(stderr,
                 "  G stale callbacks=%d group=%u mask=0x%02X recovered=%d all_ok=%d "
                 "geometry=%d callback_sample=%zu physical_end=%zu fixed_decodes=%llu\n",
                 g_stale_fallback.callbacks,
                 static_cast<unsigned>(g_stale_fallback.group_size),
                 g_stale_fallback.frame_mask, g_stale_fallback.frames_recovered,
                 g_stale_fallback.all_ok ? 1 : 0,
                 g_stale_fallback.geometry_proven ? 1 : 0,
                 g_stale_fallback.samples_at_callback,
                 g_stale_fallback.physical_end_sample,
                 static_cast<unsigned long long>(g_stale_fallback.fixed_decode_calls));
    checkProvisionalCeilingCoversWorstPhysicalVector();
    std::fprintf(stderr,
                 "  E orphan callbacks=%d unknown=%d mask=0x%02X recovered=%d; "
                 "confirmed callbacks=%d mask=0x%02X recovered=%d geometry=%d; "
                 "stale-tail callbacks=%d unknown=%d mask=0x%02X recovered=%d geometry=%d; "
                 "wrong-Z callbacks=%d unknown=%d\n",
                 e_orphan.callbacks, e_orphan.unknown_outcomes,
                 e_orphan.frame_mask, e_orphan.frames_recovered,
                 e_confirmed.callbacks, e_confirmed.frame_mask,
                 e_confirmed.frames_recovered, e_confirmed.geometry_proven ? 1 : 0,
                 e_stale_fallback.callbacks, e_stale_fallback.unknown_outcomes,
                 e_stale_fallback.frame_mask,
                 e_stale_fallback.frames_recovered,
                 e_stale_fallback.geometry_proven ? 1 : 0,
                 e_wrong_z.callbacks, e_wrong_z.unknown_outcomes);

    // ---- (A) ---------------------------------------------------------------
    // The alignment guard is correctness-critical and independent of the default-off
    // commanded-geometry experiment. Both arms must defer the short slice and recover
    // the same complete three-member group.
    EXPECT(a_knob_off.fired);
    EXPECT(a_knob_off.truncated_holds > 0);
    EXPECT(a_knob_off.frames_recovered == 3);
    EXPECT(a_knob_off.frame_mask == 0x07);
    EXPECT(a_knob_off.all_ok);
    EXPECT(a_knob_on.fired);
    EXPECT(a_knob_on.truncated_holds > 0);
    EXPECT(a_knob_on.frames_recovered == 3);
    EXPECT(a_knob_on.frame_mask == 0x07);
    EXPECT(a_knob_on.all_ok);

    // ---- (B) ---------------------------------------------------------------
    // Pre-fix: sliced with the previous rung -> group destroyed.
    EXPECT(b_broken.frames_recovered == 0);
    // Post-fix: frames 2..5 decode. Frame 1 is a DELIBERATE erasure — it was
    // demodulated with the old constellation before the deferred profile change
    // could land, so its soft bits are pushed as zero-LLR (the SACK bitmap asks for
    // it in the next round trip, instead of a full 17.5 s RTO for the whole group).
    EXPECT(b_fixed.fired);
    EXPECT(b_fixed.commanded_arms > 0);    // armed from the COMMANDED rung, not a fluke
    EXPECT(b_broken.commanded_arms == 0);
    EXPECT((b_fixed.frame_mask & 0x1E) == 0x1E);
    EXPECT(b_fixed.frames_recovered >= 4);
    // The mis-strided-finalize half: the stale (wider) geometry made the pre-fix arm
    // consume a different amount of air before publishing its ack than the true
    // group occupies. The commanded stride must change that accounting.
    if (b_broken.fired) {
        EXPECT(b_fixed.samples_at_callback != b_broken.samples_at_callback);
    }

    // ---- (C) and (D): the adoption gates -----------------------------------
    // BOTH must be COMPLETELY inert: the wire is the latched rung, so the group is
    // healthy and must decode 5/5 exactly as it does today. An arm here is the
    // regression that would make this feature a net loss on the air.
    EXPECT(c_fixed.commanded_arms == 0);
    EXPECT(c_fixed.fired);
    EXPECT(c_fixed.frames_recovered == 5);
    EXPECT(c_fixed.all_ok);
    EXPECT(d_fixed.commanded_arms == 0);
    EXPECT(d_fixed.fired);
    EXPECT(d_fixed.frames_recovered == 5);
    EXPECT(d_fixed.all_ok);

    // ---- (B2): narrow default-active same-rung long-Z restoration ----------
    // The broad commanded rung-switch experiment remains disabled in this arm.
    // Prior wire-learned Z81 geometry must nevertheless restore the exact long
    // member stride after a missed descriptor, erasing only frame 1 as designed.
    EXPECT(e_same_rung_z81.fired);
    EXPECT(e_same_rung_z81.commanded_arms > 0);
    EXPECT(e_same_rung_z81.active_lifting_z == 81);
    EXPECT(e_same_rung_z81.active_erasure_soft_bits == 7776);
    EXPECT(e_same_rung_z81.active_erasure_soft_bits != 2592);
    EXPECT(e_same_rung_z81.burst_min_block_samples ==
           e_same_rung_z81.wire_frame_samples);
    EXPECT((e_same_rung_z81.frame_mask & 0x1E) == 0x1E);
    EXPECT(e_same_rung_z81.frames_recovered >= 4);
    EXPECT(e_same_rung_z81.air_remaining_at_callback == 0);

    EXPECT(e_qpsk_r34_z81.fired);
    EXPECT(e_qpsk_r34_z81.commanded_arms > 0);
    EXPECT(e_qpsk_r34_z81.active_lifting_z == 81);
    EXPECT(e_qpsk_r34_z81.active_erasure_soft_bits == 5832);
    EXPECT(e_qpsk_r34_z81.active_erasure_soft_bits != 1944);
    EXPECT(e_qpsk_r34_z81.burst_min_block_samples ==
           e_qpsk_r34_z81.wire_frame_samples);
    EXPECT((e_qpsk_r34_z81.frame_mask & 0x1E) == 0x1E);
    EXPECT(e_qpsk_r34_z81.frames_recovered >= 4);
    EXPECT(e_qpsk_r34_z81.air_remaining_at_callback == 0);

    // Default-active same-rung Z81 recovery is deliberately BI0-only. With BI1 and
    // unknown physical N, no guessed N8 callback may drive an early ACK.
    EXPECT(e_bi1_variable_n.commanded_arms == 0);
    EXPECT(!e_bi1_variable_n.fired);

    // The inverse stale-N direction must cross configured N7 without publishing,
    // then finalize exactly once at the wire-proven N8 tail.
    EXPECT(e_bi0_grow_n.commanded_arms > 0);
    EXPECT(e_bi0_grow_n.air_gate_span_at_arm >=
           e_bi0_grow_n.wire_frame_samples * 8);
    EXPECT(e_bi0_grow_n.fired);
    EXPECT(e_bi0_grow_n.group_size == 8);
    EXPECT((e_bi0_grow_n.frame_mask & 0x00FE) == 0x00FE);
    EXPECT(e_bi0_grow_n.frames_recovered >= 7);
    EXPECT(e_bi0_grow_n.air_remaining_at_callback == 0);

    // If exact N cannot be CRC-proven, even a complete set of plausible positions
    // authorizes neither a guessed callback nor a reverse ACK.
    EXPECT(e_bi0_bad_tail.commanded_arms > 0);
    EXPECT(!e_bi0_bad_tail.fired);
    EXPECT(e_bi0_bad_tail.unknown_outcomes >= 1);
    EXPECT(e_bi0_bad_tail.backstop_callbacks == 1);
    EXPECT(e_bi0_bad_tail.backstop_payload_seen);
    EXPECT(!e_bi0_bad_tail.group_before_backstop);
    EXPECT(e_bi0_bad_tail.backstop_arm_abs > 0);
    EXPECT(e_bi0_bad_tail.backstop_fed >
           e_bi0_bad_tail.backstop_arm_abs +
               kAnchoredBackstopWindowSamples);

    // ---- (E): marker provenance --------------------------------------------
    // One sign decision plus trailing silence is not a group and must not emit a
    // synthetic 0/N callback/ACK. Neither are several process-valid slices: only
    // decoded descriptor provenance or a CRC-valid physical tail proves geometry.
    EXPECT(e_orphan.callbacks == 0);
    EXPECT(e_orphan.unknown_outcomes == 1);
    EXPECT(e_confirmed.unknown_outcomes == 0);
    EXPECT(e_stale_fallback.unknown_outcomes >= 1);
    EXPECT(e_confirmed.callbacks == 1);
    EXPECT(e_confirmed.group_size == 2);
    EXPECT(e_confirmed.frame_mask == 0x03);
    EXPECT(e_confirmed.frames_recovered == 2);
    EXPECT(e_confirmed.all_ok);
    EXPECT(e_confirmed.geometry_proven);
    EXPECT(e_stale_fallback.callbacks == 0);
    EXPECT(e_wrong_z.callbacks == 0);
    EXPECT(e_wrong_z.unknown_outcomes >= 1);

    // ---- (F): provisional full-anchor ACK gate -----------------------------
    EXPECT(f_singleton.callbacks == 1);
    EXPECT(f_singleton.physical_complete);
    EXPECT(f_singleton.air_remaining_at_callback == 0);
    EXPECT(f_singleton.backstop_callbacks == 0);
    EXPECT(f_later_light.callbacks == 1);
    EXPECT(!f_later_light.physical_complete);
    EXPECT(f_later_light.air_remaining_at_callback > 0);
    EXPECT(f_later_light.one_sample_before_deadline == 1);
    EXPECT(f_later_light.at_deadline == 0);
    EXPECT(f_later_light.backstop_callbacks == 1);
    EXPECT(f_later_light.backstop_payload_seen);
    EXPECT(f_later_tail.callbacks == 1);
    EXPECT(f_later_tail.physical_complete);
    EXPECT(f_later_tail.air_remaining_at_callback == 0);
    EXPECT(f_later_tail.backstop_callbacks == 0);
    // Ordinary short-Z fallback stays byte-for-byte on the legacy path.
    EXPECT(f_later_light.classic_long_arms == 0);
    EXPECT(f_later_tail.classic_long_arms == 0);
    EXPECT(f_later_tail.active_lifting_z_after == 27);

    // The exact live long-Z candidate takes the new path once, decodes the
    // independently protected tail, clears the physical gate/backstop, and
    // restores the short control/default geometry before the next search.
    EXPECT(f_classic_eligible);
    EXPECT(f_classic_long.data_sync_accepts >= 2);
    EXPECT(f_classic_long.last_data_sync_correlation >=
           streaming_frame_policy::kMinSyncRecoveryCorrelation);
    EXPECT(f_classic_long.classic_long_arms == 1);
    EXPECT(f_classic_long.callbacks == 1);
    EXPECT(f_classic_long.physical_complete);
    EXPECT(f_classic_long.air_remaining_at_callback == 0);
    EXPECT(f_classic_long.backstop_callbacks == 0);
    EXPECT(f_classic_long.active_lifting_z_after == 27);

    // Fail closed whenever any safety/provenance leg is absent. In particular,
    // BI1 cannot recover without independently decodable N, a learned Z27 tuple
    // is not long geometry, and a low-confidence LTS is not enough evidence to
    // reinterpret a three-times-longer physical member.
    EXPECT(f_classic_bi1_rejected);
    EXPECT(f_classic_unlearned_rejected);
    EXPECT(f_classic_low_corr_rejected);
    EXPECT(f_classic_z27_rejected);
    // Live timing regression: the later member itself is beyond the 15 s
    // cadence discriminator, but the accepted anchor that defines this receive
    // epoch is not. Gate on the anchor; still reject a genuinely post-RTO anchor.
    EXPECT(kLateMember - kPriorGroupStart >
           streaming_decode_policy::kBurstCadenceRtoGapSamples);
    EXPECT(kSteadyAnchor - kPriorGroupStart <
           streaming_decode_policy::kBurstCadenceRtoGapSamples);
    EXPECT(f_classic_late_member_cadence);
    EXPECT(f_classic_rto_anchor_rejected);
    EXPECT(f_classic_session_state_cleared);
    EXPECT(f_learned_geometry_synchronized);
    EXPECT(f_failed_anchor.callbacks == 0);
    EXPECT(f_failed_anchor.before_reset > 0);
    EXPECT(f_failed_anchor.backstop_callbacks == 1);
    EXPECT(f_failed_anchor.after_reset == 0);
    EXPECT(!a_knob_off.truncation_hold_active);
    EXPECT(!a_knob_on.truncation_hold_active);
    EXPECT(a_knob_off.air_remaining_at_callback == 0);
    EXPECT(a_knob_on.air_remaining_at_callback == 0);
    EXPECT(b_fixed.air_remaining_at_callback == 0);
    EXPECT(c_fixed.air_remaining_at_callback == 0);
    EXPECT(d_fixed.air_remaining_at_callback == 0);

    // ---- (G): exact physical-tail finalization -----------------------------
    EXPECT(g_physical_tail.callbacks == 1);
    EXPECT(g_physical_tail.group_size == 4);
    EXPECT(g_physical_tail.frame_mask == 0x0F);
    EXPECT(g_physical_tail.frames_recovered == 4);
    EXPECT(g_physical_tail.all_ok);
    EXPECT(g_physical_tail.geometry_proven);
    // Callback may trail the exact sample boundary by at most one feed chunk. It
    // must not consume even one guessed fallback frame of trailing silence.
    EXPECT(g_physical_tail.samples_at_callback >= g_physical_tail.physical_end_sample);
    EXPECT(g_physical_tail.samples_at_callback <=
           g_physical_tail.physical_end_sample + 4800);
    EXPECT(g_physical_tail.fixed_decode_calls == 4);
    // Same physical N=4 and stale configured N=6, but destroy the CRC tail while
    // retaining a substantive final OFDM member. With neither a decoded descriptor
    // nor a CRC-valid tail, guessed N=6 must not publish an ACK-driving callback.
    EXPECT(g_stale_fallback.callbacks == 0);
    EXPECT(g_stale_fallback.unknown_outcomes >= 1);

    if (g_failures == 0) {
        std::fprintf(stderr, "test_burst_stale_geometry: PASS\n");
        return 0;
    }
    std::fprintf(stderr, "test_burst_stale_geometry: %d FAILURE(S)\n", g_failures);
    return 1;
}
