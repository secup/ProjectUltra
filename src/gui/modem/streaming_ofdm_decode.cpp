// StreamingDecoder module

#include "ofdm/genie_tx_capture.hpp"
#include "streaming_decoder.hpp"
#include "streaming_buffer_policy.hpp"
#include "streaming_control_profile.hpp"
#include "streaming_decode_policy.hpp"
#include "streaming_frame_policy.hpp"
#include "sync/signal_policy.hpp"
#include "waveform/mc_dpsk_waveform.hpp"
#include "waveform/ofdm_chirp_waveform.hpp"
#include "fec/frame_interleaver.hpp"
#include "fec/burst_interleaver.hpp"
#include "ultra/fec.hpp"
#include "fec/ldpc_codec.hpp"
#include "ultra/logging.hpp"
#include "ultra/ofdm_link_adaptation.hpp"
#include "ultra/timing_profiler.hpp"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <exception>
#include <fstream>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include "ultra/phy_diagnostics.hpp"

namespace ultra {
namespace gui {

namespace v2 = protocol::v2;
namespace decode_policy = streaming_decode_policy;
namespace frame_policy = streaming_frame_policy;
namespace signal_policy = ::ultra::sync::signal_policy;
namespace arrival_policy = ::ultra::sync::frame_arrival_policy;

namespace {

bool isFixedFrameCwCount(int cw_count) {
    return cw_count >= v2::kMinFixedFrameCodewords &&
           cw_count <= v2::kMaxFixedFrameCodewords;
}

float sampleRMS(const std::vector<float>& samples) {
    if (samples.empty()) {
        return 0.0f;
    }
    double energy = 0.0;
    for (float sample : samples) {
        energy += static_cast<double>(sample) * sample;
    }
    return static_cast<float>(std::sqrt(energy / static_cast<double>(samples.size())));
}

bool decodedFrameHasMoreFrag(const DecodeResult& result) {
    if (result.frame_data.size() >= 4) {
        return (result.frame_data[3] & v2::Flags::MORE_FRAG) != 0;
    }
    if (result.has_partial_codewords) {
        return (result.partial_codewords.flags & v2::Flags::MORE_FRAG) != 0;
    }
    return false;
}

bool hasCleanFadingMeasurementTiming(const DecodeResult& result) {
    // The public coherent fading meter is an LTS magnitude-CV measurement. If
    // the LTS phase slope says the FFT window is tens of samples off the frame
    // boundary, the magnitude ripple is a timing/windowing artifact, not RF
    // selectivity. Decoding can still succeed inside the cyclic prefix, but the
    // channel-quality sample must be held unless data pilots independently show
    // static frequency selectivity. A real notch is frequency-CV dominated with
    // little common symbol-mean motion; the AWGN artifact is not.
    constexpr float kMaxLtsTimingOffsetForFadingMeterSamples = 24.0f;
    if (std::isfinite(result.lts_timing_offset_samples) &&
        std::abs(result.lts_timing_offset_samples) <=
            kMaxLtsTimingOffsetForFadingMeterSamples) {
        return true;
    }

    constexpr float kStaticSelectivityMinFrequencyCV = 0.12f;
    constexpr float kStaticSelectivityDominance = 1.5f;
    constexpr float kStaticSelectivityMaxSymbolMeanCV = 0.08f;
    return result.pilot_frequency_cv >= kStaticSelectivityMinFrequencyCV &&
           result.pilot_frequency_cv >=
               kStaticSelectivityDominance * result.pilot_temporal_cv &&
           result.pilot_symbol_mean_cv <= kStaticSelectivityMaxSymbolMeanCV;
}

bool failureAttributionEnabled() {
    static const bool enabled = [] {
        const char* value = std::getenv("ULTRA_FAILURE_ATTRIBUTION");
        return value && value[0] != '\0' && !(value[0] == '0' && value[1] == '\0');
    }();
    return enabled;
}

bool qam16GenieTimingCfoEnabled() {
    static const bool enabled = [] {
        const char* value = std::getenv("ULTRA_QAM16_GENIE_TIMING_CFO");
        return value && value[0] != '\0' && !(value[0] == '0' && value[1] == '\0');
    }();
    return enabled;
}

std::string formatU8Vector(const std::vector<uint8_t>& values) {
    std::ostringstream oss;
    oss << "[";
    for (size_t i = 0; i < values.size(); ++i) {
        if (i) oss << ",";
        oss << static_cast<int>(values[i]);
    }
    oss << "]";
    return oss.str();
}

std::string formatIntVector(const std::vector<int>& values) {
    std::ostringstream oss;
    oss << "[";
    for (size_t i = 0; i < values.size(); ++i) {
        if (i) oss << ",";
        oss << values[i];
    }
    oss << "]";
    return oss.str();
}

std::string formatFloatVector(const std::vector<float>& values) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2) << "[";
    for (size_t i = 0; i < values.size(); ++i) {
        if (i) oss << ",";
        oss << values[i];
    }
    oss << "]";
    return oss.str();
}

void copyCodewordDiagnostics(DecodeResult& result, const v2::CodewordStatus& status) {
    result.cw_decoded.clear();
    result.cw_decoded.reserve(status.decoded.size());
    for (bool decoded : status.decoded) {
        result.cw_decoded.push_back(decoded ? 1 : 0);
    }
    result.cw_iterations = status.iterations;
    result.cw_unsatisfied_checks = status.unsatisfied_checks;
    result.cw_llr_abs_mean = status.llr_abs_mean;
    result.cw_llr_abs_min = status.llr_abs_min;
    result.cw_llr_abs_p10 = status.llr_abs_p10;
    result.cw_llr_abs_p50 = status.llr_abs_p50;
    result.cw_llr_abs_p90 = status.llr_abs_p90;
    result.cw_used_perturbation = status.used_perturbation;
    result.cw_harq_attempts = status.harq_attempts;
}

void logFailureAttributionFrame(const char* prefix,
                                const DecodeResult& result,
                                IWaveform* waveform,
                                bool is_ofdm,
                                bool connected,
                                size_t frame_sync_abs,
                                size_t frame_len,
                                Modulation modulation,
                                CodeRate rate,
                                float sync_correlation) {
    if (!failureAttributionEnabled() || !is_ofdm || !connected) {
        return;
    }
    const int attempted_codewords = result.codewords_ok + result.codewords_failed;
    if (attempted_codewords < v2::kMinFixedFrameCodewords || result.cw_decoded.empty()) {
        return;
    }

    const char* outcome = result.success ? "PASS" : "FAIL";
    const auto cw_decoded = formatU8Vector(result.cw_decoded);
    const auto cw_iters = formatIntVector(result.cw_iterations);
    const auto cw_unsat = formatIntVector(result.cw_unsatisfied_checks);
    const auto cw_llr_mean = formatFloatVector(result.cw_llr_abs_mean);
    const auto cw_llr_min = formatFloatVector(result.cw_llr_abs_min);
    const auto cw_llr_p10 = formatFloatVector(result.cw_llr_abs_p10);
    const auto cw_llr_p50 = formatFloatVector(result.cw_llr_abs_p50);
    const auto cw_llr_p90 = formatFloatVector(result.cw_llr_abs_p90);
    const auto perturb = formatU8Vector(result.cw_used_perturbation);
    const auto harq_attempts = formatIntVector(result.cw_harq_attempts);

    if (result.success) {
        LOG_MODEM(WARN,
                  "[%s] FAIL_ATTR frame outcome=%s mod=%s rate=%s sync_abs=%zu frame_len=%zu cw_ok=%d cw_fail=%d snr=%.2f snr_src=%s cfo=%.2f corr=%.3f "
                  "cw_decoded=%s cw_iters=%s cw_unsat=%s cw_llr_mean=%s cw_llr_min=%s cw_llr_p10=%s cw_llr_p50=%s cw_llr_p90=%s perturb=%s harq_attempts=%s",
                  prefix, outcome, modulationToString(modulation), codeRateToString(rate),
                  frame_sync_abs, frame_len, result.codewords_ok, result.codewords_failed,
                  result.snr_db, snrSourceToString(result.snr_source), result.cfo_hz,
                  sync_correlation, cw_decoded.c_str(), cw_iters.c_str(),
                  cw_unsat.c_str(), cw_llr_mean.c_str(), cw_llr_min.c_str(),
                  cw_llr_p10.c_str(), cw_llr_p50.c_str(), cw_llr_p90.c_str(),
                  perturb.c_str(), harq_attempts.c_str());
    } else {
        LOG_MODEM(WARN,
                  "[%s] FAIL_ATTR frame outcome=%s mod=%s rate=%s sync_abs=%zu frame_len=%zu cw_ok=%d cw_fail=%d snr=%.2f snr_src=%s cfo=%.2f corr=%.3f "
                  "cw_decoded=%s cw_iters=%s cw_unsat=%s cw_llr_mean=%s cw_llr_min=%s cw_llr_p10=%s cw_llr_p50=%s cw_llr_p90=%s perturb=%s harq_attempts=%s",
                  prefix, outcome, modulationToString(modulation), codeRateToString(rate),
                  frame_sync_abs, frame_len, result.codewords_ok, result.codewords_failed,
                  result.snr_db, snrSourceToString(result.snr_source), result.cfo_hz,
                  sync_correlation, cw_decoded.c_str(), cw_iters.c_str(),
                  cw_unsat.c_str(), cw_llr_mean.c_str(), cw_llr_min.c_str(),
                  cw_llr_p10.c_str(), cw_llr_p50.c_str(), cw_llr_p90.c_str(),
                  perturb.c_str(), harq_attempts.c_str());
    }

    if (!result.success && waveform) {
        const auto eq_diag = waveform->getFailureAttributionDiagnosticsText();
        LOG_MODEM(WARN, "[%s] FAIL_ATTR %s", prefix, eq_diag.c_str());
    }
}

}  // namespace

static std::atomic<int> g_robust_retry_hits{0};   // CW0 peek: retry succeeded after initial fail
static std::atomic<int> g_salvage_hits{0};         // 1-CW control salvaged from fixed-frame path

static bool hasInvalidOFDMTraining(IWaveform* waveform, bool is_ofdm, bool connected,
                                   float& lts_signal_power, float& lts_channel_mag) {
    lts_signal_power = 1.0f;
    lts_channel_mag = 1.0f;
    if (!waveform || !is_ofdm || !connected) {
        return false;
    }

    lts_signal_power = waveform->getLastLTSSignalPower();
    lts_channel_mag = waveform->getLastLTSChannelMagnitude();
    return signal_policy::invalidOFDMLTSTraining(
        is_ofdm, connected, lts_signal_power, lts_channel_mag);
}

// Robust single-CW LDPC decode with Phase 0 decoder diversity (up to 4 retries)
// Uses standalone LDPCDecoder for setMinSumFactor (not available via ICodec interface)
// Pattern matches decodeFixedFrame() Phase 0 (frame_v2.cpp:1378-1395)
//
// max_retries: caps the retry budget (0..4). Default 4 = full diversity sweep.
//   ControlFirst (ACK decode) calls this with max_retries=2 — ACK loss is
//   recoverable via ARQ, but ACK decode CPU isn't. See profiling plan v5.
//
// call_site: routes per-call-site retry-attempt histogram. See timing_profiler.hpp.
static std::pair<bool, Bytes> robustDecodeSingleCW(
    const float* cw_data, size_t cw_size, CodeRate rate, const char* log_prefix = nullptr,
    ultra::timing::SingleCWCallSite call_site = ultra::timing::SingleCWCallSite::Default,
    int max_retries = 4, int lifting_z = 27)
{
    ultra::timing::ScopedTimer _profile_(
        ultra::timing::globalDecoderProfile().single_cw_decode_total);

    // Reuse one LDPCDecoder instance per code rate per thread. Constructing
    // an LDPCDecoder calls buildMatrix() which expands the IEEE 802.11n
    // parity-check matrix — that's expensive (~50ms+ on Pi 5). For ACK-heavy
    // workloads (~1000 ACK decodes per 50 KB transfer at hardware speeds)
    // the construction cost dominates the actual decode work.
    //
    // thread_local is safe here because the decode thread is the only caller
    // of this function. The cache is keyed by rate — if a future caller
    // ever uses a different rate from the same thread, we re-construct.
    struct CachedDecoder {
        std::unique_ptr<LDPCDecoder> decoder;
        CodeRate rate = static_cast<CodeRate>(-1);
        int lifting_z = 0;
    };
    static thread_local CachedDecoder cache;
    lifting_z = (lifting_z == 81) ? 81 : 27;
    if (!cache.decoder || cache.rate != rate || cache.lifting_z != lifting_z) {
        cache.decoder = std::make_unique<LDPCDecoder>(rate, lifting_z);
        cache.rate = rate;
        cache.lifting_z = lifting_z;
    }
    LDPCDecoder& decoder = *cache.decoder;
    decoder.setMaxIterations(fec::LDPCCodec::getRecommendedIterations(rate));
    decoder.setMinSumFactor(0.9375f);

    auto decoded = decoder.decodeSoft(std::span<const float>(cw_data, cw_size));
    bool ok = decoder.lastDecodeSuccess();

    int attempts_used = 0;  // 0 = first try; 1..max_retries = retry idx; -1 = exhausted

    if (!ok) {
        static constexpr float factors[] = {0.875f, 0.75f, 0.625f, 0.5f};
        if (max_retries < 0) max_retries = 0;
        if (max_retries > 4) max_retries = 4;
        for (int retry = 0; retry < max_retries && !ok; retry++) {
            decoder.setMinSumFactor(factors[retry]);
            decoded = decoder.decodeSoft(std::span<const float>(cw_data, cw_size));
            ok = decoder.lastDecodeSuccess();
            if (ok) {
                attempts_used = retry + 1;
                g_robust_retry_hits.fetch_add(1, std::memory_order_relaxed);
                if (log_prefix) {
                    LOG_MODEM(INFO, "[%s] Robust CW0: RETRY OK (factor=%.3f, iters=%d, total_hits=%d)",
                              log_prefix, factors[retry], decoder.lastIterations(),
                              g_robust_retry_hits.load(std::memory_order_relaxed));
                }
            }
        }
        if (!ok) attempts_used = -1;  // exhausted budget
    }

    // Bump per-call-site retry histogram
    {
        auto& dp = ultra::timing::globalDecoderProfile();
        ultra::timing::SingleCWHistogram* hist = nullptr;
        switch (call_site) {
            case ultra::timing::SingleCWCallSite::ControlFirst:
                hist = &dp.robust_cw_control_first; break;
            case ultra::timing::SingleCWCallSite::Cw0Peek:
                hist = &dp.robust_cw_cw0_peek; break;
            case ultra::timing::SingleCWCallSite::Default:
            default:
                hist = &dp.robust_cw_default; break;
        }
        hist->record(attempts_used);
    }

    Bytes data;
    if (ok) data.assign(decoded.begin(), decoded.end());
    return {ok, data};
}

int StreamingDecoder::expectedOFDMCodewordsForSamples(size_t sample_count) const {
    if (!waveform_ || !connected_ || !protocol::isOFDMMode(mode_)) {
        return 1;
    }

    if (pending_total_cw_ > 0) {
        return pending_total_cw_;
    }

    const int max_cw = v2::sanitizeFixedFrameCodewords(fixed_frame_codewords_);
    for (int cw = max_cw; cw >= 2; --cw) {
        if (sample_count >= static_cast<size_t>(waveform_->getMinSamplesForCWCount(cw))) {
            return cw;
        }
    }

    return 1;
}

bool StreamingDecoder::processWaveformForCodewords(SampleSpan samples,
                                                   int expected_codewords) {
    if (!waveform_) {
        return false;
    }

    const bool allow_carrier_ldpc =
        use_carrier_ldpc_interleaver_ && connected_ && protocol::isOFDMMode(mode_);
    const bool allow_rx_erasure =
        connected_ &&
        protocol::isOFDMMode(mode_) &&
        expected_codewords >= 2 &&
        expected_codewords <= v2::kMaxFixedFrameCodewords;
    waveform_->setCarrierLdpcInterleaverEnabled(allow_carrier_ldpc);
    waveform_->setRXCarrierErasureEnabled(allow_rx_erasure);
    return waveform_->process(samples);
}

// ============================================================================
// BUG-BURST-STALE-GEOMETRY (2026-07-28) — commanded-geometry resolver
// ============================================================================

uint64_t StreamingDecoder::absoluteForSyncPosition() {
    std::lock_guard<std::mutex> lock(sync_controller_.ring_.buffer_mutex_);
    return static_cast<uint64_t>(
        sync_controller_.ring_.ringPosToAbsoluteLocked(sync_position_));
}

void StreamingDecoder::seedRungGeometryForTesting(Modulation mod, CodeRate rate,
                                                 int cw, int lifting_z) {
    learnRungGeometry(mod, rate, cw, lifting_z);
}

std::array<int, 2> StreamingDecoder::learnedRungGeometryForTesting(
        Modulation mod, CodeRate rate) const {
    const uint8_t idx = protocol::coherentRungIndexFor(mod, rate);
    const LearnedRungGeometry learned = learnedRungGeometrySnapshot(idx);
    return {static_cast<int>(learned.cw_per_frame),
            static_cast<int>(learned.lifting_z)};
}

uint64_t StreamingDecoder::learnedRungGeometrySessionEpochForTesting() const {
    return learnedRungGeometrySessionEpochSnapshot();
}

bool StreamingDecoder::seedRungGeometryForSessionForTesting(
        Modulation mod, CodeRate rate, int cw, int lifting_z,
        uint64_t session_epoch) {
    return learnRungGeometry(mod, rate, cw, lifting_z, session_epoch);
}

void StreamingDecoder::setLDPCLiftingZForTesting(int lifting_z) {
    std::lock_guard<std::mutex> lock(sync_controller_.ring_.buffer_mutex_);
    testing_lifting_z_override_ = (lifting_z == 81) ? 81 : 27;

    // The waveform owns the stream processor's "one complete LDPC block" gate
    // and its frame-airtime geometry. Merely making decodeFixedFrame() Z-aware
    // would still return after 648 LLRs and size the receive window too short.
    if (waveform_) {
        waveform_->setActiveLDPCLiftingZ(
            static_cast<uint8_t>(testing_lifting_z_override_));
    }

    // Keep the legacy per-CW helper consistent as well. The fixed-frame path
    // builds its own Z-aware interleaver, but a test hook should not leave a
    // contradictory block size behind for diagnostic/probe paths.
    const int carriers = protocol::isOFDMMode(mode_)
        ? ofdm_data_carriers_
        : mc_dpsk_carriers_;
    const size_t bps = static_cast<size_t>(carriers) *
                       getBitsPerSymbol(current_modulation_);
    const size_t codeword_bits =
        testing_lifting_z_override_ == 81 ? size_t{1944}
                                          : v2::LDPC_CODEWORD_BITS;
    frame_decoder_.interleaver_ =
        std::make_unique<ChannelInterleaver>(bps, codeword_bits);
}

bool StreamingDecoder::learnRungGeometry(
        Modulation mod, CodeRate rate, int cw, int lifting_z,
        std::optional<uint64_t> expected_session_epoch) {
    const uint8_t idx = protocol::coherentRungIndexFor(mod, rate);
    if (idx == protocol::kRungIdxNone || idx >= protocol::kRungIdxCount) return false;
    if (cw < 1) return false;
    const LearnedRungGeometry learned{
        static_cast<uint8_t>(v2::sanitizeFixedFrameCodewords(cw)),
        static_cast<uint8_t>((lifting_z == 81) ? 81 : 27)};
    std::lock_guard<std::mutex> lock(learned_rung_geometry_mutex_);
    if (expected_session_epoch.has_value() &&
        *expected_session_epoch != learned_rung_geometry_session_epoch_) {
        return false;
    }
    learned_rung_geometry_[idx] = learned;
    return true;
}

StreamingDecoder::LearnedRungGeometry
StreamingDecoder::learnedRungGeometrySnapshot(uint8_t idx) const {
    if (idx == protocol::kRungIdxNone || idx >= protocol::kRungIdxCount) {
        return LearnedRungGeometry{};
    }
    std::lock_guard<std::mutex> lock(learned_rung_geometry_mutex_);
    return learned_rung_geometry_[idx];
}

uint64_t StreamingDecoder::learnedRungGeometrySessionEpochSnapshot() const {
    std::lock_guard<std::mutex> lock(learned_rung_geometry_mutex_);
    return learned_rung_geometry_session_epoch_;
}

void StreamingDecoder::clearLearnedRungGeometryAtSessionBoundary() {
    std::lock_guard<std::mutex> lock(learned_rung_geometry_mutex_);
    ++learned_rung_geometry_session_epoch_;
    learned_rung_geometry_.fill(LearnedRungGeometry{});
}

// ADOPTION EVIDENCE (2026-07-28). A decoded descriptor is the sender telling us, on
// the wire, which rung it is ACTUALLY transmitting. Compare it against our standing
// command: equal ⇒ adopted; different ⇒ the sender has NOT taken the command (it is
// deferring an UP to a clean boundary, has a legacy exchange in flight, snapped the
// index to its own enabled ladder, or has rate adaptation pinned). Without this the
// resolver treats "we asked" as "it happened" and destroys groups that decode fine
// today — the single most damaging thing this feature could do.
void StreamingDecoder::noteDescriptorRungObserved(Modulation mod, CodeRate rate) {
    const uint8_t cmd = commanded_rung_idx_.load(std::memory_order_relaxed);
    if (cmd == protocol::kRungIdxNone) return;
    const uint8_t seen = protocol::coherentRungIndexFor(mod, rate);
    commanded_rung_declined_.store(seen != cmd, std::memory_order_relaxed);
}

// The group's BURST_HEADER descriptor was NOT decoded (missed full dual-chirp
// anchor -> connected DATA-sync fallback). Today the decoder arms accumulation
// from the LATCHED fixed_frame_codewords_/mod/rate, which after a rung switch is
// the PREVIOUS rung: wrong constellation on every frame (0/N CWs) AND a wrong
// stride, so the group's air-end is short and the half-duplex ack interlock opens
// INTO the sender's own key-down.
//
// The receiver is the rate AUTHORITY: it commanded the sender's next rung in its
// own last tone-burst ack, so it already knows the geometry. But "we commanded X"
// is NOT "the sender is at X" — a wrong arm destroys a group that decodes fine
// today (the bug report's own xfer_3 burst#26 is exactly that case). THREE
// independent conditions must hold, each derived from the sender's actual code
// rather than tuned:
//   1. DID IT HEAR US?  The command rides in the tone-burst ack; a lost ack costs
//      the sender its full ARQ RTO, so a resend cannot arrive inside the
//      steady-state cadence  ->  the arrival-cadence guard.
//   2. WOULD IT OBEY?    Connection::maybeObeyAuthorityCommand DEFERS a faster
//      (UP) command while its TX window drains and re-checks on every re-carried
//      ACK copy, but obeys a DEMOTE immediately ("a failing window never drains").
//      So only a demote is promptly adopted  ->  the demote-only gate.
//   3. HAS IT OBEYED?    Every decoded descriptor announces the rung actually being
//      transmitted; one that disagrees with our standing command is direct wire
//      evidence of non-adoption  ->  the declined gate (noteDescriptorRungObserved).
StreamingDecoder::CommandedGeometry
StreamingDecoder::resolveCommandedGeometry(bool is_ofdm, bool burst_latched) {
    CommandedGeometry cg;
    const bool broad_commanded_geometry =
        decode_policy::commandedGeometryEnabled();
    if (!waveform_ || !is_ofdm || !connected_ || !burst_latched) return cg;
    if (mode_ != protocol::WaveformMode::OFDM_CHIRP) return cg;  // wideband scope
    if (!(use_burst_interleave_ || burst_transport_rx_)) return cg;  // burst regime
    // A descriptor for THIS group WAS consumed -> nothing to reconstruct.
    if (pending_total_cw_ > 0) return cg;
    if (sync_controller_.have_burst_descriptor_) return cg;
    // Without RX rate authority the ack's five bits are a quantized quality +
    // RELATIVE demote, not an absolute rung index — reconstructing one is garbage.
    if (!protocol::rxRateAuthorityEnabled()) return cg;

    const uint8_t idx = commanded_rung_idx_.load(std::memory_order_relaxed);
    if (idx == protocol::kRungIdxNone || idx >= protocol::kRungIdxCount) return cg;
    // GATE 3 — the sender has told us on the wire that it is NOT on this rung.
    if (commanded_rung_declined_.load(std::memory_order_relaxed)) return cg;
    const LearnedRungGeometry learned = learnedRungGeometrySnapshot(idx);
    if (learned.cw_per_frame < 1) return cg;  // table miss -> today's behaviour

    const auto pick = protocol::coherentRungFromIndex(idx);
    const int active_lifting_z = activeBurstLiftingZ();
    // LIVE Z81 RECOVERY (2026-08-04): keep the broad commanded-rung experiment
    // default-OFF, but do not let that knob disable this narrowly proven BI0
    // geometry correction. The receiver has already decoded a descriptor for one
    // of the two exact transfer-scoped long profiles and therefore learned Z81 from
    // the wire; its standing authority command names that same rung, and only the
    // current descriptor is missing. In that state Z27 is not an alternate policy
    // choice — it is stale geometry that collects cw*648 LLRs from a cw*1944-LLR
    // member and mis-strides the group.
    //
    // This path MUST fail closed for BI1. A missed descriptor also loses physical N,
    // and N is not rung-derivable. BI0 can prove the exact tail by decoding each
    // member's CRC-protected PHYSICAL_BURST_END flag before finalization. BI1 cannot
    // decode that flag until after group deinterleaving, which itself requires N.
    // Reusing burst_group_size_ there fabricated stale N=8 for a live N=7 burst,
    // declared air-end 2.38 s early, and caused the ACK to be dropped into inbound
    // audio. BI1 recovery therefore needs an outer, independently decodable N/tail
    // signal before this narrow default-active path can safely include it.
    // All resolver gates above (connected wide burst, descriptor absent, authority,
    // learned entry, not-declined) and the cadence gate below still apply.
    const bool exact_long_file_profile =
        (pick.mod == Modulation::QAM8 && pick.rate == CodeRate::R2_3 &&
         learned.cw_per_frame == 4) ||
        (pick.mod == Modulation::QPSK && pick.rate == CodeRate::R3_4 &&
         learned.cw_per_frame == 3);
    const bool same_rung_learned_z81_recovery =
        !use_burst_interleave_ &&
        exact_long_file_profile && learned.lifting_z == 81 &&
        active_lifting_z == 27 &&
        pick.mod == current_modulation_ && pick.rate == code_rate_ &&
        static_cast<int>(learned.cw_per_frame) == fixed_frame_codewords_;
    if (!broad_commanded_geometry && !same_rung_learned_z81_recovery) {
        return cg;
    }

    // Inert in steady state: only a genuine switch boundary is worth arming. If the
    // commanded rung IS what we are already configured for, the latched geometry is
    // already correct and this must be a byte-identical no-op. Lifting Z is part of
    // that identity: equal mod/rate/cw with learned Z81 and active Z27 is the live
    // lost-descriptor recovery case, not steady state.
    if (pick.mod == current_modulation_ && pick.rate == code_rate_ &&
        static_cast<int>(learned.cw_per_frame) == fixed_frame_codewords_ &&
        static_cast<int>(learned.lifting_z) == active_lifting_z) {
        return cg;
    }

    // GATE 2 — DEMOTE ONLY. This is the sender's own obey asymmetry, not a heuristic:
    // maybeObeyAuthorityCommand computes `faster = isFasterMode(cmd, current)` and,
    // when faster, RETURNS WITHOUT ARMING DEDUP while its TX window has bytes in
    // flight — an UP command can therefore stand un-adopted for many groups while the
    // receiver keeps re-stamping it, which is precisely the state in which arming it
    // would destroy a healthy group. A demote has no such path ("a failing window
    // never drains — waiting would deadlock"), so it is on the air by the next group.
    // Same predicate, same epsilon, as the sender: isFasterMode(), connection.cpp.
    // Physics corroborates the asymmetry: a demote is issued INTO a degrading channel
    // (where anchors are actually missed), while a climb is committed at a clean,
    // freshly-turned-around boundary where the anchor is most likely to be heard.
    constexpr float kModeEfficiencyEpsilonBps = 0.05f;  // isFasterMode's epsilon
    const bool commanded_is_faster =
        protocol::estimateWideOFDMRawBps(pick.mod, pick.rate) >
        protocol::estimateWideOFDMRawBps(current_modulation_, code_rate_) +
            kModeEfficiencyEpsilonBps;
    if (commanded_is_faster) return cg;

    // CADENCE GUARD. The command rides in the tone-burst ack. If that ack was LOST
    // the sender never switched and is resending the OLD geometry — applying the
    // commanded rung there would mirror the very bug we are fixing. A lost ack costs
    // the sender its full ARQ RTO, so the resend cannot arrive inside the
    // steady-state cadence. Measured 2026-07-28: steady-state 7.00-13.33 s (n=24)
    // vs post-RTO 19.43/20.01 s (n=2) — disjoint populations, threshold between.
    //   no reference (first group of a session, or just after a reset) -> allow: we
    //   cannot be resending a command we never sent (commanded_rung_idx_ is still
    //   kRungIdxNone then). reset() CLEARS last_group_start_abs_ precisely so that
    //   branch is taken deliberately — reset() also rewinds total_fed_ to 0, and a
    //   stale-high stamp would otherwise make `now_abs > last` false for minutes of
    //   audio, silently disabling this discriminator (2026-07-28 review fix). The
    //   strict '>' additionally prevents the unsigned underflow that would fake a
    //   giant gap.
    const uint64_t now_abs = absoluteForSyncPosition();
    if (last_group_start_abs_ != 0 && now_abs > last_group_start_abs_ &&
        (now_abs - last_group_start_abs_) >
            decode_policy::kBurstCadenceRtoGapSamples) {
        return cg;  // post-RTO: keep the latched geometry (today's behaviour)
    }

    cg.armed = true;
    cg.mod = pick.mod;
    cg.rate = pick.rate;
    cg.cw = static_cast<int>(learned.cw_per_frame);
    cg.lifting_z = static_cast<int>(learned.lifting_z);
    cg.frame_samples = static_cast<size_t>(ofdm_link_adaptation::minSamplesForCWCount(
        cg.cw, cg.mod, cg.rate, cg.lifting_z, ofdm_carriers_,
        std::max(1, waveform_->getSamplesPerSymbol())));
    if (cg.frame_samples == 0) {
        cg.armed = false;
    }
    return cg;
}

// A descriptor and its marked DATA head can both disappear in one fade while a
// later BI0 member's ordinary light-LTS remains perfectly usable. The control
// hypothesis must run first (all connected controls are short Z27 QPSK R1/4),
// but after that hypothesis fails we may recover the exact long DATA geometry
// already learned from an earlier descriptor for the same commanded rung.
//
// This resolver intentionally returns only per-frame geometry. It does not and
// cannot recover physical N, group sequence, or interleave provenance; callers
// therefore decode through the classic per-frame path and require the frame's
// CRC-protected PHYSICAL_BURST_END flag to prove the physical turn boundary.
StreamingDecoder::CommandedGeometry
StreamingDecoder::resolveClassicLongGeometry(bool is_ofdm,
                                             bool burst_latched,
                                             uint64_t frame_sync_abs) {
    CommandedGeometry cg;
    if (!waveform_ || !is_ofdm || !connected_ || burst_latched) return cg;
    if (mode_ != protocol::WaveformMode::OFDM_CHIRP) return cg;
    if (!burst_transport_rx_ || use_burst_interleave_) return cg;
    if (control_only_receive_.load(std::memory_order_acquire)) return cg;
    if (testing_lifting_z_override_ != 0) return cg;
    if (!anchored_burst_backstop_armed_ ||
        !sync_from_full_anchor_fallback_) {
        return cg;
    }
    if (!std::isfinite(sync_correlation_) ||
        sync_correlation_ < frame_policy::kMinSyncRecoveryCorrelation) {
        return cg;
    }
    if (pending_total_cw_ > 0 || sync_controller_.have_burst_descriptor_) {
        return cg;
    }
    if (!protocol::rxRateAuthorityEnabled()) return cg;

    // Tie the candidate to the accepted anchor's PHYSICAL sample interval. The
    // backstop timestamp is a decoder-feed watermark (total_fed_ at detection),
    // which may already lie beyond this candidate under backlog and therefore is
    // not an ordering key. The provisional gate retains the exact anchor training
    // position and its conservative physical-turn end.
    if (!burst_air_end_provisional_.load(std::memory_order_relaxed) ||
        !provisional_burst_anchor_full_chirp_.load(std::memory_order_relaxed)) {
        return cg;
    }
    const uint64_t anchor_abs =
        provisional_burst_anchor_abs_.load(std::memory_order_relaxed);
    const uint64_t physical_end_abs =
        burst_air_end_abs_.load(std::memory_order_relaxed);
    if (frame_sync_abs < anchor_abs || physical_end_abs <= anchor_abs ||
        frame_sync_abs > physical_end_abs) {
        return cg;
    }

    const uint8_t idx = commanded_rung_idx_.load(std::memory_order_relaxed);
    if (idx == protocol::kRungIdxNone || idx >= protocol::kRungIdxCount) {
        return cg;
    }
    if (commanded_rung_declined_.load(std::memory_order_relaxed)) return cg;
    const LearnedRungGeometry learned = learnedRungGeometrySnapshot(idx);
    const auto pick = protocol::coherentRungFromIndex(idx);
    const bool exact_long_file_profile =
        (pick.mod == Modulation::QAM8 && pick.rate == CodeRate::R2_3 &&
         learned.cw_per_frame == 4) ||
        (pick.mod == Modulation::QPSK && pick.rate == CodeRate::R3_4 &&
         learned.cw_per_frame == 3);
    if (!exact_long_file_profile || learned.lifting_z != 81) return cg;
    if (activeBurstLiftingZ() != 27) return cg;
    if (pick.mod != current_modulation_ || pick.rate != code_rate_ ||
        static_cast<int>(learned.cw_per_frame) != fixed_frame_codewords_) {
        return cg;
    }

    // Reuse the command-adoption cadence discriminator at the accepted ANCHOR,
    // not this later member. A healthy long burst can put its tail >15 s after
    // the previous group's head (the live recovery candidate was +15.55 s), even
    // though the new turn's accepted anchor arrived at steady cadence (+10.73 s).
    // A post-RTO ANCHOR may still be old Z27 after a lost authority ACK; applying
    // learned Z81 there would turn a recoverable resend into a deterministic bad
    // slice.
    if (last_group_start_abs_ != 0 && anchor_abs > last_group_start_abs_ &&
        (anchor_abs - last_group_start_abs_) >
            decode_policy::kBurstCadenceRtoGapSamples) {
        return cg;
    }

    cg.armed = true;
    cg.mod = pick.mod;
    cg.rate = pick.rate;
    cg.cw = static_cast<int>(learned.cw_per_frame);
    cg.lifting_z = 81;
    cg.frame_samples = static_cast<size_t>(
        ofdm_link_adaptation::minSamplesForCWCount(
            cg.cw, cg.mod, cg.rate, cg.lifting_z, ofdm_carriers_,
            std::max(1, waveform_->getSamplesPerSymbol())));
    if (cg.frame_samples == 0) cg.armed = false;
    return cg;
}

void StreamingDecoder::armClassicLongGeometry(
        const CommandedGeometry& geometry) {
    if (!geometry.armed || geometry.lifting_z != 81 || !waveform_) return;
    classic_long_geometry_recovery_active_ = true;
    classic_long_geometry_sync_pos_ = sync_position_ + 1;
    pending_total_cw_ = geometry.cw;
    // This accepted full-anchor epoch is the current physical group attempt.
    // Advance the cadence reference even though its descriptor/marked head were
    // lost; otherwise a second consecutive loss is measured from the last fully
    // framed group and is falsely classified as post-RTO.
    last_group_start_abs_ =
        provisional_burst_anchor_abs_.load(std::memory_order_relaxed);
    waveform_->setActiveLDPCLiftingZ(81);
    rebuildFrameDecoderInterleaverForLiftingZ(81);
    ++classic_long_geometry_arms_;
    LOG_MODEM(WARN,
              "[%s] [CLASSIC-LONG-Z] descriptor/head missed — retrying accepted "
              "BI0 DATA fallback as %s %s cw=%d z=81 (%zu samples); physical N "
              "remains unknown and only a CRC tail may close the turn. arm #%u",
              log_prefix_.c_str(), modulationToString(geometry.mod),
              codeRateToString(geometry.rate), geometry.cw,
              geometry.frame_samples, classic_long_geometry_arms_);
}

void StreamingDecoder::clearClassicLongGeometry() {
    if (!classic_long_geometry_recovery_active_) return;
    classic_long_geometry_recovery_active_ = false;
    classic_long_geometry_sync_pos_ = 0;
    const int target_z = testing_lifting_z_override_ != 0
        ? testing_lifting_z_override_
        : sync_controller_.activeBurstLiftingZ();
    if (waveform_) {
        waveform_->setActiveLDPCLiftingZ(static_cast<uint8_t>(target_z));
    }
    rebuildFrameDecoderInterleaverForLiftingZ(target_z);
}

bool StreamingDecoder::classicLongGeometryEligibleForTesting(
        float correlation, bool burst_latched,
        uint64_t prior_group_start_abs,
        uint64_t anchor_abs,
        uint64_t candidate_abs) {
    const bool saved_backstop = anchored_burst_backstop_armed_;
    const size_t saved_arm_abs = anchored_burst_backstop_arm_abs_;
    const bool saved_fallback = sync_from_full_anchor_fallback_;
    const float saved_correlation = sync_correlation_;
    const bool saved_provisional =
        burst_air_end_provisional_.load(std::memory_order_relaxed);
    const uint64_t saved_anchor =
        provisional_burst_anchor_abs_.load(std::memory_order_relaxed);
    const bool saved_full_chirp =
        provisional_burst_anchor_full_chirp_.load(std::memory_order_relaxed);
    const uint64_t saved_air_end =
        burst_air_end_abs_.load(std::memory_order_relaxed);
    const uint64_t saved_group_start = last_group_start_abs_;
    anchored_burst_backstop_armed_ = true;
    anchored_burst_backstop_arm_abs_ = 0;
    sync_from_full_anchor_fallback_ = true;
    sync_correlation_ = correlation;
    burst_air_end_provisional_.store(true, std::memory_order_relaxed);
    provisional_burst_anchor_abs_.store(anchor_abs, std::memory_order_relaxed);
    provisional_burst_anchor_full_chirp_.store(true,
                                               std::memory_order_relaxed);
    burst_air_end_abs_.store(candidate_abs + 1, std::memory_order_relaxed);
    last_group_start_abs_ = prior_group_start_abs;
    const bool eligible = resolveClassicLongGeometry(
        protocol::isOFDMMode(mode_), burst_latched, candidate_abs).armed;
    anchored_burst_backstop_armed_ = saved_backstop;
    anchored_burst_backstop_arm_abs_ = saved_arm_abs;
    sync_from_full_anchor_fallback_ = saved_fallback;
    sync_correlation_ = saved_correlation;
    burst_air_end_provisional_.store(saved_provisional,
                                     std::memory_order_relaxed);
    provisional_burst_anchor_abs_.store(saved_anchor,
                                        std::memory_order_relaxed);
    provisional_burst_anchor_full_chirp_.store(saved_full_chirp,
                                               std::memory_order_relaxed);
    burst_air_end_abs_.store(saved_air_end, std::memory_order_relaxed);
    last_group_start_abs_ = saved_group_start;
    return eligible;
}

void StreamingDecoder::decodeCurrentFrame() {
    if (!waveform_) {
        clearClassicLongGeometry();
        {
            std::lock_guard<std::mutex> lock(sync_controller_.ring_.buffer_mutex_);
            sync_controller_.ring_.correlation_pos_ = sync_controller_.ring_.wrapRingIndexLocked(sync_position_ + 4800);
        }
        state_ = DecoderState::SEARCHING;
        return;
    }

    // A classic-long arm spans two decode iterations: the first performs the
    // hardened control-sized hypothesis and arms the exact long wait; the second
    // owns the full Z81 slice. Keep the temporary geometry only while that same
    // candidate explicitly returns to SYNC_FOUND awaiting more samples. Every
    // terminal success/failure/search/reset path restores ordinary Z27 after all
    // decode and consumed-stride calculations have finished.
    const bool classic_long_on_entry =
        classic_long_geometry_recovery_active_ &&
        classic_long_geometry_sync_pos_ == sync_position_ + 1;
    auto classic_long_cleanup = [this, classic_long_on_entry](void*) {
        if (!classic_long_on_entry) return;
        const bool same_candidate_still_waiting =
            classic_long_geometry_recovery_active_ &&
            classic_long_geometry_sync_pos_ == sync_position_ + 1 &&
            state_ == DecoderState::SYNC_FOUND && pending_total_cw_ > 0;
        if (!same_candidate_still_waiting) {
            clearClassicLongGeometry();
        }
    };
    std::unique_ptr<void, decltype(classic_long_cleanup)>
        classic_long_scope(classic_long_on_entry ? static_cast<void*>(this) : nullptr,
                           classic_long_cleanup);

    const uint32_t gen_at_start = reset_generation_.load(std::memory_order_acquire);
    const uint64_t geometry_session_epoch_at_start =
        learnedRungGeometrySessionEpochSnapshot();
    auto resetDuringDecode = [this, gen_at_start]() {
        return reset_generation_.load(std::memory_order_acquire) != gen_at_start;
    };
    bool is_ofdm = protocol::isOFDMMode(mode_);

    // Determine how many samples to copy from buffer
    // Strategy depends on mode and state:
    // - pending_total_cw_ > 0: exact size from prior CW0 peek
    // - Connected OFDM, first pass: control-sized peek, then configured fixed-CW data buffer
    // - MC-DPSK: 1-CW for peek (MC-DPSK getMinSamplesForFrame() == 1 CW by design)
    // - Disconnected: full frame (always MC-DPSK for handshake)
    // 2026-05-29 channel-adaptive interleaver (RX decouple): burst_latched = "group-start
    // marker detected" (NOT gated on the interleave flag). The descriptor's
    // pending_total_cw_ is cleared during the SEARCH→sync transition, so the group-start
    // frame is sized via the ConnectedOFDMBurst fallback (full data frame) keyed on this
    // latch + the burst regime. Without dropping the use_burst_interleave_ gate here, an
    // interleave-off group-start sized as a control peek, never decoded as data, and never
    // reached the burst-accumulation path — the whole group stalled (verified bi0_s1).
    const bool control_only_receive =
        is_ofdm && connected_ &&
        control_only_receive_.load(std::memory_order_acquire);
    // A speculative DATA peek may have armed a fixed-CW continuation immediately
    // before the lifecycle entered DISCONNECTING.  Retire that decoder-owned
    // continuation at this decode-thread boundary so teardown always starts with
    // the normal hardened control peek.  Likewise, ignore a sign-only burst marker:
    // DATA groups are not legal during this phase.
    if (control_only_receive && pending_total_cw_ > 0) {
        LOG_MODEM(INFO,
                  "[%s] Control-only RX: discarding pending %d-CW DATA escalation",
                  log_prefix_.c_str(), pending_total_cw_);
        pending_total_cw_ = 0;
    }
    const bool burst_latched =
        !control_only_receive && waveform_ && waveform_->wasBurstInterleaved();
    const bool burst_regime_active = use_burst_interleave_ || burst_transport_rx_;
    const size_t pending_samples = pending_total_cw_ > 0
        ? static_cast<size_t>(waveform_->getMinSamplesForCWCount(pending_total_cw_))
        : 0;
    const size_t ofdm_control_samples = (is_ofdm && connected_)
        ? getOFDMControlFrameSamplesForCurrentMode()
        : 0;
    // BUG-BURST-STALE-GEOMETRY E8a: when the group's descriptor was MISSED, size the
    // group-start frame from the rung WE COMMANDED, not the latched one. This tuple
    // has THREE consumers (this slice length; the readiness mirror in
    // checkIfReadyToDecode; burst_min_block_ at the marker) — all three must resolve
    // IDENTICALLY or the group is misaligned even with a correct stride. So re-use the
    // readiness resolve rather than resolving again: DECODING is only ever entered from
    // checkIfReadyToDecode (streaming_sync_acquisition.cpp), and the fresh-resolve
    // fallback below exists purely so a future entry path cannot silently read stale
    // state.
    const auto cg = (cg_snapshot_sync_pos_ == sync_position_ + 1)
        ? cg_snapshot_
        : resolveCommandedGeometry(is_ofdm, burst_latched);
    const size_t full_frame_samples = cg.armed
        ? cg.frame_samples
        : ((is_ofdm && connected_)
               ? static_cast<size_t>(
                     waveform_->getMinSamplesForCWCount(fixed_frame_codewords_))
               : 0);
    const size_t control_frame_samples =
        static_cast<size_t>(waveform_->getMinSamplesForControlFrame());
    auto requirement = decode_policy::selectDecodeSampleRequirement(
        pending_total_cw_,
        is_ofdm,
        connected_,
        burst_regime_active,
        burst_latched,
        pending_samples,
        ofdm_control_samples,
        full_frame_samples,
        control_frame_samples);
    size_t frame_len = requirement.samples;

    // 2026-07-28 DIAGNOSTIC (ULTRA_EQ_TRACE=1)
    if (ultra::genie::eqTrace().enabled) {
        std::fprintf(stderr,
                     "[eqtrace] >>> decodeCurrentFrame sync_pos=%zu pending_cw=%d fixed_cw=%d "
                     "req_mode=%d req_samples=%zu ctrl_samples=%zu burst_regime=%d latched=%d\n",
                     sync_position_, pending_total_cw_, fixed_frame_codewords_,
                     static_cast<int>(requirement.mode), requirement.samples,
                     control_frame_samples, burst_regime_active ? 1 : 0,
                     burst_latched ? 1 : 0);
    }

    // Copy frame samples from buffer
    std::vector<float> frame_buffer;
    size_t frame_sync_abs = 0;
    bool frame_truncated = false;
    {
        std::lock_guard<std::mutex> lock(sync_controller_.ring_.buffer_mutex_);
        frame_sync_abs = sync_controller_.ring_.ringPosToAbsoluteLocked(sync_position_);

        size_t available;
        if (sync_controller_.ring_.write_pos_ >= sync_position_) {
            available = sync_controller_.ring_.write_pos_ - sync_position_;
        } else {
            available = sync_controller_.ring_.buffer_capacity_samples_ - sync_position_ + sync_controller_.ring_.write_pos_;
        }

        frame_len = std::min(frame_len, available);
        frame_truncated = frame_len < requirement.samples;

        frame_buffer.resize(frame_len);
        for (size_t i = 0; i < frame_len; i++) {
            frame_buffer[i] = sync_controller_.ring_.buffer_[sync_controller_.ring_.wrapRingIndexLocked(sync_position_ + i)];
        }
    }

    // ========================================================================
    // BUG-BURST-STALE-GEOMETRY defect (A): NEVER ARM A BURST GROUP FROM A
    // TRUNCATED FRAME. This is the half that produces the operator-visible
    // TIMEOUT, and it is independent of the stale-constellation half (B).
    // ========================================================================
    // checkIfReadyToDecode released this frame once `available` cleared the
    // requirement IT computed. `available` only grows (write_pos advances,
    // sync_position_ is fixed), so reaching here with frame_len < requirement
    // means the REQUIREMENT GREW between the readiness check and this decode.
    // Three ways that happens, all live:
    //   1. applyPendingConnectedOFDMMode/applyPendingDescriptorDataMode run at the
    //      TOP of the processBuffer iteration that calls us — i.e. strictly after
    //      the SYNC_FOUND iteration that gated on the old profile. A rate change
    //      moves pilot spacing -> bits/symbol -> samples.
    //   2. setFixedFrameCodewords is written from the GUI thread (plain int).
    //   3. the commanded-rung tuple (B) can change between the two resolves — the
    //      ack-emit thread writes commanded_rung_idx_.
    // The damage is silent and structural: burst_next_pos_ and
    // burst_data_start_abs_ are both derived from frame_len, so a short slice
    // shifts the WHOLE group one partial frame early; refreshBurstAirEnd() then
    // inherits it (burst_data_start_abs_ + N*burst_min_block_) and the F176
    // half-duplex ack interlock opens INSIDE the sender's own key-down — the ack
    // is transmitted into our peer's transmission, is lost, and the sender eats a
    // full ARQ RTO (measured 17.5 s; 28.2 s between useful acks).
    // FIX: do not decode a truncated burst frame at all. Return to SYNC_FOUND with
    // sync_position_ untouched; checkIfReadyToDecode re-gates on the CURRENT
    // requirement and releases the frame again once the audio has arrived.
    // BOUNDED BY PROGRESS, NOT BY A TIMER (2026-07-28 review fix). Holding until the
    // readiness timeout expires would DROP the frame to SEARCHING, and on a missed
    // descriptor `have_burst_descriptor_` is false, so neither the group-abandonment
    // backstop NACK nor the F165 anchored-burst backstop would fire — the group would
    // vanish with NO tone-ack at all and the sender would eat the full ARQ RTO. That
    // is the ❌ "refuse to arm" trap KNOWN_BUGS names explicitly, and it would be a
    // strictly worse failure than the one being fixed. So: hold only while the missing
    // audio is still ARRIVING. The first hold that makes no progress means the peer
    // has stopped transmitting and the frame can never complete — fall through to the
    // pre-fix truncated decode so the group ARMS and its ack still emits. processBuffer
    // performs exactly one state step per audio wake-up, so a hold costs one wake-up.
    // SCOPE: connected wideband/narrow OFDM burst-regime frames only — the two
    // requirement modes that can arm a group (ConnectedOFDMBurst = marker-latched
    // group start, PendingCodewords = descriptor-declared group start). Peek/control
    // sizing is untouched. In steady state the requirement does not move between the
    // two calls, so this guard is a strict no-op.
    // A held episode is complete as soon as the same frame reaches its full
    // requirement. Clear the ring-position key before decoding it; otherwise a
    // later circular-buffer wrap at the same index can inherit the old episode
    // and falsely take the "audio stopped" fallthrough on its first attempt.
    if (!frame_truncated && truncation_hold_sync_pos_ != 0) {
        truncation_hold_sync_pos_ = 0;
        truncation_hold_frame_len_ = 0;
    }

    if (frame_truncated && is_ofdm && connected_ && burst_regime_active &&
        (requirement.mode == decode_policy::DecodeSampleMode::ConnectedOFDMBurst ||
         requirement.mode == decode_policy::DecodeSampleMode::PendingCodewords)) {
        const bool same_episode = (truncation_hold_sync_pos_ == sync_position_ + 1);
        const bool progressing = !same_episode || frame_len > truncation_hold_frame_len_;
        if (progressing) {
            // Count and log EPISODES, not retries: one deferral typically re-arms on
            // the next few audio chunks, and a per-retry WARN would spam the log.
            if (!same_episode) {
                truncation_hold_sync_pos_ = sync_position_ + 1;  // +1 so 0 stays "none"
                ++truncated_burst_holds_;
                LOG_MODEM(WARN,
                          "[%s] Burst group-start frame TRUNCATED (%zu < %zu samples) — "
                          "deferring arm (geometry grew between readiness and decode; "
                          "arming here would shift the whole group early). hold #%u",
                          log_prefix_.c_str(), frame_len, requirement.samples,
                          truncated_burst_holds_);
            }
            truncation_hold_frame_len_ = frame_len;
            state_ = DecoderState::SYNC_FOUND;
            return;
        }
        LOG_MODEM(WARN,
                  "[%s] Burst group-start frame still TRUNCATED (%zu < %zu) and the "
                  "audio STOPPED arriving — proceeding with the short slice so the "
                  "group still arms and still ACKs (a silent drop would cost a full RTO)",
                  log_prefix_.c_str(), frame_len, requirement.samples);
        truncation_hold_sync_pos_ = 0;
        truncation_hold_frame_len_ = 0;
    }

    // CFO pre-correction: remove known CFO from raw samples so the entire
    // demodulation chain sees a clean, CFO-free signal.  The waveform/demodulator
    // is then told CFO=0 and only needs to handle small residuals.
    const bool timing_cfo_genie =
        qam16GenieTimingCfoEnabled() &&
        is_ofdm &&
        connected_ &&
        current_modulation_ == Modulation::QAM16;
    if (timing_cfo_genie) {
        frame_demodulator_.resetPreCorrection();
        LOG_MODEM(WARN,
                  "[%s] DIAG genie-timing-cfo: bypassing CFO pre-correction "
                  "and forcing demod CFO to 0 Hz",
                  log_prefix_.c_str());
    } else if (is_ofdm && !frame_buffer.empty()) {
        frame_demodulator_.applyCFOPreCorrection(frame_buffer, sync_cfo_, frame_sync_abs, log_prefix_.c_str());
    }

    // After pre-correction, tell waveform CFO=0 (already removed from samples).
    // For non-OFDM (MC-DPSK), no pre-correction — pass original sync_cfo_.
    const float decode_cfo = timing_cfo_genie ? 0.0f
        : ((is_ofdm && std::abs(frame_demodulator_.preCorrectionCfo()) > 0.01f) ? 0.0f : sync_cfo_);

    if (frame_buffer.empty()) {
        {
            std::lock_guard<std::mutex> lock(sync_controller_.ring_.buffer_mutex_);
            sync_controller_.ring_.correlation_pos_ = sync_controller_.ring_.wrapRingIndexLocked(sync_position_ + 4800);
            sync_controller_.ring_.setSearchFloorLocked(frame_sync_abs + 4800);
        }
        state_ = DecoderState::SEARCHING;
        return;
    }

    // Invalid connected-OFDM LTS locks are often silence/payload autocorr peaks
    // just before the next real LTS. Advancing by a whole frame can skip the real
    // frame; advance only far enough to avoid re-locking the same false peak.
    auto advancePastFalseOFDMLock = [&]() {
        const int data_preamble = waveform_ ? waveform_->getDataPreambleSamples() : 0;
        const size_t advance = frame_policy::falseOFDMLockAdvanceSamples(
            frame_len, data_preamble);

        std::lock_guard<std::mutex> lock(sync_controller_.ring_.buffer_mutex_);
        sync_controller_.ring_.correlation_pos_ = sync_controller_.ring_.wrapRingIndexLocked(sync_position_ + advance);
        sync_controller_.ring_.setSearchFloorLocked(frame_sync_abs + advance);
        if (sync_from_warm_timed_window_) {
            noteFrameArrivalSyncMissLocked();
            sync_from_warm_timed_window_ = false;
        }
    };

    // PING is a disconnected MC-DPSK chirp-only presence probe. Do not run this
    // RMS heuristic for connected OFDM light-preamble frames: valid data/control
    // frames must be accepted or rejected by the demodulator + CRC/FEC path.
    const bool allow_ping_detection = !connected_ && mode_ == protocol::WaveformMode::MC_DPSK;

    // ROBUST LOW-SNR IDLE PING (#70): now DEFAULT-ON; opt OUT via ULTRA_ROBUST_IDLE_PING=0.
    // The ping/connect classifier is level/ratio-based (kPingMaxDataToTrainingRMSRatio
    // = 0.5, absolute floor 0.16). A bare PING is just a chirp with a SILENT data
    // region; at low SNR the broadband noise floods that region (data/train RMS ratio
    // rises to 0.68-0.88 well above 0.5) so a real PING reads as a faded CONNECT and
    // is sent to wait for a 4-CW frame that never comes -> no PONG -> never connects
    // (sim floor: never below ~15 dB Good, vs the chirp itself locking solidly at
    // corr 0.6-0.75 to ~6 dB AWGN). When ON, a solid chirp-lock (corr >= floor, gap
    // OK) emits the PING on the chirp signature alone, independent of the data-region
    // level. SAFE-by-construction: only frames that ALREADY tripped the pre-LDPC
    // false-lock reject (low LLR) reach the robust emit, so a cleanly decodable
    // CONNECT/CONNECT_ACK (good LLR -> CW0 magic peek) is never short-circuited. Both
    // starvation directions are now GATED: initiator #27 by bare_chirp_expected_=FALSE
    // during CONNECTING, responder by STAGE2 (app.cpp: PONG -> expects-CONNECT window).
    // PROMOTED default-ON 2026-07-01. A first flip regressed the good@20 sim gate — the
    // ratio-based data_bearing test fired the robust emit on strong high-SNR false-syncs
    // (data_rms ~0.3) as well as genuine flooded PINGs -> PING/PONG churn. Fixed by the
    // high-SNR-safe emit gate below (data_rms <= kPingChirpLockMaxDataRMS separates the two;
    // measured good@20 0.28-0.33 vs rig low-SNR 0.04-0.09). Now: good@20 PASS with the emit
    // active; rig low-SNR connect (MPG@9/MPM@8) preserved. Both starvation directions gated
    // (initiator #27 + responder STAGE2). Opt OUT via ULTRA_ROBUST_IDLE_PING=0.
    static const bool robust_idle_ping = [] {
        const char* v = std::getenv("ULTRA_ROBUST_IDLE_PING");
        return !(v && v[0] == '0');  // default-ON; "0" opts out
    }();
    const size_t mc_dpsk_symbol_samples =
        (allow_ping_detection && waveform_)
            ? static_cast<size_t>(std::max(1, waveform_->getSamplesPerSymbol()))
            : 512;
    const size_t ping_training_skip = 9 * mc_dpsk_symbol_samples;
    const size_t ping_check_samples = std::max(
        frame_policy::kPingRMSCheckSamples, 3 * mc_dpsk_symbol_samples);
    auto evaluatePingDecision = [&](bool ldpc_attempted,
                                    bool ldpc_decode_succeeded,
                                    bool ldpc_magic_valid) {
        // #70 stage 1.5: give the classifier the receiver's idle noise floor so
        // ping_by_chirp_lock can use the level-invariant noise-relative gap test
        // (a flooded PING gap reads ≈ the noise floor at any SNR; the absolute
        // 0.16 floor fails above ~8 dB of broadband noise). BOTH sides in-band:
        // the ambient reference is the estimator's post-FIR value, and the gap
        // RMS is measured through an identical FIR here. In the raw domain the
        // out-of-band noise fraction compresses payload-vs-noise separation to
        // ~1.5x at good@10 (a real CONNECT misclassified as PING); in-band the
        // separation is sqrt(1+SNR_lin) ≈ 3.3x. Knob-off passes 0, which
        // disables the noise arm inside evaluatePingFrame.
        float idle_noise_rms = 0.0f;
        float data_inband_rms = 0.0f;
        // #27 PROTOCOL-STATE GATE (rig F225 regression fix): the noise-relative
        // chirp-lock arm may only classify PING when a bare chirp is PLAUSIBLE
        // (idle / probing — bare_chirp_expected_). While CONNECTING the only
        // chirp-locked frame we expect is the CONNECT_ACK; a deep fade makes its
        // payload read at the noise floor, and PONGing it away destroys the
        // handshake the peer is retrying (F225: Pi5 PONG'd the Mac's faded
        // CONNECT_ACK -> 10 CONNECT retries -> half-open). With the arm off the
        // frame is a plain decode failure -> retries/HARQ do their job. The old
        // pre-unification emit had exactly this gate; restore it at the single
        // choke point (idle_noise_rms=0 disables the arm in evaluatePingFrame).
        if (robust_idle_ping &&
            bare_chirp_expected_.load(std::memory_order_relaxed)) {
            // Reference preference: (1) THIS frame's inter-chirp-gap RMS —
            // burst-time, same channel-noise state and fade state as the frame
            // (S:N-holding channel sims scale idle noise differently from
            // burst noise — F224); (2) fallback: idle min-statistics floor.
            if (sync_noise_ref_rms_ > frame_policy::kMinIdleNoiseRMSForPingGap) {
                idle_noise_rms = sync_noise_ref_rms_;
            } else {
                const auto ns = idle_noise_snr_estimator_.snapshot();
                if (ns.valid) idle_noise_rms = ns.floor_noise_rms;
            }
            if (idle_noise_rms > 0.0f && !frame_buffer.empty()) {
                // Same design as IdleNoiseSNREstimator's reference filter.
                // ONE filter pass over [0, check_end): the training span
                // [0, training_skip) feeds the PHYSICAL channel-SNR estimate
                // (power ratio vs the burst-time noise ref — no phase model,
                // so unlike the coherence-weighted mcdpsk_in_band estimator it
                // reads the CHANNEL, not the demod's usable margin), and the
                // filter is fully settled by the gap window it doubles as
                // priming for.
                FIRFilter gap_filter = FIRFilter::bandpass(101, 50.0f, 2950.0f, 48000.0f);
                const size_t check_start =
                    std::min(ping_training_skip, frame_buffer.size());
                const size_t check_len =
                    std::min(frame_buffer.size() - check_start, ping_check_samples);
                double train_sum_sq = 0.0;
                for (size_t i = 0; i < check_start; ++i) {
                    const float y = gap_filter.process(frame_buffer[i]);
                    train_sum_sq += static_cast<double>(y) * static_cast<double>(y);
                }
                // (The physical-SNR latch moved into the waveform's exact
                // training-span decode — handoff §2; the loop above remains
                // as FIR priming for the gap window below.)
                (void)train_sum_sq;
                if (check_len > 0) {
                    double sum_sq = 0.0;
                    for (size_t i = check_start; i < check_start + check_len; ++i) {
                        const float y = gap_filter.process(frame_buffer[i]);
                        sum_sq += static_cast<double>(y) * static_cast<double>(y);
                    }
                    data_inband_rms = static_cast<float>(
                        std::sqrt(sum_sq / static_cast<double>(check_len)));
                }
            }
        }
        return frame_policy::evaluatePingFrame(
            frame_buffer.data(), frame_buffer.size(), ping_training_skip,
            ping_check_samples, sync_correlation_, sync_gap_error_samples_,
            ldpc_decode_succeeded, ldpc_magic_valid, ldpc_attempted,
            idle_noise_rms, data_inband_rms);
    };
    auto emitPingFrame = [&](const frame_policy::PingFrameDecision& ping_decision,
                             bool ldpc_attempted) {
        LOG_MODEM(INFO, "[%s] PING detected: path1=%d path2=%d ratio=%.3f "
                  "chirp_corr=%.3f gap_error=%.1f ldpc_attempted=%d "
                  "ldpc_ok=%s magic=%s "
                  "SNR=%.1f dB (%s) CFO=%.1f Hz",
                  log_prefix_.c_str(), ping_decision.ping_by_silence ? 1 : 0,
                  ping_decision.ping_by_chirp_lock ? 1 : 0,
                  ping_decision.ratio, ping_decision.chirp_corr,
                  ping_decision.gap_error_samples,
                  ldpc_attempted ? 1 : 0,
                  ldpc_attempted
                      ? (ping_decision.ldpc_decode_succeeded ? "1" : "0")
                      : "skipped",
                  ldpc_attempted
                      ? (ping_decision.ldpc_magic_valid ? "1" : "0")
                      : "skipped",
                  sync_snr_, snrSourceToString(SNRSource::SYNC_QUALITY), sync_cfo_);

        DecodeResult ping;
        ping.success = true;
        ping.is_ping = true;
        ping.frame_type = v2::FrameType::PING;
        ping.snr_db = sync_snr_;
        ping.snr_source = SNRSource::SYNC_QUALITY;
        ping.sync_quality_db = sync_snr_;
        ping.cfo_hz = sync_cfo_;
        ping.sync_correlation = sync_correlation_;
        ping.ping_training_rms = ping_decision.training_rms;
        ping.ping_data_rms = ping_decision.data_rms;
        ping.ping_data_to_training_ratio = ping_decision.ratio;
        ping.ping_chirp_corr = ping_decision.chirp_corr;
        ping.ping_gap_error_samples = ping_decision.gap_error_samples;
        ping.ping_by_silence = ping_decision.ping_by_silence;
        ping.ping_by_chirp_lock = ping_decision.ping_by_chirp_lock;
        ping.ping_ldpc_attempted = ldpc_attempted;
        ping.ping_ldpc_decode_succeeded = ping_decision.ldpc_decode_succeeded;
        ping.ping_ldpc_magic_valid = ping_decision.ldpc_magic_valid;

        {
            std::lock_guard<std::mutex> qlock(queue_mutex_);
            frame_queue_.push(ping);
        }

        if (frame_callback_) {
            frame_callback_(ping);
            if (resetDuringDecode()) {
                return true;
            }
        }
        if (ping_callback_) {
            ping_callback_(sync_snr_, sync_cfo_);
            if (resetDuringDecode()) {
                return true;
            }
        }

        {
            std::lock_guard<std::mutex> slock(stats_mutex_);
            stats_.pings_received++;
        }

        // Skip past the PING.
        {
            std::lock_guard<std::mutex> lock(sync_controller_.ring_.buffer_mutex_);
            size_t min_frame = static_cast<size_t>(waveform_->getMinSamplesForFrame());
            sync_controller_.ring_.correlation_pos_ = sync_controller_.ring_.wrapRingIndexLocked(sync_position_ + min_frame);
            sync_controller_.ring_.setSearchFloorLocked(frame_sync_abs + min_frame);
            last_decoded_sync_pos_ = sync_position_;
        }

        state_ = DecoderState::SEARCHING;
        return true;
    };
    auto tryEmitPingByChirpLock = [&](const char* reason, bool ldpc_attempted) {
        const auto ping_decision = evaluatePingDecision(ldpc_attempted, false, false);

        LOG_MODEM(INFO, "[%s] PING check PATH2: ratio=%.3f, "
                  "chirp_corr=%.3f (floor=%.2f), gap_error=%.1f (max=%.0f), "
                  "valid_frame=0, reason=%s, inband_rms=%.4f noise_rms=%.4f "
                  "gap_is_noise=%d, path1=%d, path2=%d",
                  log_prefix_.c_str(), ping_decision.ratio,
                  ping_decision.chirp_corr, frame_policy::kPingCorrFloor,
                  ping_decision.gap_error_samples, frame_policy::kPingMaxGapError,
                  reason,
                  ping_decision.data_inband_rms, ping_decision.idle_noise_rms,
                  ping_decision.gap_is_noise ? 1 : 0,
                  ping_decision.ping_by_silence ? 1 : 0,
                  ping_decision.ping_by_chirp_lock ? 1 : 0);

        if (ping_decision.ping_by_chirp_lock) {
            emitPingFrame(ping_decision, ldpc_attempted);
            return true;
        }
        return false;
    };

    if (allow_ping_detection) {
        // PATH 1: clean-cable/AWGN silence after training. LDPC inputs are set
        // true here only to keep the PATH 2 fallback disabled until LDPC has
        // actually been attempted below.
        const auto ping_decision = evaluatePingDecision(false, true, true);

        LOG_MODEM(INFO, "[%s] PING check: RMS=%.4f, train_RMS=%.4f, "
                  "ratio=%.3f (threshold=%.1f), chirp_corr=%.3f, "
                  "gap_error=%.1f, path1=%d, skip=%zu, sync_pos=%zu",
                  log_prefix_.c_str(), ping_decision.data_rms,
                  ping_decision.training_rms, ping_decision.ratio,
                  frame_policy::kPingMaxDataToTrainingRMSRatio,
                  ping_decision.chirp_corr, ping_decision.gap_error_samples,
                  ping_decision.ping_by_silence ? 1 : 0,
                  ping_training_skip, sync_position_);

        if (ping_decision.ping_by_silence) {
            emitPingFrame(ping_decision, false);
            return;
        }
    }

    // Connected OFDM control-first hypothesis:
    // Try the control profile before using the data profile. OFDM is coherent-only,
    // so control always rides coherent QPSK R1/4 (same receiver family as the data).
    const bool first_pass_ofdm_peek = frame_policy::shouldRunControlFirstOFDMPeek(
        pending_total_cw_, is_ofdm, connected_, frame_len,
        getOFDMControlFrameSamplesForCurrentMode());
    if (first_pass_ofdm_peek) {
        constexpr size_t CONTROL_LDPC_BLOCK = v2::LDPC_CODEWORD_BITS;
        Modulation saved_mod = current_modulation_;
        CodeRate saved_rate = code_rate_;
        const auto control_profile =
            streaming_control_profile::profileForDataMode(saved_mod);
        bool switched_profile =
            (saved_mod != control_profile.modulation ||
             saved_rate != control_profile.rate);

        if (switched_profile) {
            LOG_MODEM(DEBUG, "[%s] OFDM control-first profile: %s %s (data=%s %s)",
                      log_prefix_.c_str(),
                      modulationToString(control_profile.modulation),
                      codeRateToString(control_profile.rate),
                      modulationToString(saved_mod), codeRateToString(saved_rate));
            waveform_->configure(control_profile.modulation, control_profile.rate);
        }

        waveform_->setFrequencyOffset(decode_cfo);
        ultra::genie::eqTrace().site = "ctrl-first-peek";
        bool control_ok = processWaveformForCodewords(
            SampleSpan(frame_buffer.data(), frame_buffer.size()), 1);
        if (control_ok) {
            float lts_signal_power = 1.0f;
            float lts_channel_mag = 1.0f;
            if (hasInvalidOFDMTraining(waveform_.get(), is_ofdm, connected_,
                                       lts_signal_power, lts_channel_mag)) {
                LOG_MODEM(INFO, "[%s] False control chirp lock rejected: "
                          "lts_signal=%.6f, |H|_avg=%.4f — re-searching",
                          log_prefix_.c_str(), lts_signal_power, lts_channel_mag);
                if (switched_profile) {
                    waveform_->configure(saved_mod, saved_rate);
                }
                advancePastFalseOFDMLock();
                state_ = DecoderState::SEARCHING;
                return;
            }

            captureConstellationSnapshot();
            auto control_soft_bits = waveform_->getSoftBits();
            if (control_soft_bits.size() >= CONTROL_LDPC_BLOCK) {
                // LLR pre-screen: skip the 1-CW decode (~85ms incl. retries)
                // when the soft bits look like noise. Real ACKs cluster well
                // above this threshold; false-sync attempts cluster near 1.
                std::pair<bool, Bytes> control_decode = {false, {}};
                const float llr_avg = signal_policy::meanAbsLLR(
                    control_soft_bits.data(), CONTROL_LDPC_BLOCK);
                if (llr_avg < signal_policy::kMinLLRForSingleCWDecode) {
                    ultra::timing::globalDecoderProfile()
                        .low_llr_1cw_skipped_control_first
                        .fetch_add(1, std::memory_order_relaxed);
                    // Record skipped calls under fail for threshold telemetry.
                    ultra::timing::globalDecoderProfile()
                        .llr_dist_control_first.record(llr_avg, false);
                } else {
                    ultra::timing::ScopedTimer _profile_(
                        ultra::timing::globalDecoderProfile().control_first_1cw);
                    // Reduced retry budget (2 instead of 4) for ACK decode path:
                    // ACK loss is recoverable via ARQ, but the 4-retry sweep was
                    // costing ~22ms × 2 extra attempts × hundreds of ACKs per
                    // transfer. Profiling plan v5 + ChatGPT 5.5 review.
                    control_decode = robustDecodeSingleCW(
                        control_soft_bits.data(), CONTROL_LDPC_BLOCK,
                        CodeRate::R1_4, log_prefix_.c_str(),
                        ultra::timing::SingleCWCallSite::ControlFirst,
                        /*max_retries=*/2);
                    ultra::timing::globalDecoderProfile()
                        .llr_dist_control_first.record(llr_avg, control_decode.first);
                }
                auto [ok_r14, data_r14] = control_decode;
                size_t bpc_r14 = v2::getBytesPerCodeword(CodeRate::R1_4);

                if (ok_r14 && data_r14.size() >= 4
                    && data_r14[0] == 0x55 && data_r14[1] == 0x4C) {
                    if (data_r14.size() > bpc_r14) data_r14.resize(bpc_r14);
                    auto hdr = v2::parseHeader(data_r14);
                    if (hdr.valid && hdr.total_cw == 1 && v2::isControlFrame(hdr.type)) {
                        // BUG-CLASSIC-CFO-CERT (2026-08-04): the control-first path
                        // returns before the common post-demod CFO feedback below.
                        // That violated CFOTracker's feedback invariant and, more
                        // importantly, left pilot_seeded=false after a preceding 0/N
                        // group revoked the warm certificate.  A CRC/LDPC-valid
                        // connected OFDM control frame proves both this demodulation
                        // and its pilot-refined CFO.  Feed the residual back and
                        // certify it before BURST_HEADER hands off to the following
                        // DATA anchor (or a synchronous protocol callback can reset
                        // the decoder).
                        const float control_residual_cfo = waveform_->estimatedCFO();
                        const float control_current_cfo = cfo_tracker_.tracked();
                        const auto control_cfo_update = cfo_tracker_.ingestPilotResidual(
                            frame_demodulator_.preCorrectionCfo(), control_residual_cfo,
                            control_current_cfo, /*clamp_drift=*/true);
                        sync_cfo_ = control_cfo_update.accepted_cfo;
                        cfo_tracker_.certifyWarm();
                        LOG_MODEM(INFO,
                                  "[%s] Classic OFDM control CFO certified: %.2f Hz "
                                  "(pre_corr=%.2f residual=%.2f)",
                                  log_prefix_.c_str(), sync_cfo_,
                                  frame_demodulator_.preCorrectionCfo(),
                                  control_residual_cfo);
                        // Self-describing burst (design §14.17): a BURST_HEADER
                        // descriptor configures THIS receiver's group decode from
                        // the SENDER's declaration (group size, CW/frame, interleave
                        // flags), then is consumed internally — not surfaced as a
                        // user frame. This is the fix for the cross-station 0/8: the
                        // receiver stops guessing the structure from local config.
                        if (hdr.type == v2::FrameType::BURST_HEADER) {
                            // DESC-SWITCH Phase 1 (ULTRA_DESCRIPTOR_MODE_SWITCH,
                            // docs/MODE_SWITCH_PIGGYBACK_DESIGN_2026_07_03.md §5.1 step 4):
                            // a descriptor whose mod/rate differs from the current data
                            // mode is a MODE-HOP announcement. Captured here, consumed
                            // below at (a) the warm-handoff decision (demote: the pilot/
                            // carrier geometry changed, stale |H| would be garbage — expect
                            // the full anchor the sender is guaranteed to have sent) and
                            // (b) the decoder→protocol notification, fired after all
                            // decoder-local locks are released.
                            bool descriptor_mode_hop = false;
                            bool descriptor_current_group_full_anchor = false;
                            Modulation descriptor_hop_mod = current_modulation_;
                            CodeRate descriptor_hop_rate = code_rate_;
                            int descriptor_hop_cw = 0;
                            if (auto cf = v2::ControlFrame::deserialize(data_r14)) {
                                const auto bi = cf->getBurstHeaderInfo();
                                descriptor_current_group_full_anchor =
                                    bi.current_group_full_anchor;
                                if (bi.group_size >= 2) {
                                    setBurstGroupSizeFromDescriptor(bi.group_size);
                                }
                                if (bi.cw_per_frame >= 1) {
                                    setFixedFrameCodewords(bi.cw_per_frame);
                                }
                                setBurstInterleave(bi.burst_interleave);
                                setCarrierLdpcInterleaver(bi.carrier_ldpc);
                                // §14.36 Phase 5c: the descriptor's declared rate is
                                // authoritative — when the sender adapts mid-transfer
                                // (chunk-at-rate), the next group's frames are sized
                                // for bi.code_rate, NOT our negotiated start rate. We
                                // must reconfigure the decoder to that rate before
                                // demodulating the group, or it decodes at the wrong
                                // K and fails 0/8 on every adapted resend.
                                if (bi.code_rate != code_rate_ ||
                                    bi.modulation != current_modulation_) {
                                    // §14.36 crash fix: DEFER the configure() to
                                    // the top of the NEXT processBuffer call via
                                    // pending_descriptor_*. Calling configure()
                                    // here (inside a downstream scoped lock that
                                    // doesn't cover the rest of the loop) swapped
                                    // modulator_/demodulator_/chirp_sync_ while
                                    // other paths still held references —
                                    // SIGSEGV in HilbertTransform::process
                                    // (ultra_gui-2026-05-28-000112.ips).
                                    LOG_MODEM(INFO,
                                        "[%s] Burst descriptor rate change pending: %s -> %s",
                                        log_prefix_.c_str(),
                                        codeRateToString(code_rate_),
                                        codeRateToString(bi.code_rate));
                                    pending_descriptor_mod_ = bi.modulation;
                                    pending_descriptor_rate_ = bi.code_rate;
                                    pending_descriptor_rate_change_ = true;
                                    // DESC-SWITCH: same differ-condition = the hop.
                                    descriptor_mode_hop = true;
                                    descriptor_hop_mod = bi.modulation;
                                    descriptor_hop_rate = bi.code_rate;
                                    descriptor_hop_cw =
                                        (bi.cw_per_frame >= 1) ? bi.cw_per_frame : 0;
                                }
                                // Declare the data size so the immediately-following
                                // group-start frame is sized as a full data frame
                                // (PendingCodewords requirement) instead of a
                                // control-sized peek. This stops the group-start from
                                // entering the control-first peek path, whose
                                // control-profile probe double-demodulated the frame
                                // and poisoned the coherent channel estimate
                                // (near-erasure LLRs, 0/8 deinterleave — §14.24 bug B).
                                // The negated-LTS marker still triggers accumulation;
                                // this only fixes the buffer sizing + decode profile.
                                if (bi.cw_per_frame >= 1) {
                                    pending_total_cw_ = bi.cw_per_frame;
                                }
                                sync_controller_.last_burst_descriptor_ = bi;
                                sync_controller_.have_burst_descriptor_ = true;
                                // BUG-BURST-STALE-GEOMETRY: LEARN this rung's
                                // (cw/frame, z) from wire truth. cw is not
                                // rate-derivable and the sender's coherence walk is
                                // not reproducible here, so the only honest source is
                                // a descriptor we actually decoded. Used when a LATER
                                // descriptor for this same rung is missed.
                                learnRungGeometry(
                                    bi.modulation, bi.code_rate,
                                    bi.cw_per_frame, bi.lifting_z,
                                    geometry_session_epoch_at_start);
                                // ...and RECORD WHETHER THE SENDER TOOK OUR COMMAND.
                                // This descriptor is the sender stating the rung it is
                                // actually transmitting; if it is not the rung we
                                // commanded, the command was declined and must not be
                                // used to slice a later missed-descriptor group.
                                noteDescriptorRungObserved(bi.modulation, bi.code_rate);
                                // Backstop-NACK window (2026-07-04): HEADNULL
                                // drops count PER descriptor-declared group.
                                headnull_resync_drop_count_ = 0;
                                // #69 anchor-skip: stash the sender's announcement of the NEXT
                                // group's anchor type so noteGroupDelivered arms the right search
                                // (full-search chirp groups, light-search announced-skip groups).
                                sync_controller_.setNextGroupLightAnchor(bi.next_light_anchor);
                                // 2026-05-28 Phase 3: propagate the announced
                                // lifting_z to the waveform so
                                // getMinSamplesForCWCount returns the right
                                // airtime for z=81 (n=1944) data frames.
                                if (waveform_) {
                                    waveform_->setActiveLDPCLiftingZ(bi.lifting_z);
                                }
                                // §14.27: the descriptor frame header seq carries
                                // the burst group_seq (encoder stamps it via
                                // setBurstGroupSeq). The following group's
                                // logical frames inherit it for whole-burst ACK.
                                last_burst_group_seq_ = hdr.seq;
                                // HARQ provisional keys: reset per-group state at
                                // descriptor consume. Gate D2 (timeout-batch resend,
                                // whose fill rule differs from the steady-state
                                // mirror) keys on the SAMPLE-CLOCK inter-descriptor
                                // gap — the observable faithful to the mechanism:
                                // steady-state groups arrive every burst+turnaround
                                // (<= ~11.5 s worst case: 8.6 s duty-capped burst +
                                // rig turnaround), while a timeout resend can only
                                // arrive after the sender's ACK RTO (>= ~19 s of
                                // silence). 15 s sits between the two populations.
                                // Two prior gates FAILED here: expect_full_ofdm_anchor_
                                // is armed by the routine full-chirp cadence (blocked
                                // every group -> provisional=0), and sync-distress
                                // counters fire exactly during the fades the feature
                                // targets (blocked the target population).
                                {
                                    // ONE definition, shared with the
                                    // commanded-geometry cadence guard
                                    // (BUG-BURST-STALE-GEOMETRY) so the two cannot
                                    // drift: streaming_decode_policy.hpp.
                                    // Do not conflate an announced FULL anchor with
                                    // an RTO gap. Selective SACK repairs deliberately
                                    // use FULL acquisition at steady cadence, and are
                                    // exactly where CW0-loss combining is valuable.
                                    burst_harq_cadence_blocked_ =
                                        decode_policy::burstCadenceBlocksProvisionalHarq(
                                            last_descriptor_abs_sample_, frame_sync_abs);
                                    last_descriptor_abs_sample_ = frame_sync_abs;
                                }
                                last_burst_src_hash_ = hdr.src_hash;
                                burst_harq_ctx_.reset();
                                burst_harq_ctx_pulled_ = false;
                                burst_harq_prediction_invalid_ = false;
                                burst_harq_prediction_validated_ = false;
                                LOG_MODEM(INFO,
                                    "[%s] Burst descriptor RX: group=%u cw/frame=%u z=%u bi=%d cldpc=%d current_anchor=%s",
                                    log_prefix_.c_str(), bi.group_size, bi.cw_per_frame,
                                    static_cast<unsigned>(bi.lifting_z),
                                    bi.burst_interleave ? 1 : 0, bi.carrier_ldpc ? 1 : 0,
                                    bi.current_group_full_anchor ? "FULL" : "LIGHT");
                            }
                            {
                                std::lock_guard<std::mutex> slock(stats_mutex_);
                                stats_.frames_decoded++;
                            }
                            noteFrameArrivalSuccess(frame_sync_abs, frame_sync_abs + frame_len);
                            if (resetDuringDecode()) {
                                return;
                            }
                            // CRITICAL: restore the data waveform profile. Decoding
                            // the descriptor switched the waveform to the 1-CW control
                            // profile (control modulation/pilots). If we return without
                            // restoring, the following group-start DATA frame is sized
                            // and demodulated with the control-profile pilot geometry,
                            // so getMinSamplesForCWCount() is one OFDM symbol short
                            // (e.g. 31104 vs 32256) → the group-start demodulates 27
                            // symbols instead of 28 → deinterleave fails 0/4 on every
                            // logical frame. This mirrors the normal control-decode
                            // path's profile restore.
                            if (switched_profile) {
                                waveform_->configure(saved_mod, saved_rate);
                            }
                            // Advance past the descriptor frame so the search resumes
                            // at the data group that follows — otherwise the LTS
                            // detector re-locks the descriptor anchor and re-decodes it
                            // forever. Reset frame-arrival tracking and expect the
                            // group's anchor fresh, identical to the no-descriptor sync
                            // path (mirrors the FILE_CANCEL control handling).
                            //
                            // 2026-05-28: reliability first — keep sync_controller_.expect_full_ofdm_anchor_
                            // armed so bravo waits for the full chirp+LTS that alpha now
                            // emits at the start of each burst group (see streaming_encoder
                            // group-start preamble change). The light-LTS-only path was
                            // dropping Group 1+ because warm-sync goes DEGRADED across the
                            // BURST_HEADER→data gap and the data sync corr stays <0.52.
                            // Airtime overhead from the per-group full chirp is acceptable
                            // until the warm-sync hand-off across the gap is hardened.
                            //
                            // F147: a consumed BURST_HEADER is SUBSTANTIVE peer-TX
                            // evidence (a decoded descriptor cannot come from noise).
                            stampRxSubstantive();
                            // F165: the standard group/ack path owns this burst now.
                            anchored_burst_backstop_armed_ = false;
                            anchored_burst_payload_seen_ = false;
                            // §16.8 step 1: BURST_HEADER-consume snapshot (instrumentation).
                            // Logs the warm-sync state we held when the next group's
                            // BURST_HEADER arrived (just before we throw it away with
                            // resetFrameArrivalTrackingLocked + sync_controller_.expect_full_ofdm_anchor_).
                            // Pair with the end-of-group snapshot in
                            // streaming_burst_interleave.cpp to compute the gap delta
                            // (Δphase, Δconf, Δcfo, elapsed samples). Multi-group
                            // file transfer trace = the §16.8 instrumentation deliverable.
                            LOG_MODEM(INFO,
                                "[%s] s16-snapshot pre-reset (BURST_HEADER consumed) "
                                "group_seq=%u phase=%s misses=%d conf=%.2f "
                                "last_cfo=%.2f next_expected=%llu last_frame_end=%llu "
                                "expect_full_anchor=%d frame_sync_abs=%llu frame_len=%zu",
                                log_prefix_.c_str(),
                                static_cast<unsigned>(last_burst_group_seq_),
                                arrival_policy::warmSyncPhaseName(sync_controller_.derivePhase()),
                                sync_controller_.consecutiveSyncMisses(),
                                sync_controller_.frameArrivalConfidence(),
                                cfo_tracker_.tracked(),
                                static_cast<unsigned long long>(sync_controller_.next_expected_frame_sample_),
                                static_cast<unsigned long long>(sync_controller_.lastFrameEndSample()),
                                sync_controller_.expect_full_ofdm_anchor_ ? 1 : 0,
                                static_cast<unsigned long long>(frame_sync_abs),
                                frame_len);
                            // §16.8 step 2 (warm-handoff, now always on): if the
                            // BURST_HEADER's noteFrameArrivalSuccess promoted warm-sync
                            // to WARM (which §16.11 confirmed happens every group on
                            // Good@20), SKIP the reset. The warm timing seeded by the
                            // BURST_HEADER (which arrived inside its own full chirp+LTS
                            // anchor) carries into the data group that immediately
                            // follows. Matching alpha-side change in streaming_encoder.cpp
                            // uses light LTS for group-start data instead of a redundant
                            // second full chirp+LTS. Falls back to the legacy reset path
                            // when warm-sync wasn't actually WARM (sync miss path), or when
                            // sync_controller_.expect_full_ofdm_anchor_ was already true going in
                            // (handshake / cold acquisition still pending).
                            // Warm-handoff BURST_HEADER-consume keeper — now unconditional
                            // (promoted past ULTRA_S16_WARM_HANDOFF). Still gated on actually
                            // being WARM with positive confidence (else fall to the reset path).
                            //
                            // DESC-SWITCH §5.1 step 4a (knob-gated, byte-identical OFF): a
                            // MODE-HOP descriptor DEMOTES warm-handoff for the group that
                            // follows — the mod/rate change moves the pilot/carrier geometry,
                            // so the warm |H| estimate is stale-by-construction (the 06-09
                            // unilateral-flip 0/8-forever arm). The reset path below arms
                            // expect_full_ofdm_anchor_, matching the full chirp+LTS the
                            // sender's commit one-shot guarantees on a mode-hop group.
                            // F164 CONTRACT SYMMETRY (reverts the F142 fixed-grid
                            // relaxation): the SENDER now arms a full chirp+LTS anchor
                            // on EVERY mode commit (F163: missed light switch
                            // descriptors cost 25 s each), so a mode-hop descriptor
                            // means a FULL anchor is coming — the receiver MUST demote
                            // warm-handoff and expect it. F164 proved the asymmetry
                            // fatal the other way: receiver kept warm-light expectation,
                            // the unexpected 1.2 s chirp shifted the group start, and
                            // warm-predicted frame positions decoded garbage — 13
                            // craters, 20-minute timeout. Same-mode headers (no hop)
                            // still ride the warm path.
                            const bool warm_handoff_eligible =
                                decode_policy::keepDescriptorWarmHandoff(
                                    sync_controller_.derivePhase() ==
                                        arrival_policy::WarmSyncPhase::WARM,
                                    sync_controller_.frameArrivalConfidence(),
                                    descriptor_current_group_full_anchor,
                                    descriptor_mode_switch_enabled_, descriptor_mode_hop);
                            {
                                std::lock_guard<std::mutex> lock(sync_controller_.ring_.buffer_mutex_);
                                if (warm_handoff_eligible) {
                                    // KEEP warm state. Just advance the search
                                    // window past the BURST_HEADER so the data
                                    // group is picked up next.
                                    sync_controller_.expect_full_ofdm_anchor_ = false;
                                    sync_controller_.clearRejectStreak();
                                    sync_controller_.ring_.correlation_pos_ = sync_controller_.ring_.wrapRingIndexLocked(
                                        sync_position_ + frame_len);
                                    sync_controller_.ring_.setSearchFloorLocked(frame_sync_abs + frame_len);
                                    last_decoded_sync_pos_ = sync_position_;
                                } else {
                                    sync_from_warm_timed_window_ = false;
                                    resetFrameArrivalTrackingLocked();
                                    sync_controller_.expect_full_ofdm_anchor_ = true;
                                    sync_controller_.clearRejectStreak();
                                    sync_controller_.ring_.correlation_pos_ = sync_controller_.ring_.wrapRingIndexLocked(
                                        sync_position_ + frame_len);
                                    sync_controller_.ring_.setSearchFloorLocked(frame_sync_abs + frame_len);
                                    last_decoded_sync_pos_ = sync_position_;
                                }
                            }
                            if (warm_handoff_eligible) {
                                LOG_MODEM(INFO,
                                    "[%s] s16-warm-handoff: KEPT warm state across "
                                    "BURST_HEADER consume (phase=%s conf=%.2f cfo=%.2f); "
                                    "expecting light LTS for data group",
                                    log_prefix_.c_str(),
                                    arrival_policy::warmSyncPhaseName(sync_controller_.derivePhase()),
                                    sync_controller_.frameArrivalConfidence(),
                                    cfo_tracker_.tracked());
                            } else if (descriptor_current_group_full_anchor) {
                                LOG_MODEM(INFO,
                                    "[%s] BURST_HEADER announces full chirp+LTS for "
                                    "current DATA group; arming full-anchor search",
                                    log_prefix_.c_str());
                            }
                            // §16.8 step 1: post-reset snapshot. What did we throw
                            // away?
                            LOG_MODEM(INFO,
                                "[%s] s16-snapshot post-reset phase=%s misses=%d "
                                "conf=%.2f last_cfo=%.2f expect_full_anchor=%d",
                                log_prefix_.c_str(),
                                arrival_policy::warmSyncPhaseName(sync_controller_.derivePhase()),
                                sync_controller_.consecutiveSyncMisses(),
                                sync_controller_.frameArrivalConfidence(),
                                cfo_tracker_.tracked(),
                                sync_controller_.expect_full_ofdm_anchor_ ? 1 : 0);
                            // DESC-SWITCH §5.1 step 4b (knob-gated): a mode-hop descriptor
                            // drives the PROTOCOL layer too (RX applyDataMode: window/
                            // timers/chunk capacity follow the announced mode). Fired here
                            // — after every decoder-local lock scope above has closed —
                            // into the same decoder-thread → ProtocolEngine-mutex path
                            // burst_group_callback_ uses (§6 row 11: no new concurrency
                            // class; the demod config itself applies at the top of the
                            // next processBuffer via the pending_descriptor_* channel).
                            if (descriptor_mode_switch_enabled_ && descriptor_mode_hop &&
                                descriptor_mode_change_callback_) {
                                descriptor_mode_change_callback_(
                                    descriptor_hop_mod, descriptor_hop_rate,
                                    descriptor_hop_cw);
                            }
                            state_ = DecoderState::SEARCHING;
                            return;  // consumed; the data group follows next
                        }
                        DecodeResult control_result;
                        control_result.success = true;
                        control_result.frame_data = data_r14;
                        control_result.frame_type = hdr.type;
                        control_result.snr_db = sync_snr_;
                        control_result.cfo_hz = sync_cfo_;
                        control_result.codewords_ok = 1;
                        control_result.codewords_failed = 0;
                        populateDecodeMetrics(control_result, is_ofdm,
                                              waveform_ ? waveform_->estimatedCFO() : sync_cfo_);

                        // A control frame can arrive while the receiver is waiting
                        // for the peer's full-anchored DATA burst. It is a complete
                        // standalone transmission, so do not expose the conservative
                        // DATA-burst gate to its synchronous protocol callback.
                        clearProvisionalBurstAirGate();
                        control_result.physical_turn_complete = true;
                        {
                            std::lock_guard<std::mutex> qlock(queue_mutex_);
                            frame_queue_.push(control_result);
                        }
                        if (frame_callback_) {
                            frame_callback_(control_result);
                        }
                        if (resetDuringDecode()) {
                            return;
                        }
                        {
                            std::lock_guard<std::mutex> slock(stats_mutex_);
                            stats_.frames_decoded++;
                        }

                        // Connected ACK/NACK/MODE_CHANGE frames use the short
                        // hardened control profile; keep the adaptation meter
                        // tied to data-channel frames after the link is up.
                        if (!connected_) {
                            last_fading_index_.store(waveform_->getFadingIndex());
                        }

                        if (switched_profile) {
                            waveform_->configure(saved_mod, saved_rate);
                        }

                        const bool file_cancel_control =
                            hdr.type == v2::FrameType::FILE_CANCEL;
                        if (!file_cancel_control) {
                            noteFrameArrivalSuccess(frame_sync_abs, frame_sync_abs + frame_len);
                        }
                        {
                            std::lock_guard<std::mutex> lock(sync_controller_.ring_.buffer_mutex_);
                            sync_from_warm_timed_window_ = false;
                            if (file_cancel_control) {
                                resetFrameArrivalTrackingLocked();
                                sync_controller_.expect_full_ofdm_anchor_ = true;
                                sync_controller_.clearRejectStreak();
                            } else {
                                // A pending connected full-anchor request is for
                                // the next DATA burst after a turn/control
                                // boundary. ACK/NACK/control repeats may arrive
                                // first; decoding them must not consume that
                                // DATA re-anchor latch.
                                if (sync_controller_.expect_full_ofdm_anchor_) {
                                    LOG_MODEM(INFO,
                                              "[%s] OFDM control %s decoded; preserving pending full DATA anchor",
                                              log_prefix_.c_str(),
                                              v2::frameTypeToString(hdr.type));
                                }
                            }
                            sync_controller_.ring_.correlation_pos_ = sync_controller_.ring_.wrapRingIndexLocked(sync_position_ + frame_len);
                            sync_controller_.ring_.setSearchFloorLocked(frame_sync_abs + frame_len);
                            last_decoded_sync_pos_ = sync_position_;
                        }

                        LOG_MODEM(INFO, "[%s] OFDM control-profile decode SUCCESS (%s seq=%d)",
                                  log_prefix_.c_str(), v2::frameTypeToString(hdr.type), hdr.seq);
                        if (file_cancel_control) {
                            LOG_MODEM(INFO,
                                      "[%s] FILE_CANCEL decoded; forcing full OFDM anchor for next frame",
                                      log_prefix_.c_str());
                        }
                        state_ = DecoderState::SEARCHING;
                        return;
                    }
                }
            }
        }

        const bool control_probe_saw_burst_marker =
            waveform_->wasBurstInterleaved();
        if (switched_profile) {
            waveform_->configure(saved_mod, saved_rate);
        }

        // DISCONNECTING shares the connected OFDM waveform only so the hardened
        // QPSK R1/4 ACK/control plane remains decodable.  Once that exact control
        // decode has failed, the candidate is not legal teardown traffic.  Never
        // fall through to the generic connected DATA path or arm fixed-CW
        // escalation; advance just as for another false OFDM lock and keep looking
        // for the real ACK.
        if (control_only_receive) {
            LOG_MODEM(INFO,
                      "[%s] Control-only RX: invalid control candidate rejected; "
                      "DATA fallback suppressed",
                      log_prefix_.c_str());
            pending_total_cw_ = 0;
            advancePastFalseOFDMLock();
            state_ = DecoderState::SEARCHING;
            return;
        }

        // The descriptor and marked head can be lost in the accepted full-anchor
        // fade while a later unmarked BI0 member survives. Only after the robust
        // control hypothesis has failed may the learned long-DATA geometry take
        // over. Re-enter SYNC_FOUND at the same training position so readiness and
        // decode both wait for the full Z81 member; do not synthesize descriptor/N.
        const bool candidate_burst_latched =
            burst_latched || control_probe_saw_burst_marker;
        const auto classic_long = resolveClassicLongGeometry(
            is_ofdm, candidate_burst_latched,
            static_cast<uint64_t>(frame_sync_abs));
        if (classic_long.armed) {
            armClassicLongGeometry(classic_long);
            state_ = DecoderState::SYNC_FOUND;
            return;
        }

        // The control-first peek ran and found no valid control frame. INSIDE an
        // active burst group ALL data is decoded at the descriptor-declared size
        // (pending_total_cw_, set on BURST_HEADER), so it never reaches this
        // control-sized peek — a control-sized frame here that is not a valid
        // control frame is noise / a false sync, not a data frame to
        // speculatively decode. Re-search instead of the legacy fall-through to
        // a multi-CW data decode (whose control-profile probe + double-demod
        // poisoned the *burst's shared* coherent channel estimate — §14.24/§14.25).
        //
        // 2026-06-03 (BUG-TNC-B2F-001): gate on have_burst_descriptor_, NOT merely
        // burst_transport_rx_. burst_transport_rx_ became the unconditional default
        // (ULTRA_BURST_TRANSPORT gate removed 2026-06-02), which made EVERY connected
        // station treat its non-burst interactive DATA (sendBinary → SR-ARQ z=27 short
        // LDPC, the chat/B2F path) as burst-regime noise: a non-burst multi-CW DATA
        // frame carries no descriptor → pending_total_cw_=0 → it enters the control
        // peek, fails it (it is DATA, not control), and was re-searched away here →
        // the receiver synced cleanly (corr=1.0) yet delivered NOTHING. The poisoning
        // hazard is specifically a burst's SHARED estimate; a standalone non-burst
        // frame has no group to poison, so it must fall through to the data decode.
        // Only re-search when we are genuinely mid-burst (a descriptor is active).
        if ((use_burst_interleave_ || burst_transport_rx_) && connected_ &&
            sync_controller_.have_burst_descriptor_) {
            // LATE-JOIN recovery (ULTRA_DESC_ARMED_ACCUM, default OFF;
            // docs/DESC_ARMED_ACCUMULATION_DESIGN_2026_07_05.md): instead of
            // dropping this clean group-member, arm accumulation AT it using the
            // latched descriptor geometry; finalize tail-anchors with head
            // erasures. Falls through to the legacy drop when the member's full
            // data frame is not yet in the ring (the next member retries).
            static const bool kDescArmedAccum = [] {
                const char* e = std::getenv("ULTRA_DESC_ARMED_ACCUM");
                return !(e && e[0] == '0');  // DEFAULT-ON 2026-07-05
            }();
            if (kDescArmedAccum && lateJoinBurstAccumulation(frame_sync_abs)) {
                return;  // armed at this member; the slicer owns the rest of the group
            }
            // BUG-BURST-HEADNULL-DROP observability (2026-07-01): when the group
            // HEAD (marker frame) is nulled, accumulation never arms and every
            // clean mid-group frame lands here and is re-searched away with no
            // decode attempt and no ACK credit — previously with NO log at all
            // (a 27 dB frame vanished silently; one such saga cost 83 s). The
            // re-search itself stays (the §14.24 estimate-poisoning guard is
            // load-bearing); this counter makes the drop measurable. Recovery
            // (enter accumulation from any group-member sync with the head
            // erasure-marked) is LATE-JOIN above (knob-gated).
            ++headnull_resync_drop_count_;
            LOG_MODEM(INFO,
                      "[%s] [HEADNULL] sync-accepted frame consumed without decode "
                      "attempt (group not armed; drop #%u)",
                      log_prefix_.c_str(), headnull_resync_drop_count_);
            // GROUP-ABANDONMENT BACKSTOP NACK (2026-07-04, 16QAM zero-decode
            // forensics fix #5): when the drops exhaust the descriptor-declared
            // group (every member frame consumed unarmed), the sender would
            // otherwise hear NOTHING — no finalize, no ACK — and eat a full ~18 s
            // burst-RTO. Synthesize an EMPTY finalize (mask 0) so the normal
            // group-ack path emits a fast NACK (~one group period instead of the
            // RTO; this deliberately restores the fast-NACK latency the pre-fix
            // premature finalize provided by accident, without its geometry
            // corruption). group size >= 2 guaranteed by have_burst_descriptor_.
            {
                const int declared = std::max(2, burst_group_size_);
                if (static_cast<int>(headnull_resync_drop_count_) >= declared &&
                    burst_soft_buffer_.empty() && burst_group_callback_) {
                    LOG_MODEM(WARN,
                              "[%s] [HEADNULL] declared group (%d) exhausted unarmed — "
                              "synthesizing empty finalize (backstop NACK)",
                              log_prefix_.c_str(), declared);
                    headnull_resync_drop_count_ = 0;
                    clearActiveBurstDescriptorGeometry();
                    finalizeGroupEvmSnr(/*geometry_proven=*/true, declared);
                    burst_group_callback_(/*group_seq=*/0, {}, /*all_ok=*/false,
                                          /*quality=*/0.0f, /*frame_mask=*/0,
                                          use_burst_interleave_,
                                          static_cast<uint8_t>(declared),
                                          /*geometry_proven=*/true);
                }
            }
            {
                std::lock_guard<std::mutex> lock(sync_controller_.ring_.buffer_mutex_);
                sync_controller_.ring_.correlation_pos_ = sync_controller_.ring_.wrapRingIndexLocked(sync_position_ + frame_len);
                sync_controller_.ring_.setSearchFloorLocked(frame_sync_abs + frame_len);
            }
            state_ = DecoderState::SEARCHING;
            return;
        }
    }

    // Data frame - process audio to get soft bits
    waveform_->setFrequencyOffset(decode_cfo);

    auto decode_start = std::chrono::steady_clock::now();
    const int expected_codewords = expectedOFDMCodewordsForSamples(frame_buffer.size());
    ultra::genie::eqTrace().site = "data-main";
    bool ok = processWaveformForCodewords(
        SampleSpan(frame_buffer.data(), frame_buffer.size()), expected_codewords);

    if (!ok) {
        LOG_MODEM(DEBUG, "[%s] process() failed", log_prefix_.c_str());
        {
            std::lock_guard<std::mutex> lock(sync_controller_.ring_.buffer_mutex_);
            sync_controller_.ring_.correlation_pos_ = sync_controller_.ring_.wrapRingIndexLocked(sync_position_ + frame_len);
            sync_controller_.ring_.setSearchFloorLocked(frame_sync_abs + frame_len);
        }
        state_ = DecoderState::SEARCHING;
        return;
    }

    // If the LTS detector locked early on a marked burst group, the first
    // physical block is the one most likely to poison the deinterleaver. Retry
    // that block from the timing-slope-corrected origin before collecting LLRs.
    // 2026-05-29 channel-adaptive interleaver (RX decouple): the negated-LTS marker
    // signals a burst GROUP-START, which the encoder emits for EVERY group regardless
    // of whether the group's bytes are byte-interleaved (TX decouple, d5d8eaa). So the
    // group-accumulation path must trigger on the marker ALONE within the burst-transport
    // regime — NOT on use_burst_interleave_. The interleave flag now gates ONLY the byte
    // de-permutation in finalizeBurstGroup; with it OFF each physical frame is one logical
    // frame, so per-frame survival is meaningful (the SR-ARQ-on-Good path). The regime
    // guard (use_burst_interleave_ || burst_transport_rx_) preserves the exact pre-decouple
    // behavior when interleave is on, and only extends accumulation to interleave-off
    // groups inside an actual burst file transfer (never a stray non-file frame).
    // A classic-long candidate was explicitly accepted only after the short
    // control probe saw no marker. Do not let a second demodulation's noisy sign
    // decision promote it into group accumulation: this recovery knows cw/Z but
    // deliberately knows no N. It must remain an independently decoded frame.
    bool burst_marker = !classic_long_on_entry && connected_ && is_ofdm
                        && mode_ == protocol::WaveformMode::OFDM_CHIRP
                        && waveform_->wasBurstInterleaved()
                        && (use_burst_interleave_ || burst_transport_rx_);
    if (ultra::phyDiagnosticsEnabled() && connected_ && is_ofdm
        && mode_ == protocol::WaveformMode::OFDM_CHIRP) {
        std::ostringstream oss;
        oss << "burst_rx_check use_bi=" << (use_burst_interleave_ ? 1 : 0)
            << " was_bi=" << (waveform_->wasBurstInterleaved() ? 1 : 0)
            << " marker=" << (burst_marker ? 1 : 0);
        ultra::phyDiagLine(oss.str());
    }
    if (burst_marker) {
        const float burst_timing_offset = waveform_->getLastTimingOffsetSamples();
        constexpr float kBurstFrameRetryThreshold = 64.0f;
        constexpr float kBurstFrameRetryMax = 320.0f;
        if (std::abs(burst_timing_offset) >= kBurstFrameRetryThreshold &&
            std::abs(burst_timing_offset) <= kBurstFrameRetryMax) {
            const int sample_correction = static_cast<int>(std::lround(burst_timing_offset));
            const size_t corrected_sync_pos = sync_controller_.ring_.wrapRingIndexLocked(
                sync_position_ + sync_controller_.ring_.buffer_capacity_samples_ + sample_correction);
            const size_t corrected_sync_abs =
                (sample_correction >= 0)
                    ? frame_sync_abs + static_cast<size_t>(sample_correction)
                    : (frame_sync_abs > static_cast<size_t>(-sample_correction)
                           ? frame_sync_abs - static_cast<size_t>(-sample_correction)
                           : 0);

            bool have_corrected_frame = false;
            {
                std::lock_guard<std::mutex> lock(sync_controller_.ring_.buffer_mutex_);
                size_t corrected_available;
                if (sync_controller_.ring_.write_pos_ >= corrected_sync_pos) {
                    corrected_available = sync_controller_.ring_.write_pos_ - corrected_sync_pos;
                } else {
                    corrected_available = sync_controller_.ring_.buffer_capacity_samples_ - corrected_sync_pos + sync_controller_.ring_.write_pos_;
                }
                have_corrected_frame = corrected_available >= frame_len;
                if (have_corrected_frame) {
                    frame_buffer.assign(frame_len, 0.0f);
                    for (size_t i = 0; i < frame_len; i++) {
                        frame_buffer[i] = sync_controller_.ring_.buffer_[sync_controller_.ring_.wrapRingIndexLocked(corrected_sync_pos + i)];
                    }
                }
            }

            if (have_corrected_frame) {
                frame_demodulator_.applyCFOPreCorrection(frame_buffer, sync_cfo_, corrected_sync_abs, log_prefix_.c_str());

                // The marker flag was consumed by the first process() call.
                // Normalize the first LTS symbol manually for this retry so
                // channel estimation sees two same-polarity training symbols.
                const size_t lts_sym_len = static_cast<size_t>(waveform_->getSamplesPerSymbol());
                for (size_t i = 0; i < lts_sym_len && i < frame_buffer.size(); ++i) {
                    frame_buffer[i] = -frame_buffer[i];
                }

                waveform_->setAbsoluteTrainingPosition(corrected_sync_abs);
                waveform_->setFrequencyOffset(decode_cfo);
                ultra::genie::eqTrace().site = "marker-retry";
                bool retry_ok = processWaveformForCodewords(
                    SampleSpan(frame_buffer.data(), frame_buffer.size()), expected_codewords);
                if (retry_ok) {
                    sync_position_ = corrected_sync_pos;
                    frame_sync_abs = corrected_sync_abs;
                    LOG_MODEM(WARN, "[%s] Burst marker frame timing retry: %.1f samples, sync_pos=%zu",
                              log_prefix_.c_str(), burst_timing_offset, sync_position_);
                } else {
                    LOG_MODEM(WARN, "[%s] Burst marker frame timing retry failed: %.1f samples",
                              log_prefix_.c_str(), burst_timing_offset);
                }
            } else {
                LOG_MODEM(INFO, "[%s] Burst marker frame timing retry deferred: %.1f samples, need %zu",
                          log_prefix_.c_str(), burst_timing_offset, frame_len);
            }
        }
    }

    float lts_signal_power = 1.0f;
    float lts_channel_mag = 1.0f;
    if (hasInvalidOFDMTraining(waveform_.get(), is_ofdm, connected_,
                               lts_signal_power, lts_channel_mag)) {
        LOG_MODEM(INFO, "[%s] False chirp lock rejected: "
                  "lts_signal=%.6f, |H|_avg=%.4f — re-searching",
                  log_prefix_.c_str(), lts_signal_power, lts_channel_mag);
        advancePastFalseOFDMLock();
        state_ = DecoderState::SEARCHING;
        return;
    }

    captureConstellationSnapshot();

    auto soft_bits = waveform_->getSoftBits();
    if (soft_bits.empty()) {
        LOG_MODEM(DEBUG, "[%s] getSoftBits() returned empty", log_prefix_.c_str());
        {
            std::lock_guard<std::mutex> lock(sync_controller_.ring_.buffer_mutex_);
            sync_controller_.ring_.correlation_pos_ = sync_controller_.ring_.wrapRingIndexLocked(sync_position_ + frame_len);
            sync_controller_.ring_.setSearchFloorLocked(frame_sync_abs + frame_len);
        }
        state_ = DecoderState::SEARCHING;
        return;
    }

    // Reject false chirp locks before paying for full LDPC decode. Low average
    // confidence is a reliable false-lock signature. A near-zero population is
    // only decisive when it dominates the CW; valid short/tail DATA frames can
    // show a modest near-zero tail after a strong LTS and still decode cleanly.
    {
        const auto llr_quality = signal_policy::evaluatePreSyncLLR(
            soft_bits.data(), soft_bits.size(), v2::LDPC_CODEWORD_BITS);
        if (llr_quality.reject_as_false_lock) {
            // RATIOMETRIC guard (same principle as the 06-14 CONNECT-as-PING fix at
            // the wait gate ~line 1290): a weak 1-CW peek is only PING/false-lock
            // evidence when the frame is ratiometrically SILENT. For MC-DPSK, a weak
            // 1-CW peek on a DATA-BEARING frame (data_rms ~ training_rms, ratio not
            // silent) is almost always a multi-CW CONNECT/CONNECT_ACK we simply
            // cannot decode from a single CW yet — NOT a bare-chirp PING. Emitting a
            // PING here (tryEmitPingByChirpLock runs with ldpc_attempted=false, whose
            // ping_by_chirp_lock short-circuits to true) fires BEFORE the ratiometric
            // 4-CW wait gate, so on a quieter analog chain (real soundcards/IONOS) the
            // initiator PONGed the Pi5's low-level CONNECT_ACK instead of decoding it.
            // Let a data-bearing MC-DPSK frame fall through to the wait gate (full 4-CW
            // decode); a genuine silent PING still pings, and OFDM keeps the original
            // false-lock rejection.
            bool data_bearing_mcdpsk = false;
            if (!is_ofdm && allow_ping_detection) {
                const auto reject_ping = evaluatePingDecision(false, false, false);
                data_bearing_mcdpsk = !reject_ping.ping_by_silence;
                // #70: when the data region only LOOKS data-bearing because low-SNR
                // noise floods a bare PING's silent gap, a solid chirp signature is the
                // trustworthy discriminator (it carries the chirp's processing gain).
                // Emit the PING on chirp-lock alone instead of waiting for a CONNECT.
                // #70 HIGH-SNR-SAFE GATE (2026-07-01): the data/train RATIO cannot separate a
                // flooded PING from a strong false-sync-on-data (BOTH read ratio ~1.0 —
                // measured good@20 spurious 1.007-1.014 vs rig low-SNR 0.52-1.11). What DOES
                // separate them is the ABSOLUTE data-region power: a noise-flooded PING is
                // quiet (rig data_rms 0.04-0.09) while a false lock on a real high-SNR data
                // frame is loud (good@20 data_rms 0.28-0.33). So require the data region to be
                // quiet enough to be a PING — kPingChirpLockMaxDataRMS (0.16), its documented
                // purpose. This is what let default-ON regress the good@20 handshake (spurious
                // emits -> PING/PONG churn). NOTE: an absolute level is fragile per #74's
                // lesson (RX level varies on a real radio); a noise-floor-RELATIVE gate is the
                // level-invariant refinement, but the observed margins here are 2-8x and
                // bare_chirp_expected_ (STAGE2) backstops the residual low-SNR ambiguity.
                // #70 STAGE-1.5 (F222, sim good@10): the NOISE-RELATIVE gate
                // the 2026-07-01 note called for. A bare PING's gap IS the
                // ambient noise (gap_rms/noise_rms ≈ 1) at EVERY level and SNR;
                // a real CONNECT's 4-CW data rides ≥ ~3x above the floor even
                // at 10 dB, and the good@20 spurious-lock class reads ~10x —
                // both rejected by the same test. Level-invariant (#74 lesson:
                // absolute thresholds break on real radios), high-SNR-safe by
                // construction, and it replaces the STAGE2 bare_chirp_expected_
                // gate that was never implemented (the knob was inert — sim
                // good@10: responder heard corr 0.754 PINGs and never PONGed;
                // initiator gave up after 5).
                if (data_bearing_mcdpsk && robust_idle_ping &&
                    reject_ping.gap_is_noise &&
                    reject_ping.chirp_corr >= frame_policy::kPingCorrFloor &&
                    std::abs(reject_ping.gap_error_samples) <=
                        frame_policy::kPingMaxGapError) {
                    LOG_MODEM(INFO, "[%s] robust idle PING by chirp-lock "
                              "(ratio=%.2f corr=%.2f gap=%.1f inband_rms=%.4f "
                              "noise_rms=%.4f) — gap is NOISE, not data",
                              log_prefix_.c_str(), reject_ping.ratio,
                              reject_ping.chirp_corr, reject_ping.gap_error_samples,
                              reject_ping.data_inband_rms,
                              reject_ping.idle_noise_rms);
                    emitPingFrame(reject_ping, false);
                    return;
                }
                if (data_bearing_mcdpsk) {
                    LOG_MODEM(INFO, "[%s] MC-DPSK weak 1-CW LLRs but data-bearing "
                              "(ratio=%.2f >= silence) — not a PING; falling through to "
                              "fixed-frame wait", log_prefix_.c_str(), reject_ping.ratio);
                }
            }
            // F147 EXPECTED-ANCHOR IMMUNITY: this gate exists to kill false
            // chirp locks during IDLE search. A strong chirp match found by the
            // full-anchor search WE ARMED (sender resend expected at exactly
            // this boundary) is a different prior — mush LLRs there mean a
            // faded or cold-CFO group, and bouncing to re-search forfeits the
            // whole group AND its ack (F147: corr=0.76 genuine anchors rejected
            // twice; 2 bursts + 40 s + the ack channel lost). HOLD the lock and
            // let the frames fail LDPC individually — the group/erasure/ARQ
            // machinery owns that failure mode and the group ack still emits.
            // 0.60 sits well above the chirp accept floor; noise does not
            // produce 0.6+ chirp correlations inside an armed window.
            constexpr float kExpectedAnchorHoldCorr = 0.60f;
            // BUG-BURST-STALE-GEOMETRY: a commanded-geometry arm is the SAME prior.
            // Frame 1 is correctly SLICED but is necessarily demapped with the OLD
            // constellation (the profile change cannot be applied inline), so mush
            // LLRs are EXPECTED here — that frame is erased by design at the marker
            // block. Bouncing to re-search would forfeit the whole group AND its ack,
            // which is exactly the failure this fix exists to remove.
            // THE 0.60 FLOOR APPLIES HERE TOO (2026-07-28 review fix). The burst latch
            // this arm rides on is a SIGN TEST on the marker metric — a coin flip on
            // noise — so exempting it from the correlation floor as well would let a
            // noise lock arm a burst group and queue a decoder reconfigure. The floor
            // is what makes the exemption a PRIOR rather than a hole; the measured
            // failure's own DATA-sync accept was corr=0.95, far above it.
            const bool hold_expected_anchor =
                is_ofdm && connected_ &&
                sync_correlation_ >= kExpectedAnchorHoldCorr &&
                (cg.armed || last_sync_expected_full_anchor_);
            if (hold_expected_anchor) {
                LOG_MODEM(WARN,
                          "[%s] Weak LLRs on EXPECTED full anchor (corr=%.2f, "
                          "|llr|_avg=%.2f, near_zero=%.1f%%) — holding lock "
                          "(faded group, not a false lock)",
                          log_prefix_.c_str(), sync_correlation_,
                          llr_quality.mean_abs,
                          llr_quality.near_zero_fraction * 100.0f);
            } else if (!data_bearing_mcdpsk) {
                if (allow_ping_detection &&
                    tryEmitPingByChirpLock("pre_ldpc_llr_reject", false)) {
                    return;
                }
                LOG_MODEM(INFO, "[%s] False chirp lock rejected: |llr|_avg=%.2f, "
                          "near_zero=%zu/%zu (%.1f%%), soft_bits=%zu — re-searching",
                          log_prefix_.c_str(), llr_quality.mean_abs,
                          llr_quality.near_zero_count, llr_quality.count,
                          llr_quality.near_zero_fraction * 100.0f, soft_bits.size());
                advancePastFalseOFDMLock();
                state_ = DecoderState::SEARCHING;
                return;
            }
            // data_bearing_mcdpsk: fall through to the CW0 peek / 4-CW wait gate below.
        }
    }

    LOG_MODEM(INFO, "[%s] Got %zu soft bits (%zu samples), proceeding to decode",
              log_prefix_.c_str(), soft_bits.size(), frame_buffer.size());

    if (burst_marker) {
        LOG_MODEM(INFO, "[%s] Burst interleave marker detected, entering accumulation",
                  log_prefix_.c_str());

        // The descriptor set pending_total_cw_ to size THIS group-start frame as a
        // full data frame (bypassing the control peek). Clear it now: the rest of
        // the group is sliced by burst_min_block_ in tryDemodulateNextBurstFrame,
        // and a stale pending count must not leak onto the next post-burst frame.
        pending_total_cw_ = 0;

        // Initialize accumulation state with first frame's soft bits
        burst_soft_buffer_.clear();
        burst_predecoded_.clear();
        descriptor_group_size_locked_ = false;  // group ended/aborted — cfg writes may apply again
        burst_metric_templates_.clear();
        const bool descriptor_provenance = sync_controller_.have_burst_descriptor_;
        burst_arm_provenance_ = cg.armed
            ? BurstArmProvenance::COMMANDED_GEOMETRY
            : (descriptor_provenance
                   ? BurstArmProvenance::DESCRIPTOR
                   : BurstArmProvenance::UNPROVEN_MARKER);
        burst_substantive_members_ = cg.armed ? 0u : 1u;
        burst_unproven_start_abs_ =
            burst_arm_provenance_ == BurstArmProvenance::UNPROVEN_MARKER
                ? static_cast<uint64_t>(frame_sync_abs)
                : 0u;
        if (cg.armed) {
            // ================================================================
            // BUG-BURST-STALE-GEOMETRY defect (B): this group's BURST_HEADER was
            // MISSED. Arm from the rung THIS receiver COMMANDED, as ONE tuple.
            // ================================================================
            // Applying it PARTIALLY (new cw with the old profile, which is what the
            // wire path does today: setFixedFrameCodewords is immediate while mod/
            // rate are deferred) reproduces this very bug from the other side, so
            // every consumer below is fed from the same (mod, rate, cw, z).
            //
            // (1) SCALARS FIRST — no live-object swap here, so this cannot SIGSEGV.
            const int latched_cw = fixed_frame_codewords_;
            setFixedFrameCodewords(cg.cw);
            waveform_->setActiveLDPCLiftingZ(static_cast<uint8_t>(cg.lifting_z));
            // (2) Synthesize the descriptor so activeBurstLiftingZ() and finalize's
            //     bytes_per_cw_rx agree with the stride we are about to publish.
            //     group_size / burst_interleave / carrier_ldpc / next_light_anchor
            //     are NOT rung-derivable — carry the latched values forward.
            {
                auto synth = sync_controller_.last_burst_descriptor_;
                synth.group_size = static_cast<uint8_t>(std::max(2, burst_group_size_));
                synth.cw_per_frame = static_cast<uint8_t>(cg.cw);
                synth.modulation = cg.mod;
                synth.code_rate = cg.rate;
                synth.burst_interleave = use_burst_interleave_;
                synth.carrier_ldpc = use_carrier_ldpc_interleaver_;
                synth.lifting_z = static_cast<uint8_t>(cg.lifting_z);
                sync_controller_.last_burst_descriptor_ = synth;
                sync_controller_.have_burst_descriptor_ = true;
            }
            headnull_resync_drop_count_ = 0;
            // (2b) GROUP-SEQ INFERENCE — required for correctness, not cosmetics
            //      (2026-07-28 review fix). last_burst_group_seq_ is only written at
            //      descriptor consume, so a missed descriptor leaves the PREVIOUS
            //      group's seq latched. Pre-fix that was harmless because the group
            //      always finalized 0/N and the ack carried no positive credit; this
            //      arm makes frames 2..N decode, so the tone-ack would report a
            //      NON-ZERO frame_mask keyed to the previous group — the sender would
            //      credit frames it never delivered. A new group is +1. Identical
            //      inference, and identical justification, to the late-join head-
            //      missing path (streaming_burst_interleave.cpp): a wrong guess costs
            //      only ack dedup/crater matching for one event, because the decoded
            //      frames carry their own seq headers and those drive the real SACK.
            last_burst_group_seq_ =
                static_cast<uint8_t>((last_burst_group_seq_ + 1u) & 0x3Fu);
            // (3) The PROFILE change stays DEFERRED (§14.36 SIGSEGV rule intact).
            //     applyPendingDescriptorDataMode() runs at the top of the NEXT
            //     processBuffer — the same call that first runs
            //     accumulateBurstFrames() — so it lands strictly BEFORE frame 2 is
            //     demodulated, and before any finalize.
            if (cg.mod != current_modulation_ || cg.rate != code_rate_) {
                pending_descriptor_mod_ = cg.mod;
                pending_descriptor_rate_ = cg.rate;
                pending_descriptor_rate_change_ = true;
            }
            // (4) Frame 1 was DEMODULATED with the old profile (the deferral above
            //     cannot land before it). Its soft bits are noise AND the wrong
            //     width for bitsPerFrame(cg.cw) — pushing them makes the burst
            //     deinterleave THROW and the WHOLE group is dropped. Push a
            //     zero-LLR erasure of the correct width instead: exactly the
            //     late-join pattern. Cost = 1 frame, requested by the SACK bitmap
            //     in the next round-trip. Benefit = frames 2..N decode, and the
            //     group still emits its ack (the fast NACK that
            //     BUG-ANCHOR-WAIT-NO-ACK-STALL depends on).
            {
                const int bytes_per_cw = (cg.lifting_z == 81) ? 243 : 81;
                burst_soft_buffer_.emplace_back(
                    static_cast<size_t>(
                        fec::BurstInterleaver::bitsPerFrame(cg.cw, bytes_per_cw)),
                    0.0f);
            }
            burst_min_block_ = cg.frame_samples;
            ++commanded_geometry_arms_;
            LOG_MODEM(WARN,
                      "[%s] [COMMANDED-GEOMETRY] descriptor MISSED — arming group from "
                      "the rung we commanded: %s %s cw=%d z=%d frame=%zu samples "
                      "(latched was cw=%d); frame 1 erased by design. arm #%u",
                      log_prefix_.c_str(), modulationToString(cg.mod),
                      codeRateToString(cg.rate), cg.cw, cg.lifting_z, cg.frame_samples,
                      latched_cw, commanded_geometry_arms_);
        } else {
            burst_soft_buffer_.push_back(std::move(soft_bits));
            burst_min_block_ = static_cast<size_t>(
                waveform_->getMinSamplesForCWCount(fixed_frame_codewords_));
        }
        // Keep the early-decode cache positionally identical to the physical
        // soft-buffer from the instant accumulation arms. Continuation members
        // append their decoded results in tryDemodulateNextBurstFrame(); without
        // this frame-1 placeholder the cache is always N-1 entries, so finalize
        // rejects the whole cache and LDPC-decodes every continuation a second
        // time at the turnaround boundary.
        burst_predecoded_.emplace_back();
        burst_next_pos_ = sync_controller_.ring_.wrapRingIndexLocked(sync_position_ + frame_len);
        // F176/F221: data frames begin just past this header; the air-end is
        // published by refreshBurstAirEnd() once the group's ACTUAL per-frame
        // sample count is live (frame_len here is the HEADER's length — using
        // it declared a 30 s R1/4 burst as 1.7 s and gutted the gate).
        burst_data_start_abs_ =
            static_cast<uint64_t>(frame_sync_abs) + static_cast<uint64_t>(frame_len);
        burst_air_end_provisional_.store(false, std::memory_order_relaxed);
        provisional_burst_anchor_abs_.store(0, std::memory_order_relaxed);
        provisional_burst_anchor_full_chirp_.store(false, std::memory_order_relaxed);
        if (cg.armed) {
            // The descriptorless tuple knows frame stride but not N. Publish the
            // conservative sender-policy ceiling NOW, before returning to the event
            // loop. Waiting for frame 2's refresh leaves one full long-Z member with
            // burstAirSamplesRemaining()==0, during which an already-deferred SACK or
            // timer control could key over the peer. Exact tail finalization clears
            // this ceiling before the new group's own callback runs.
            refreshBurstAirEnd();
            const uint64_t armed_end =
                burst_air_end_abs_.load(std::memory_order_relaxed);
            commanded_geometry_air_gate_span_at_arm_ =
                armed_end > burst_data_start_abs_
                    ? armed_end - burst_data_start_abs_
                    : 0;
        } else {
            // Descriptor-proven groups publish their exact N-based span after the
            // first continuation establishes the live member stride. Commanded
            // geometry deliberately skips this zero store: its acquisition-time
            // provisional deadline must remain nonzero until refreshBurstAirEnd()
            // atomically replaces it with the conservative unknown-N ceiling.
            burst_air_end_abs_.store(0, std::memory_order_relaxed);
        }
        burst_snr_ = sync_snr_;
        burst_cfo_ = sync_cfo_;
        burst_start_time_ = std::chrono::steady_clock::now();

        // Feed back CFO from first frame (add pre-correction amount back)
        const float residual_cfo = waveform_->estimatedCFO();
        DecodeResult first_metrics;
        populateDecodeMetrics(first_metrics, is_ofdm, residual_cfo);
        burst_metric_templates_.push_back(first_metrics);
        accumulateBurstCarrierGamma();
        const float current_cfo = cfo_tracker_.tracked();
        const auto cfo_update = cfo_tracker_.ingestPilotResidual(
            frame_demodulator_.preCorrectionCfo(), residual_cfo, current_cfo, /*clamp_drift=*/true);
        burst_cfo_ = cfo_update.accepted_cfo;
        const float anchor_rms = sampleRMS(frame_buffer);
        burst_anchor_rms_ = anchor_rms;  // relative-erasure-gate reference for this group
        // Software-ALC: fresh per-group level accumulator (kept data frames 2..N only;
        // the chirp-anchored frame 1 rides 6-7 dB hotter under per-burst peak
        // normalization and would bias the data-segment level).
        burst_level_sum_sq_ = 0.0;
        burst_level_sample_count_ = 0;
        burst_level_peak_ = 0.0f;
        beginBurstDiagnosticsGroup(frame_sync_abs, burst_soft_buffer_.back(),
                                   anchor_rms,
                                   frame_demodulator_.preCorrectionCfo(), residual_cfo,
                                   cfo_update.accepted_cfo);

        // LTS autocorrelation can lock early on later marked groups inside a
        // long burst. The marker frame is retried above; keep this as a guard
        // for continuation slicing if residual timing is still large.
        const float burst_timing_offset = waveform_->getLastTimingOffsetSamples();
        constexpr float kBurstTimingCorrectionThreshold = 80.0f;
        constexpr float kBurstMaxTimingCorrection = 320.0f;
        if (std::abs(burst_timing_offset) >= kBurstTimingCorrectionThreshold &&
            std::abs(burst_timing_offset) <= kBurstMaxTimingCorrection) {
            const int sample_correction = static_cast<int>(std::lround(burst_timing_offset));
            burst_next_pos_ = sync_controller_.ring_.wrapRingIndexLocked(
                burst_next_pos_ + sync_controller_.ring_.buffer_capacity_samples_ + sample_correction);
            LOG_MODEM(WARN, "[%s] Burst group timing correction: %.1f samples, next_pos=%zu",
                      log_prefix_.c_str(), burst_timing_offset, burst_next_pos_);
        }

        // BUG-BURST-STALE-GEOMETRY: cadence reference for the commanded-geometry
        // guard. Stamped on EVERY group start (armed or not) — a run of missed
        // descriptors must not inflate the gap and mis-classify a steady-state group
        // as a post-RTO resend, which is why this is not last_descriptor_abs_sample_.
        // A sign-only marker with no descriptor/command provenance is not a group yet:
        // defer its stamp until a second substantive member proves physical cadence.
        if (burst_arm_provenance_ != BurstArmProvenance::UNPROVEN_MARKER) {
            last_group_start_abs_ = static_cast<uint64_t>(frame_sync_abs);
        }

        state_ = DecoderState::BURST_ACCUMULATING;
        // DELIBERATELY NO descriptor_mode_change_callback_ HERE (2026-07-28 review
        // fix; an earlier revision of this patch fired it and it was fatal).
        // The REAL descriptor fires that callback and then goes straight to SEARCHING
        // — no group is armed under it. From here the chain is
        //   onDescriptorModeChange -> applyDataMode -> notifyDataModeChanged
        //   -> app.cpp -> ModemEngine::setDataMode -> setConnectedOFDMMode
        //   -> pending_connected_ofdm_change_ = true
        // and applyPendingConnectedOFDMMode() at the TOP of the very next
        // processBuffer rebuilds waveform_, forces state_ = SEARCHING and clears
        // pending_total_cw_. That tears down the group we just armed BEFORE
        // accumulateBurstFrames() ever runs, so neither its group timeout nor its
        // FAST-NACK deliver fires and the group vanishes with NO tone-ack — strictly
        // worse than the bug (a 0/N group at least NACKs). The ARQ-window follow-
        // through stays with the real descriptor, which the sender re-announces on
        // the next group; one group at the old window costs at most a below-window
        // drop that ARQ resends, which is also exactly today's behaviour.
        return;  // processBuffer() will call accumulateBurstFrames() on next iteration
    }

    // Feed back pilot-corrected CFO to cached value (OFDM only).
    // MC-DPSK does not have pilot-based tracking; keep chirp-derived CFO.
    if (is_ofdm) {
        // After pre-correction, the demodulator sees near-zero CFO.
        // waveform_->estimatedCFO() returns the RESIDUAL (pre-correction error).
        // Add back the pre-correction amount to get the true total CFO.
        const float residual_cfo = waveform_->estimatedCFO();
        const float current_cfo = cfo_tracker_.tracked();
        const auto cfo_update = cfo_tracker_.ingestPilotResidual(
            frame_demodulator_.preCorrectionCfo(), residual_cfo, current_cfo, connected_);
        if (cfo_update.clamped) {
            LOG_MODEM(WARN, "[%s] Pilot CFO drift clamped: %.2f → %.2f Hz (drift=%.2f, max=%.1f)",
                      log_prefix_.c_str(), current_cfo, cfo_update.unclamped_cfo,
                      cfo_update.drift_hz, signal_policy::kMaxPilotCFODriftHz);
        }

        if (std::abs(cfo_update.accepted_cfo - current_cfo) > 0.1f) {
            LOG_MODEM(INFO, "[%s] CFO updated: %.2f → %.2f Hz (pre_corr=%.2f + residual=%.2f)",
                      log_prefix_.c_str(), current_cfo, cfo_update.accepted_cfo,
                      frame_demodulator_.preCorrectionCfo(), residual_cfo);
        }
        sync_cfo_ = cfo_update.accepted_cfo;
    }

    constexpr size_t LDPC_BLOCK = v2::LDPC_CODEWORD_BITS;
    CodeRate rate = connected_ ? code_rate_ : CodeRate::R1_4;

    // ========================================================================
    // CW0 peek for MC-DPSK (non-OFDM, 1-CW buffer)
    // MC-DPSK starts with 1-CW buffer, needs to check total_cw before decode
    // ========================================================================

    if (pending_total_cw_ == 0 && !is_ofdm && soft_bits.size() >= LDPC_BLOCK) {
        std::vector<float> cw0(soft_bits.begin(), soft_bits.begin() + LDPC_BLOCK);
        frame_decoder_.codec_->setRate(rate);
        auto [peek_ok, peek_data] = frame_decoder_.codec_->decode(cw0);

        if (peek_ok && peek_data.size() >= 4 && peek_data[0] == 0x55 && peek_data[1] == 0x4C) {
            // F227: a decoded CW0 with valid magic IS substantive peer-TX
            // evidence (LDPC-decoded header — not an F147 false-lock reject).
            // Without this stamp the MC-DPSK path was invisible to the
            // ack-repeat evidence gate: the silent repeat keyed up over the
            // peer's already-arriving burst (CCA can't see a quiet-chain
            // MC-DPSK burst), wiping frame 1 -> 30.9 s RTO every other cycle.
            stampRxSubstantive();
            auto hdr = v2::parseHeader(peek_data);
            if (hdr.valid && hdr.total_cw > 1) {
                int avail_cw = static_cast<int>(soft_bits.size() / LDPC_BLOCK);
                if (avail_cw < hdr.total_cw) {
                    pending_total_cw_ = hdr.total_cw;
                    state_ = DecoderState::SYNC_FOUND;
                    LOG_MODEM(INFO, "[%s] CW0 peek: need %d CWs (have %d) — waiting for %d samples",
                              log_prefix_.c_str(), hdr.total_cw, avail_cw,
                              waveform_->getMinSamplesForCWCount(hdr.total_cw));
                    return;
                }
            }
        } else {
            const bool wait_for_fixed_connect = [&]() {
                if (!allow_ping_detection) {
                    return true;
                }
                const auto ping_decision = evaluatePingDecision(true, false, false);
                // Gate the 4-CW CONNECT decode on the RATIOMETRIC silence test
                // only — NOT the full is_ping (which also fires on an absolute
                // data_rms<=0.16 floor via ping_by_chirp_lock). A real CONNECT on
                // a quieter analog chain (real soundcards/IONOS deliver ~half the
                // OTASim-calibrated in-band RMS) has data_rms~=training_rms (not
                // silent) yet a low absolute level; gating on is_ping skipped its
                // 4-CW decode entirely and PONGed it as a ping, so the handshake
                // never completed on hardware. A genuine bare-chirp PING is
                // ratiometrically silent (ping_by_silence), so it still short-
                // circuits here; an ambiguous low-level frame instead ATTEMPTS the
                // 4-CW decode, and the post-decode PATH2 fallback (which keeps the
                // absolute floor) re-classifies it as a ping only if that decode
                // also fails. Correctness (try to decode) over a fast false PONG.
                //
                // #70 stage 1.5: gap_is_noise is the NOISE-RELATIVE arm — the
                // payload region reads at the receiver's own measured ambient
                // floor (within kPingNoiseGapFactor), i.e. there is nothing to
                // decode. Unlike the absolute floor this does NOT re-create the
                // hardware regression above: a real low-level CONNECT still
                // rides well above its own chain's floor, fails this test, and
                // proceeds to the 4-CW decode. Without this arm, every noise-
                // flooded probe (ratio > 0.5 once broadband noise fills the gap,
                // ~10 dB and below) costs the full multi-second 4-CW wait before
                // PATH2 can classify it — the responder half of the #70
                // handshake floor.
                return !(ping_decision.ping_by_silence ||
                         ping_decision.gap_is_noise);
            }();
            if (wait_for_fixed_connect) {
                pending_total_cw_ = v2::kDefaultFixedFrameCodewords;
                state_ = DecoderState::SYNC_FOUND;
                LOG_MODEM(INFO,
                          "[%s] MC-DPSK CW0 failed with data energy; waiting for %d-CW fixed CONNECT frame (%d samples)",
                          log_prefix_.c_str(), pending_total_cw_,
                          waveform_->getMinSamplesForCWCount(pending_total_cw_));
                return;
            }
        }
    }

    // ========================================================================
    // CW0 peek for connected OFDM (control-sized initial buffer)
    // Was: try a 1-CW R1/4 decode + code_rate_ fallback to short-circuit
    //      control frames or read the multi-CW total_cw from the header.
    // Now: removed the decode attempts. Profiling (LLR distribution histogram,
    //      see plan v5) showed the R1/4 1-CW decode at this site succeeds 0
    //      times in 1300+ calls across multiple seeds, even at |LLR|>=6.
    //      Real 1-CW control frames are always R1/4 (hardened) and are
    //      caught upstream by the control-first hypothesis at line ~1136.
    //      Cost of the dead branch: ~36 s of decode CPU per 50 KB transfer.
    // Kept: LLR pre-empt as a coarse false-sync gate and the fixed-frame escalation
    //       that drives the actual data-frame decode.
    //
    // For high-order OFDM data modes, the robust control-sized window can demap
    // more than one full LDPC block while still being short of the fixed frame.
    // QAM16 R1/2 on 59 carriers produces exactly two CWs from the first
    // 10368-sample peek, so the old "< 2 CW" guard skipped escalation and the
    // fixed-frame decoder returned cw_ok=0/cw_fail=0 with insufficient bits.
    // ========================================================================
    // The connected DATA profile may use the descriptor-announced long LDPC
    // geometry (Z=81, N=1944).  This gate used the legacy 648-bit constant,
    // which makes one long CW look like a complete 3-CW/Z27 frame.  The
    // cw3/Z81 path then falls through to decodeFixedFrame() with only 1944 of
    // the required 5832 LLRs and returns 0/0 instead of waiting for the rest.
    const int peek_lifting_z = activeBurstLiftingZ();
    const size_t peek_ldpc_block =
        peek_lifting_z == 81 ? size_t{1944} : LDPC_BLOCK;
    const bool legacy_single_cw_peek =
        soft_bits.size() >= peek_ldpc_block &&
        soft_bits.size() < 2 * peek_ldpc_block;
    const bool ofdm_subfixed_peek =
        decode_policy::hasSubFixedFrameSoftBits(
            soft_bits.size(), fixed_frame_codewords_, peek_ldpc_block);
    if (pending_total_cw_ == 0 && is_ofdm && connected_
        && (legacy_single_cw_peek || ofdm_subfixed_peek)) {

        // Large OFDM FILE_BLOCK frames use variable-CW encoding without the
        // fixed-frame interleaver. Give raw CW0 one chance to declare the
        // true frame length before defaulting to the fixed-frame path.
        frame_decoder_.codec_->setRate(rate);
        {
            ultra::timing::ScopedTimer _profile_(
                ultra::timing::globalDecoderProfile().ofdm_cw0_probe_decode);
            std::pair<bool, Bytes> peek;
            if (peek_lifting_z == 81) {
                peek = robustDecodeSingleCW(
                    soft_bits.data(), peek_ldpc_block, rate,
                    log_prefix_.c_str(),
                    ultra::timing::SingleCWCallSite::Cw0Peek,
                    /*max_retries=*/4, peek_lifting_z);
            } else {
                peek = frame_decoder_.codec_->decode(
                    std::vector<float>(soft_bits.begin(),
                                       soft_bits.begin() + peek_ldpc_block));
            }
            auto [peek_ok, peek_data] = std::move(peek);
            const size_t bytes_per_cw =
                v2::getBytesPerCodewordZ(rate, peek_lifting_z);
            if (peek_ok && peek_data.size() >= bytes_per_cw) {
                if (peek_data.size() > bytes_per_cw) {
                    peek_data.resize(bytes_per_cw);
                }
                auto hdr = v2::parseHeader(peek_data);
                if (hdr.valid && !hdr.is_control &&
                    isFixedFrameCwCount(hdr.total_cw)) {
                    pending_total_cw_ = hdr.total_cw;
                    state_ = DecoderState::SYNC_FOUND;
                    LOG_MODEM(INFO, "[%s] OFDM fixed CW0: need %d CWs, waiting for %d samples",
                              log_prefix_.c_str(), pending_total_cw_,
                              waveform_->getMinSamplesForCWCount(pending_total_cw_));
                    return;
                }
                if (hdr.valid && !hdr.is_control &&
                    hdr.total_cw > v2::kMaxFixedFrameCodewords) {
                    pending_total_cw_ = hdr.total_cw;
                    state_ = DecoderState::SYNC_FOUND;
                    LOG_MODEM(INFO, "[%s] OFDM variable CW0: need %d CWs, waiting for %d samples",
                              log_prefix_.c_str(), pending_total_cw_,
                              waveform_->getMinSamplesForCWCount(pending_total_cw_));
                    return;
                }
            }
        }

        // Check LLR quality before expensive fixed-frame escalation.
        // False syncs (e.g. from fading-corrupted LTS) cluster near
        // |llr|_avg <= 1.0. Moderate SNR12 tail frames can be real at ~1.7,
        // so this gate stays permissive and lets LDPC be the final arbiter.
        // Escalating on garbage wastes seconds on failed LDPC attempts,
        // blocking the decoder from processing real frames arriving in the buffer.
        const size_t llr_count = std::min(soft_bits.size(), peek_ldpc_block);
        const float llr_abs_avg = signal_policy::meanAbsLLR(soft_bits.data(), llr_count);
        if (llr_abs_avg < signal_policy::kMinLLRForEscalation) {
            // F165: EXPECTED-ANCHOR IMMUNITY covers this gate too — the F147
            // fix held the lock at the LLR reject, then THIS peek gate dropped
            // the held anchor anyway ("holding lock" at 143.9, peek-skip at
            // 143.915, burst lost, no ack). With an armed expected anchor the
            // decode must run ALL the way (LDPC is the arbiter; the group /
            // erasure / backstop-ack machinery owns the failure).
            const bool hold_expected =
                connected_ && last_sync_expected_full_anchor_;
            if (!hold_expected) {
                ultra::timing::globalDecoderProfile()
                    .low_llr_escalation_skipped.fetch_add(1, std::memory_order_relaxed);
                LOG_MODEM(INFO, "[%s] OFDM CW0 peek: |llr|_avg=%.1f too low — skipping fixed-frame escalation (likely false sync)",
                          log_prefix_.c_str(), llr_abs_avg);
                advancePastFalseOFDMLock();
                state_ = DecoderState::SEARCHING;
                return;
            }
            LOG_MODEM(WARN,
                      "[%s] OFDM CW0 peek weak (|llr|_avg=%.1f) on EXPECTED full "
                      "anchor — escalating anyway (LDPC arbitrates)",
                      log_prefix_.c_str(), llr_abs_avg);
        }

        pending_total_cw_ = fixed_frame_codewords_;
        state_ = DecoderState::SYNC_FOUND;
        LOG_MODEM(INFO, "[%s] OFDM CW0 peek: |llr|_avg=%.1f, soft_bits=%zu, escalating to %d CWs",
                  log_prefix_.c_str(), llr_abs_avg, soft_bits.size(), pending_total_cw_);
        return;
    }

    // Decode the frame using the soft bits
    DecodeResult result = decodeFrame(soft_bits, sync_snr_, sync_cfo_);

    if (!result.success && result.codewords_ok == 1 && is_ofdm && connected_
        && !result.frame_data.empty()) {
        auto partial_hdr = v2::parseHeader(result.frame_data);
        if (partial_hdr.valid && !partial_hdr.is_control &&
            partial_hdr.total_cw > v2::kMaxFixedFrameCodewords) {
            pending_total_cw_ = partial_hdr.total_cw;
            state_ = DecoderState::SYNC_FOUND;
            LOG_MODEM(INFO, "[%s] OFDM variable frame: need %d CWs, waiting for %d samples",
                      log_prefix_.c_str(), pending_total_cw_,
                      waveform_->getMinSamplesForCWCount(pending_total_cw_));
            return;
        }
    }

    if (!result.success && fixed_frame_header_discovery_ && is_ofdm && connected_
        && pending_total_cw_ > v2::kMinFixedFrameCodewords && waveform_) {
        const int saved_pending_total_cw = pending_total_cw_;
        const CodeRate saved_code_rate = code_rate_;
        const Modulation saved_modulation = current_modulation_;

        std::vector<CodeRate> candidate_rates;
        auto addRate = [&](CodeRate candidate_rate) {
            if (std::find(candidate_rates.begin(), candidate_rates.end(), candidate_rate) ==
                candidate_rates.end()) {
                candidate_rates.push_back(candidate_rate);
            }
        };
        addRate(saved_code_rate);
        addRate(CodeRate::R1_4);
        addRate(CodeRate::R1_2);
        addRate(CodeRate::R2_3);
        addRate(CodeRate::R3_4);

        for (CodeRate candidate_rate : candidate_rates) {
            if (code_rate_ != candidate_rate || current_modulation_ != saved_modulation) {
                setDataMode(saved_modulation, candidate_rate);
            }

            for (int candidate_cw = saved_pending_total_cw - 1;
                 candidate_cw >= v2::kMinFixedFrameCodewords; --candidate_cw) {
                const size_t exact_size =
                    static_cast<size_t>(waveform_->getMinSamplesForCWCount(candidate_cw));
                if (exact_size == 0 || exact_size >= frame_buffer.size()) {
                    continue;
                }

                std::vector<float> exact_buffer(frame_buffer.begin(),
                                                frame_buffer.begin() + exact_size);
                waveform_->reset();
                waveform_->setAbsoluteTrainingPosition(frame_sync_abs);
                waveform_->setFrequencyOffset(decode_cfo);
                pending_total_cw_ = candidate_cw;

                LOG_MODEM(INFO, "[%s] Fixed-frame CW discovery: reprocessing %d-CW audio at %s (%zu samples)",
                          log_prefix_.c_str(), candidate_cw, codeRateToString(candidate_rate), exact_size);

                ultra::genie::eqTrace().site = "cw-discovery";
                if (!processWaveformForCodewords(
                        SampleSpan(exact_buffer.data(), exact_buffer.size()), candidate_cw)) {
                    continue;
                }

                captureConstellationSnapshot();
                auto candidate_bits = waveform_->getSoftBits();
                if (candidate_bits.empty()) {
                    continue;
                }

                auto candidate_result = decodeFrame(candidate_bits, sync_snr_, sync_cfo_);
                if (candidate_result.success) {
                    result = std::move(candidate_result);
                    frame_len = exact_size;
                    LOG_MODEM(INFO, "[%s] Fixed-frame CW discovery decoded %d-CW %s frame",
                              log_prefix_.c_str(), candidate_cw, codeRateToString(candidate_rate));
                    break;
                }
            }

            if (result.success) {
                break;
            }
        }

        if (!result.success) {
            pending_total_cw_ = saved_pending_total_cw;
            if (code_rate_ != saved_code_rate || current_modulation_ != saved_modulation) {
                setDataMode(saved_modulation, saved_code_rate);
            }
        }
    }

    // ========================================================================
    // Small-frame recovery for OFDM connected mode:
    // If full fixed-frame buffer decode failed, the frame might be a small non-data
    // frame (e.g. DISCONNECT = 2 CWs at R1/2) where trailing noise symbols
    // degraded LLR quality. Retry with 1-CW peek to determine actual size.
    // ========================================================================

    if (!result.success && is_ofdm && connected_) {
        size_t one_cw_s = static_cast<size_t>(waveform_->getMinSamplesForControlFrame());
        if (one_cw_s <= frame_buffer.size()) {
            waveform_->setFrequencyOffset(decode_cfo);
            ultra::genie::eqTrace().site = "smallframe-1cw";
            if (processWaveformForCodewords(SampleSpan(frame_buffer.data(), one_cw_s), 1)) {
                captureConstellationSnapshot();
            }
            auto short_bits = waveform_->getSoftBits();

            if (short_bits.size() >= LDPC_BLOCK) {
                // Try R1/4 first (control frames hardened), then code_rate_ fallback
                auto trySmallFrame = [&](CodeRate sr) -> bool {
                    auto [ok2, data2] = robustDecodeSingleCW(
                        short_bits.data(), LDPC_BLOCK, sr, log_prefix_.c_str());
                    if (!ok2 || data2.size() < 4 || data2[0] != 0x55 || data2[1] != 0x4C)
                        return false;
                    auto hdr2 = v2::parseHeader(data2);
                    if (hdr2.valid && hdr2.total_cw == 1) {
                        // 1-CW control frame — decode via decodeFrame (has R1/4 fast-path)
                        result = decodeFrame(short_bits, sync_snr_, sync_cfo_);
                        frame_len = one_cw_s;
                        return true;
                    } else if (hdr2.valid && hdr2.total_cw > 1 &&
                               isFixedFrameCwCount(hdr2.total_cw) &&
                               hdr2.total_cw < fixed_frame_codewords_) {
                        // Variable-CW frame (2-3 CWs) — reprocess with exact size
                        size_t exact_size = static_cast<size_t>(
                            waveform_->getMinSamplesForCWCount(hdr2.total_cw));
                        exact_size = std::min(exact_size, frame_buffer.size());

                        LOG_MODEM(INFO, "[%s] Small-frame recovery: reprocessing %zu samples (%d CWs)",
                                  log_prefix_.c_str(), exact_size, hdr2.total_cw);
                        waveform_->setFrequencyOffset(decode_cfo);
                        ultra::genie::eqTrace().site = "smallframe-exact";
                        if (processWaveformForCodewords(
                                SampleSpan(frame_buffer.data(), exact_size), hdr2.total_cw)) {
                            captureConstellationSnapshot();
                        }
                        auto recovered_bits = waveform_->getSoftBits();
                        result = decodeFrame(recovered_bits, sync_snr_, sync_cfo_);
                        frame_len = exact_size;
                        return true;
                    }
                    return false;
                };

                if (!trySmallFrame(CodeRate::R1_4) && rate != CodeRate::R1_4) {
                    trySmallFrame(rate);
                }
            }
        }
    }

    // Multi-candidate light-sync recovery (connected OFDM):
    // If decode fails at the detected sync point, retry nearby timing candidates.
    // detectDataSync() scans with coarse steps, and clean light-preamble locks can
    // still land late enough to leave only part of a fixed frame decodable.
    const int attempted_codewords = result.codewords_ok + result.codewords_failed;
    const bool partial_fixed_ofdm_failure =
        attempted_codewords >= 2 &&
        attempted_codewords <= v2::kMaxFixedFrameCodewords &&
        result.codewords_ok < attempted_codewords;
    const bool d8psk_data_mode = (current_modulation_ == Modulation::D8PSK);
    if (!result.success && is_ofdm && connected_ &&
        (result.codewords_ok == 0 || (d8psk_data_mode && partial_fixed_ofdm_failure))) {
        // Keep this recovery path gated by high sync correlation. Moderate-fading
        // hardware traces showed low-confidence syncs can pass the LLR gate, then
        // repeated full fixed-frame LDPC retries burn several seconds with zero
        // recoveries and trigger ARQ timeouts. Prefer earlier candidates first:
        // late light-sync locks show up as a positive LTS phase slope.
        const int d8psk_retry_deltas[] = {-32, -24, -16, -8, 8, 16, 24, 32};
        const int default_retry_deltas[] = {8, -8};
        const int* retry_deltas = d8psk_data_mode ? d8psk_retry_deltas : default_retry_deltas;
        const size_t retry_delta_count = d8psk_data_mode
            ? (sizeof(d8psk_retry_deltas) / sizeof(d8psk_retry_deltas[0]))
            : (sizeof(default_retry_deltas) / sizeof(default_retry_deltas[0]));
        bool recovered = false;
        int recovered_delta = 0;
        uint64_t recovery_attempts = 0;

        const bool allow_sync_recovery = frame_policy::allowSyncRecovery(sync_correlation_);
        if (!allow_sync_recovery) {
            LOG_MODEM(INFO, "[%s] Multi-candidate sync recovery skipped: corr=%.2f < %.2f",
                      log_prefix_.c_str(), sync_correlation_,
                      frame_policy::kMinSyncRecoveryCorrelation);
        }

        auto ringPosToAbsolute = [this](size_t ring_pos) -> size_t {
            if (sync_controller_.ring_.total_fed_ < sync_controller_.ring_.buffer_capacity_samples_) {
                return ring_pos;
            }
            const size_t oldest_abs = sync_controller_.ring_.total_fed_ - sync_controller_.ring_.buffer_capacity_samples_;
            const size_t oldest_pos = sync_controller_.ring_.write_pos_;
            const size_t offset = (ring_pos >= oldest_pos)
                ? (ring_pos - oldest_pos)
                : (sync_controller_.ring_.buffer_capacity_samples_ - oldest_pos + ring_pos);
            return oldest_abs + offset;
        };

        if (allow_sync_recovery) {
            for (size_t retry_idx = 0; retry_idx < retry_delta_count; ++retry_idx) {
                const int delta = retry_deltas[retry_idx];
                if (delta < 0 && sync_controller_.ring_.total_fed_ < sync_controller_.ring_.buffer_capacity_samples_ &&
                    sync_position_ < static_cast<size_t>(-delta)) {
                    continue;
                }
                recovery_attempts++;
                size_t retry_sync = sync_controller_.ring_.wrapRingIndexLocked(sync_position_ + sync_controller_.ring_.buffer_capacity_samples_ + delta);

                std::vector<float> retry_buffer;
                size_t retry_len = frame_len;
                {
                    std::lock_guard<std::mutex> lock(sync_controller_.ring_.buffer_mutex_);
                    size_t available;
                    if (sync_controller_.ring_.write_pos_ >= retry_sync) {
                        available = sync_controller_.ring_.write_pos_ - retry_sync;
                    } else {
                        available = sync_controller_.ring_.buffer_capacity_samples_ - retry_sync + sync_controller_.ring_.write_pos_;
                    }
                    retry_len = std::min(retry_len, available);
                    if (retry_len == 0) {
                        continue;
                    }
                    retry_buffer.resize(retry_len);
                    for (size_t i = 0; i < retry_len; ++i) {
                        retry_buffer[i] = sync_controller_.ring_.buffer_[sync_controller_.ring_.wrapRingIndexLocked(retry_sync + i)];
                    }
                }

                // Pre-correct CFO on retry buffer too
                if (is_ofdm && std::abs(frame_demodulator_.preCorrectionCfo()) > 0.01f) {
                    frame_demodulator_.applyCFOPreCorrection(retry_buffer, sync_cfo_, ringPosToAbsolute(retry_sync), log_prefix_.c_str());
                }

                waveform_->reset();
                waveform_->setAbsoluteTrainingPosition(ringPosToAbsolute(retry_sync));
                waveform_->setFrequencyOffset(decode_cfo);
                const int retry_expected_codewords =
                    expectedOFDMCodewordsForSamples(retry_buffer.size());
                ultra::genie::eqTrace().site = "sync-recovery";
                bool retry_ok = processWaveformForCodewords(
                    SampleSpan(retry_buffer.data(), retry_buffer.size()), retry_expected_codewords);
                if (!retry_ok) {
                    continue;
                }
                captureConstellationSnapshot();
                auto retry_bits = waveform_->getSoftBits();
                if (retry_bits.empty()) {
                    continue;
                }

                auto retry_result = decodeFrame(retry_bits, sync_snr_, sync_cfo_);
                if (d8psk_data_mode) {
                    if (!retry_result.success) {
                        continue;
                    }
                } else if (!(retry_result.success || retry_result.codewords_ok > 0)) {
                    continue;
                }

                LOG_MODEM(INFO, "[%s] Multi-candidate sync recovery: delta=%+d samples succeeded",
                          log_prefix_.c_str(), delta);

                // Keep CFO tracking consistent with the accepted retry candidate.
                const float residual_cfo = waveform_->estimatedCFO();
                const float current_cfo = cfo_tracker_.tracked();
                const auto cfo_update = cfo_tracker_.ingestPilotResidual(
                    frame_demodulator_.preCorrectionCfo(), residual_cfo, current_cfo, connected_);
                sync_cfo_ = cfo_update.accepted_cfo;

                sync_position_ = retry_sync;
                frame_len = retry_len;
                result = std::move(retry_result);
                recovered_delta = delta;
                recovered = true;
                break;
            }
        }

        if (recovery_attempts > 0) {
            std::lock_guard<std::mutex> slock(stats_mutex_);
            stats_.sync_recovery_attempts += recovery_attempts;
            if (recovered) {
                stats_.sync_recovery_successes++;
                switch (recovered_delta) {
                    case 8: stats_.sync_recovery_delta_p8++; break;
                    case -8: stats_.sync_recovery_delta_m8++; break;
                    case 16: stats_.sync_recovery_delta_p16++; break;
                    case -16: stats_.sync_recovery_delta_m16++; break;
                    case 24: stats_.sync_recovery_delta_p24++; break;
                    case -24: stats_.sync_recovery_delta_m24++; break;
                    case 32: stats_.sync_recovery_delta_p32++; break;
                    case -32: stats_.sync_recovery_delta_m32++; break;
                    default: break;
                }
            }
        }

        if (!recovered) {
            LOG_MODEM(DEBUG, "[%s] Multi-candidate sync recovery: no nearby offset decoded",
                      log_prefix_.c_str());
        }
    }

    if (allow_ping_detection && !result.success) {
        // PATH 2: real chirp lock plus no valid LDPC frame. This uses only
        // discriminator signals the receiver has already computed: the chirp
        // matched-filter lock/gap and the binary LDPC frame outcome.
        const bool ldpc_decode_succeeded = result.success;
        const bool ldpc_magic_valid =
            ldpc_decode_succeeded && result.frame_data.size() >= 2 &&
            result.frame_data[0] == 0x55 && result.frame_data[1] == 0x4C;
        const auto ping_decision =
            evaluatePingDecision(true, ldpc_decode_succeeded, ldpc_magic_valid);

        LOG_MODEM(INFO, "[%s] PING check PATH2: ratio=%.3f, "
                  "chirp_corr=%.3f (floor=%.2f), gap_error=%.1f (max=%.0f), "
                  "ldpc_ok=%d, magic=%d, inband_rms=%.4f noise_rms=%.4f "
                  "gap_is_noise=%d, path1=%d, path2=%d",
                  log_prefix_.c_str(), ping_decision.ratio,
                  ping_decision.chirp_corr, frame_policy::kPingCorrFloor,
                  ping_decision.gap_error_samples, frame_policy::kPingMaxGapError,
                  ping_decision.ldpc_decode_succeeded ? 1 : 0,
                  ping_decision.ldpc_magic_valid ? 1 : 0,
                  ping_decision.data_inband_rms, ping_decision.idle_noise_rms,
                  ping_decision.gap_is_noise ? 1 : 0,
                  ping_decision.ping_by_silence ? 1 : 0,
                  ping_decision.ping_by_chirp_lock ? 1 : 0);

        if (ping_decision.is_ping) {
            emitPingFrame(ping_decision, true);
            return;
        }
    }

    auto decode_end = std::chrono::steady_clock::now();
    float ms = std::chrono::duration<float, std::milli>(decode_end - decode_start).count();

    {
        std::lock_guard<std::mutex> slock(stats_mutex_);
        if (result.success) stats_.frames_decoded++;
        else stats_.frames_failed++;
        stats_.avg_decode_time_ms = 0.9f * stats_.avg_decode_time_ms + 0.1f * ms;
    }

    if (is_ofdm && waveform_) {
        populateDecodeMetrics(result, is_ofdm, waveform_->estimatedCFO());
    } else {
        populateDecodeMetrics(result, is_ofdm, sync_cfo_);
    }

    logFailureAttributionFrame(log_prefix_.c_str(), result, waveform_.get(),
                               is_ofdm, connected_, frame_sync_abs, frame_len,
                               current_modulation_, code_rate_, sync_correlation_);

    const bool is_non_data_frame = frame_policy::isNonDataFrame(
        result.success, result.frame_data.data(), result.frame_data.size());
    const bool clean_fading_timing = hasCleanFadingMeasurementTiming(result);
    const bool initial_link_quality_frame =
        result.success && !connected_ && waveform_;
    const bool measurement_quality_data_frame =
        result.success && connected_ && is_ofdm && !is_non_data_frame &&
        clean_fading_timing;
    if (initial_link_quality_frame) {
        last_fading_index_.store(waveform_->getFadingIndex());
    } else if (measurement_quality_data_frame) {
        last_fading_index_.store(result.lts_fading_index);
    } else if (result.success && connected_ && is_ofdm && !is_non_data_frame &&
               sync_from_full_anchor_fallback_) {
        LOG_MODEM(INFO,
                  "[%s] Fading measurement held: decoded via full-anchor DATA fallback "
                  "(candidate=%.3f, keeping %.3f)",
                  log_prefix_.c_str(), result.lts_fading_index,
                  last_fading_index_.load());
    } else if (result.success && connected_ && is_ofdm && !is_non_data_frame &&
               !clean_fading_timing) {
        LOG_MODEM(INFO,
                  "[%s] Fading measurement held: LTS timing offset %.1f samples "
                  "contaminates magnitude-CV (candidate=%.3f, keeping %.3f, "
                  "pilot_freq_cv=%.3f pilot_temporal_cv=%.3f pilot_symbol_cv=%.3f)",
                  log_prefix_.c_str(), result.lts_timing_offset_samples,
                  result.lts_fading_index, last_fading_index_.load(),
                  result.pilot_frequency_cv, result.pilot_temporal_cv,
                  result.pilot_symbol_mean_cv);
    }

    const bool deliver_partial_mc_dpsk =
        !result.success && mode_ == protocol::WaveformMode::MC_DPSK &&
        result.has_partial_codewords;
    if (result.success || result.codewords_ok > 0) {
        // F147: decoded codewords are SUBSTANTIVE peer-TX evidence (noise
        // candidates never survive LDPC) — feeds the ack-repeat cancel gate.
        stampRxSubstantive();
        const bool physical_burst_end =
            frame_policy::hasPhysicalBurstEndMarker(
                result.success, connected_, is_ofdm, burst_transport_rx_,
                result.frame_data);
        if (result.success && anchored_burst_backstop_armed_ &&
            v2::isDataFrame(result.frame_type) &&
            result.frame_type != v2::FrameType::DATA_REPAIR) {
            // Descriptor/group framing can fail while independently-decodable
            // Good-channel DATA survives. Preserve that positive evidence for
            // the timer fallback: it may still owe an async cumulative SACK if
            // the marked tail dies, but it must never report a fabricated 0/N.
            anchored_burst_payload_seen_ = true;
        }
        result.physical_turn_complete =
            physical_burst_end ||
            frame_policy::shouldClearProvisionalBurstAirGate(
                result.success, is_non_data_frame,
                burst_air_end_provisional_.load(std::memory_order_relaxed),
                provisional_burst_anchor_full_chirp_.load(std::memory_order_relaxed),
                provisional_burst_anchor_abs_.load(std::memory_order_relaxed),
                static_cast<uint64_t>(frame_sync_abs));
        if (result.physical_turn_complete) {
            // A successful classic decode at the accepted anchor is a standalone
            // frame. A CRC-protected physical-end marker likewise proves that a
            // later fallback DATA member is the actual wire tail. By contrast, an
            // unmarked later member keeps the conservative ceiling. A protocol
            // FINAL remains only a logical boundary. Clear before the callback so
            // its cumulative ACK carries safe reverse-egress provenance.
            clearProvisionalBurstAirGate();
        }
        {
            std::lock_guard<std::mutex> qlock(queue_mutex_);
            frame_queue_.push(result);
        }
        if ((result.success || deliver_partial_mc_dpsk) && frame_callback_) {
            frame_callback_(result);
        }
        if (resetDuringDecode()) {
            return;
        }

        if (result.success && connected_ && is_ofdm) {
            std::lock_guard<std::mutex> lock(sync_controller_.ring_.buffer_mutex_);
            sync_from_warm_timed_window_ = false;
        }

        if (last_physical_snr_valid_.load()) {
            // Two SNRs, two questions: physical = the CHANNEL's in-band S:N
            // (power ratio vs the burst-time noise ref); the routed SNR is the
            // demod-usable EFFECTIVE S:N (coherence-weighted — CFO/clock/jitter
            // wander lands in its residual). Their gap IS the hardware chain's
            // implementation loss, now visible per frame.
            LOG_MODEM(INFO, "[%s] StreamingDecoder: Frame decoded, %d/%d CWs, SNR=%.1f dB (%s) [physical %.1f dB in-band], CFO=%.1f Hz",
                      log_prefix_.c_str(), result.codewords_ok, result.codewords_ok + result.codewords_failed,
                      result.snr_db, snrSourceToString(result.snr_source),
                      last_physical_snr_db_.load(), result.cfo_hz);
        } else {
            LOG_MODEM(INFO, "[%s] StreamingDecoder: Frame decoded, %d/%d CWs, SNR=%.1f dB (%s), CFO=%.1f Hz",
                      log_prefix_.c_str(), result.codewords_ok, result.codewords_ok + result.codewords_failed,
                      result.snr_db, snrSourceToString(result.snr_source), result.cfo_hz);
        }
    } else {
        LOG_MODEM(WARN, "[%s] StreamingDecoder: Decode failed (cw_ok=%d, cw_fail=%d, is_ping=%d)",
                  log_prefix_.c_str(), result.codewords_ok, result.codewords_failed, result.is_ping ? 1 : 0);
    }

    // Calculate consumed samples based on actual frame content
    // For non-data frames (control/connect), use exact sample count to avoid eating into next frame
    size_t consumed = frame_len;
    if (result.success && is_ofdm && waveform_) {
        // Use exact sample count for decoded OFDM frames when the copied buffer
        // was larger than the frame. This matters when a header-derived CW count
        // is smaller than a configured/probe-sized buffer.
        const int actual_cw = result.codewords_ok + result.codewords_failed;
        if (actual_cw > 0) {
            const size_t exact_consumed =
                static_cast<size_t>(waveform_->getMinSamplesForCWCount(actual_cw));
            if (is_non_data_frame) {
                const size_t adjusted_consumed = frame_policy::consumedSamplesForDecodedFrame(
                    result.success, is_ofdm, is_non_data_frame, actual_cw, consumed, exact_consumed);
                if (adjusted_consumed < consumed) {
                    LOG_MODEM(INFO, "[%s] Non-data frame (%d CWs): advancing %zu samples (not %zu)",
                              log_prefix_.c_str(), actual_cw, adjusted_consumed, consumed);
                    consumed = adjusted_consumed;
                }
            } else if (exact_consumed < consumed) {
                LOG_MODEM(INFO, "[%s] DATA frame (%d CWs): advancing %zu samples (not %zu)",
                          log_prefix_.c_str(), actual_cw, exact_consumed, consumed);
                consumed = exact_consumed;
            }
        }
    }
    size_t next_block_pos = sync_controller_.ring_.wrapRingIndexLocked(sync_position_ + consumed);
    size_t next_search_abs = frame_sync_abs + consumed;

    if (result.success && connected_ && is_ofdm) {
        crater_reanchor_streak_ = 0;  // lever #3: a decode proves warm sync is fine
        noteFrameArrivalSuccess(frame_sync_abs, next_search_abs);
        std::lock_guard<std::mutex> lock(sync_controller_.ring_.buffer_mutex_);
        if (!is_non_data_frame) {
            sync_controller_.expect_full_ofdm_anchor_ = false;
        }
    } else if (!result.success && result.codewords_ok == 0 && !result.frame_evidence &&
               connected_ && is_ofdm) {
        std::lock_guard<std::mutex> lock(sync_controller_.ring_.buffer_mutex_);
        if (sync_from_warm_timed_window_) {
            noteFrameArrivalSyncMissLocked();
            sync_from_warm_timed_window_ = false;
        }
        // SELF-ECHO GUARD (2026-07-04, re-anchor-stall forensic wrp84o66o — the
        // measured 2.50->1.44 gap; knob ULTRA_ECHO_REANCHOR_GATE, default-ON):
        // while the tone-burst ACK monitor is ARMED we are the DATA-SENDER in our
        // post-burst ACK-listen turnaround — the peer emits ONLY a tone-burst ACK,
        // never OFDM data. A 0-CW OFDM "decode" here is our OWN transmitted burst
        // echoing back through the medium (self-echo; corr 0.96-0.98 at sample
        // positions inside our just-transmitted span). Forcing a full chirp+LTS
        // re-anchor on it re-arms a 120000-sample blind search that consumes the RX
        // path so we MISS the receiver's crater-demote tone-ACK (the fast path that
        // resolves a 16QAM crater in ~3.5 s: F4/F6/F7/F10/F11), dropping us to a
        // 2x-RTO collapse (~36 s: the F59/F21b outliers). Suppress the destructive
        // re-anchor during the ACK window; the tone monitor keeps running and hears
        // the ACK, so the existing fast demote fires. A genuine receiver decoding
        // real peer OFDM data does NOT arm the monitor -> isArmed()==false ->
        // legitimate crater re-anchoring is untouched. Sender-local, no wire change.
        static const bool kEchoReanchorGate = [] {
            const char* e = std::getenv("ULTRA_ECHO_REANCHOR_GATE");
            return !(e && e[0] == '0');  // default-ON; =0 restores legacy for A/B
        }();
        const bool ack_listen_self_echo =
            kEchoReanchorGate && tone_burst_monitor_.isArmed();
        // Lever #3 TWO-CRATER SYNC DISCIPLINE (ULTRA_CRATER_REANCHOR_HOLD, DEFAULT-ON
        // 2026-07-21; =0 opts out): a deep fade is an AMPLITUDE event, not a lost sync —
        // warm TIMING survives it (the samples are there, just attenuated). Forcing a
        // full chirp+LTS re-anchor on the FIRST crater treats the fade as a sync loss:
        // it delays the ACK (the RX is re-acquiring instead of signaling) and makes the
        // sender prepend a ~1.2s chirp. The RATE controller already declines to react to
        // a single crater (F122 two-crater rule); this gives the SYNC re-anchor the same
        // restraint — hold warm sync through crater #1, re-anchor only on crater #2.
        // Worst case is ONE extra warm-attempt group before the re-anchor (bounded), and
        // crater #2 still re-centers a genuinely-walked CFO exactly as before. Rig A/B
        // (F470-481, MPG@20): full re-anchors HALVED (25 vs 50), goodput +44% on
        // delivered runs (1.87 vs 1.30 kbps), delivery tied 4/6 (no regression); the
        // HOLD FAILs churned high re-anchors = correct fallback on sustained failure.
        static const bool kCraterReanchorHold = [] {
            const char* e = std::getenv("ULTRA_CRATER_REANCHOR_HOLD");
            // default-OFF (reverted 2026-07-23; =1 opts in). Was DEFAULT-ON 2026-07-21 on
            // the F470-481 A/B (re-anchors halved), but the aggregate all-levers-ON vs
            // baseline reckoning (F580-591) was a WASH and the "hold" levers over-hold a
            // failing rung (88s crater stalls). Kept behind the knob for measurement.
            return e && e[0] == '1' && e[1] == '\0';
        }();
        // CHEAP RE-ANCHOR (ULTRA_CHEAP_REANCHOR, default-off): a full dual-chirp (~1.2s)
        // re-acquires TIMING + CFO, but a fade only threatens the CFO (warm timing
        // survives). So on a crater, HOLD warm timing and roll the CFO back to the last
        // CERTIFIED value (last delivered group's proven CFO — un-poisons a walked CFO
        // for free), and fire the expensive full chirp only PERIODICALLY (every
        // kChirpEvery craters) as a "is sync really lost?" probe. A chirp that didn't
        // recover the link (still cratering) means a FADE, not lost sync — more chirps
        // won't help, so ride it warm. This attacks the ~8.4 re-anchors/run (each 1.2s)
        // that the EMA-hold FEEDS by holding a high rung through fades (F520-531). Legacy
        // lever #3 (below) chirps on crater #2; cheap drops that to 1 chirp per 4 craters.
        static const bool kCheapReanchor = [] {
            const char* e = std::getenv("ULTRA_CHEAP_REANCHOR");
            return e && e[0] == '1' && e[1] == '\0';  // default-off
        }();
        constexpr int kChirpEvery = 4;  // cheap: 1 full chirp per N craters, warm between
        ++crater_reanchor_streak_;
        bool force_chirp;
        if (!kCraterReanchorHold) {
            force_chirp = true;  // legacy: full chirp on every crater
        } else if (!kCheapReanchor) {
            force_chirp = (crater_reanchor_streak_ >= 2);  // lever #3: hold #1 only
        } else {
            force_chirp = (crater_reanchor_streak_ % kChirpEvery == 0);  // periodic probe
        }
        if (mode_ == protocol::WaveformMode::OFDM_CHIRP && !ack_listen_self_echo &&
            force_chirp) {
            resetFrameArrivalTrackingLocked();
            sync_controller_.expect_full_ofdm_anchor_ = true;
            sync_controller_.clearRejectStreak();
            LOG_MODEM(WARN,
                      "[%s] OFDM decode failed with 0/%d CWs; forcing full chirp+LTS re-anchor",
                      log_prefix_.c_str(), result.codewords_failed);
        } else if (mode_ == protocol::WaveformMode::OFDM_CHIRP && !ack_listen_self_echo) {
            // HOLD warm sync (no chirp). Cheap re-anchor: un-poison the CFO to the last
            // certified value so the warm retry uses a proven CFO, not a fade-walked one.
            const bool rolled = kCheapReanchor && cfo_tracker_.rollbackToCertified();
            LOG_MODEM(INFO,
                      "[%s] OFDM decode failed 0/%d CWs (crater #%d) — HOLDING warm sync%s "
                      "(fade is not a sync loss)",
                      log_prefix_.c_str(), result.codewords_failed, crater_reanchor_streak_,
                      rolled ? " + CFO rollback-to-certified" : "");
        } else if (ack_listen_self_echo) {
            LOG_MODEM(INFO,
                      "[%s] SELF-ECHO 0-CW during ACK-listen (monitor armed) — "
                      "skipping re-anchor so the crater-demote tone-ACK is heard",
                      log_prefix_.c_str());
        }
    }

    // Legacy fixed-offset continuation is only safe for a physical burst.
    // Marker-based bursts are handled earlier by BURST_ACCUMULATING; ordinary
    // ARQ-refilled frames must re-acquire sync because hardware audio latency
    // can insert gaps between frames. Treating those gaps as contiguous burst
    // payload decodes the wrong samples, wastes LDPC time, and misses the real
    // next frame.
    if (result.success && connected_ && is_ofdm && !is_non_data_frame
        && use_burst_interleave_ && waveform_ && waveform_->wasBurstInterleaved()) {
        size_t min_block = static_cast<size_t>(
            waveform_->getMinSamplesForCWCount(fixed_frame_codewords_));

        // Loop to decode multiple burst continuation blocks
        while (burst_blocks_decoded_ < MAX_BURST_BLOCKS) {
            // Check if there are enough samples for another block
            size_t next_available;
            {
                std::lock_guard<std::mutex> lock(sync_controller_.ring_.buffer_mutex_);
                if (sync_controller_.ring_.write_pos_ >= next_block_pos) {
                    next_available = sync_controller_.ring_.write_pos_ - next_block_pos;
                } else {
                    next_available = sync_controller_.ring_.buffer_capacity_samples_ - next_block_pos + sync_controller_.ring_.write_pos_;
                }
            }

            if (next_available < min_block) break;  // Not enough samples, burst over

            // Copy next block samples
            std::vector<float> next_block(min_block);
            {
                std::lock_guard<std::mutex> lock(sync_controller_.ring_.buffer_mutex_);
                for (size_t i = 0; i < min_block; i++) {
                    next_block[i] = sync_controller_.ring_.buffer_[sync_controller_.ring_.wrapRingIndexLocked(next_block_pos + i)];
                }
            }

            // Check RMS energy to detect if there's actually a signal
            float next_rms = 0.0f;
            size_t burst_check_start = std::min(size_t(1024), min_block);  // Skip training area
            size_t burst_check_len = std::min(min_block - burst_check_start, size_t(5000));
            if (burst_check_len > 0) {
                for (size_t i = 0; i < burst_check_len; i++) {
                    next_rms += next_block[burst_check_start + i] * next_block[burst_check_start + i];
                }
                next_rms = std::sqrt(next_rms / burst_check_len);
            }

            constexpr float BURST_ENERGY_THRESHOLD = 0.04f;
            if (next_rms < BURST_ENERGY_THRESHOLD) break;  // No energy, burst over

            // Energy present - try to decode as continuation block
            burst_blocks_decoded_++;
            LOG_MODEM(INFO, "[%s] Burst continuation: block %d, RMS=%.4f, pos=%zu",
                      log_prefix_.c_str(), burst_blocks_decoded_, next_rms, next_block_pos);

            waveform_->setFrequencyOffset(decode_cfo);
            ultra::genie::eqTrace().site = "burst-continue";
            bool next_ok = processWaveformForCodewords(
                SampleSpan(next_block.data(), next_block.size()), fixed_frame_codewords_);

            if (!next_ok) break;  // Process failed, burst over
            captureConstellationSnapshot();

            auto next_soft_bits = waveform_->getSoftBits();
            if (next_soft_bits.empty()) break;

            // LLR sanity check: if the soft bits look like noise (typical
            // sign of demodulating into the next frame's chirp/training
            // area, or into silence between frames), bail before paying
            // ~600ms of LDPC retry attempts that all fail. Real data
            // frames at usable SNR show |LLR|_avg >> 2; lost-frame attempts
            // collapse to |LLR|_avg < 1. See hardware-test analysis in
            // commit message — saves ~600ms per lost frame which lets
            // chirp-search lock on to the next real frame promptly.
            // 2026-05-28 Phase 3: probe size = one codeword (648 at z=27,
            // 1944 at z=81). Sourced from the active descriptor's lifting_z
            // so the LLR-quality heuristic samples the right span per CW.
            const size_t probe_cw_bits =
                (sync_controller_.have_burst_descriptor_ && sync_controller_.last_burst_descriptor_.lifting_z == 81)
                    ? size_t{1944} : size_t{648};
            const size_t burst_llr_n = std::min(next_soft_bits.size(), probe_cw_bits);
            const float burst_llr_avg = signal_policy::meanAbsLLR(
                next_soft_bits.data(), burst_llr_n);
            if (burst_llr_avg < signal_policy::kMinBurstContinuationLLR) {
                LOG_MODEM(INFO, "[%s] Burst continuation: bail at block %d "
                          "(|llr|_avg=%.2f < %.2f, no real frame here)",
                          log_prefix_.c_str(), burst_blocks_decoded_,
                          burst_llr_avg, signal_policy::kMinBurstContinuationLLR);
                break;
            }

            // Update CFO from pilot tracking
            const float residual_cfo = waveform_->estimatedCFO();
            const auto cfo_update = cfo_tracker_.ingestPilotResidual(
                0.0f, residual_cfo, sync_cfo_, /*clamp_drift=*/true);
            sync_cfo_ = cfo_update.accepted_cfo;

            // Decode the continuation block
            DecodeResult next_result = decodeFrame(next_soft_bits, sync_snr_, sync_cfo_);
            populateDecodeMetrics(next_result, true, residual_cfo);
            logFailureAttributionFrame(log_prefix_.c_str(), next_result, waveform_.get(),
                                       true, connected_, next_search_abs, min_block,
                                       current_modulation_, code_rate_, sync_correlation_);

            {
                std::lock_guard<std::mutex> slock(stats_mutex_);
                if (next_result.success) stats_.frames_decoded++;
                else stats_.frames_failed++;
            }

            if (next_result.success || next_result.codewords_ok > 0) {
                if (next_result.success && hasCleanFadingMeasurementTiming(next_result)) {
                    last_fading_index_.store(next_result.lts_fading_index);
                } else if (next_result.success) {
                    LOG_MODEM(INFO,
                              "[%s] Burst continuation fading measurement held: "
                              "LTS timing offset %.1f samples contaminates magnitude-CV "
                              "(candidate=%.3f, keeping %.3f, "
                              "pilot_freq_cv=%.3f pilot_temporal_cv=%.3f pilot_symbol_cv=%.3f)",
                              log_prefix_.c_str(),
                              next_result.lts_timing_offset_samples,
                              next_result.lts_fading_index,
                              last_fading_index_.load(),
                              next_result.pilot_frequency_cv,
                              next_result.pilot_temporal_cv,
                              next_result.pilot_symbol_mean_cv);
                }
                {
                    std::lock_guard<std::mutex> qlock(queue_mutex_);
                    frame_queue_.push(next_result);
                }
                if (next_result.success && frame_callback_) frame_callback_(next_result);
                if (resetDuringDecode()) {
                    return;
                }

                LOG_MODEM(INFO, "[%s] Burst block %d decoded: %d/%d CWs",
                          log_prefix_.c_str(), burst_blocks_decoded_,
                          next_result.codewords_ok, next_result.codewords_ok + next_result.codewords_failed);
            }

            if (next_result.success) {
                noteFrameArrivalSuccess(next_search_abs, next_search_abs + min_block);
            }

            // Advance position for next iteration (or final sync_controller_.ring_.correlation_pos_)
            sync_position_ = next_block_pos;
            next_block_pos = sync_controller_.ring_.wrapRingIndexLocked(next_block_pos + min_block);
            next_search_abs += min_block;

            // If decode failed completely, stop the burst
            if (!next_result.success && next_result.codewords_ok == 0 &&
                !next_result.frame_evidence) break;
        }
    }

    const bool can_continue_mc_dpsk =
        result.success &&
        connected_ &&
        mode_ == protocol::WaveformMode::MC_DPSK &&
        v2::isDataFrame(result.frame_type) &&
        result.frame_type != v2::FrameType::DATA_REPAIR &&
        decodedFrameHasMoreFrag(result);
    if (can_continue_mc_dpsk) {
        startMCDPSKBurstContinuation(next_block_pos, next_search_abs,
                                     sync_snr_, sync_cfo_);
        return;
    }

    // Burst over (or non-burst) - skip past everything we decoded and return to SEARCHING
    if (!is_ofdm) {
        pending_total_cw_ = 0;
    }
    burst_blocks_decoded_ = 0;
    {
        std::lock_guard<std::mutex> lock(sync_controller_.ring_.buffer_mutex_);
        sync_controller_.ring_.correlation_pos_ = sync_controller_.ring_.wrapRingIndexLocked(next_block_pos);
        sync_controller_.ring_.setSearchFloorLocked(next_search_abs);
        last_decoded_sync_pos_ = sync_position_;
    }

    state_ = DecoderState::SEARCHING;
}

void StreamingDecoder::startMCDPSKBurstContinuation(size_t next_pos, size_t next_abs,
                                                    float snr_db, float cfo_hz) {
    mc_burst_next_pos_ = next_pos;
    mc_burst_next_abs_ = next_abs;
    mc_burst_snr_ = snr_db;
    mc_burst_cfo_ = cfo_hz;
    mc_burst_frames_decoded_ = 1;
    mc_burst_pending_frame_ = false;
    mc_burst_pending_soft_bits_.clear();
    mc_burst_wait_start_time_ = std::chrono::steady_clock::now();
    state_ = DecoderState::MCDPSK_BURST_CONTINUING;

    LOG_MODEM(INFO, "[%s] MC-DPSK continuous burst: waiting for data-only continuation at pos=%zu",
              log_prefix_.c_str(), mc_burst_next_pos_);
}

void StreamingDecoder::finishMCDPSKBurstContinuation(size_t search_pos, size_t search_abs) {
    mc_burst_pending_frame_ = false;
    mc_burst_pending_soft_bits_.clear();
    mc_burst_frames_decoded_ = 0;

    {
        std::lock_guard<std::mutex> lock(sync_controller_.ring_.buffer_mutex_);
        sync_controller_.ring_.correlation_pos_ = search_pos;
        sync_controller_.ring_.setSearchFloorLocked(search_abs);
        last_decoded_sync_pos_ = sync_position_;
    }
    state_ = DecoderState::SEARCHING;
}

void StreamingDecoder::continueMCDPSKBurst() {
    auto* mc_waveform = dynamic_cast<MCDPSKWaveform*>(waveform_.get());
    if (!mc_waveform) {
        finishMCDPSKBurstContinuation(mc_burst_next_pos_, mc_burst_next_abs_);
        return;
    }

    constexpr size_t LDPC_BLOCK = v2::LDPC_CODEWORD_BITS;
    constexpr float kContinuationRmsFloor = 0.012f;
    constexpr int kSampleRateHz = 48000;

    const int one_cw_samples_i = mc_waveform->getDataOnlySamplesForCWCount(1);
    if (one_cw_samples_i <= 0) {
        finishMCDPSKBurstContinuation(mc_burst_next_pos_, mc_burst_next_abs_);
        return;
    }
    const size_t one_cw_samples = static_cast<size_t>(one_cw_samples_i);

    auto availableFrom = [&](size_t pos) -> size_t {
        std::lock_guard<std::mutex> lock(sync_controller_.ring_.buffer_mutex_);
        if (sync_controller_.ring_.write_pos_ >= pos) {
            return sync_controller_.ring_.write_pos_ - pos;
        }
        return sync_controller_.ring_.buffer_capacity_samples_ - pos + sync_controller_.ring_.write_pos_;
    };

    auto copyFrom = [&](size_t pos, size_t len) -> std::vector<float> {
        std::vector<float> out(len);
        std::lock_guard<std::mutex> lock(sync_controller_.ring_.buffer_mutex_);
        for (size_t i = 0; i < len; ++i) {
            out[i] = sync_controller_.ring_.buffer_[sync_controller_.ring_.wrapRingIndexLocked(pos + i)];
        }
        return out;
    };

    auto timedOutWaitingFor = [&](size_t samples_needed) -> bool {
        const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - mc_burst_wait_start_time_).count();
        const int needed_ms = static_cast<int>(
            (samples_needed * 1000 + kSampleRateHz - 1) / kSampleRateHz);
        return elapsed_ms > needed_ms + 1000;
    };

    auto deliverContinuationResult = [&](DecodeResult& result) {
        populateDecodeMetrics(result, false, mc_burst_cfo_);
        {
            std::lock_guard<std::mutex> slock(stats_mutex_);
            if (result.success) {
                stats_.frames_decoded++;
            } else {
                stats_.frames_failed++;
            }
        }

        const bool deliver_partial =
            !result.success && result.has_partial_codewords;
        if (result.success || result.codewords_ok > 0) {
            {
                std::lock_guard<std::mutex> qlock(queue_mutex_);
                frame_queue_.push(result);
            }
            if ((result.success || deliver_partial) && frame_callback_) {
                frame_callback_(result);
            }
        }
    };

    const size_t frame_start_pos = mc_burst_pending_frame_
        ? mc_burst_pending_start_pos_
        : mc_burst_next_pos_;
    const size_t frame_start_abs = mc_burst_pending_frame_
        ? mc_burst_pending_start_abs_
        : mc_burst_next_abs_;

    if (!mc_burst_pending_frame_) {
        const size_t available = availableFrom(mc_burst_next_pos_);
        if (available < one_cw_samples) {
            if (timedOutWaitingFor(one_cw_samples)) {
                LOG_MODEM(INFO, "[%s] MC-DPSK continuous burst ended after %d frame(s)",
                          log_prefix_.c_str(), mc_burst_frames_decoded_);
                finishMCDPSKBurstContinuation(mc_burst_next_pos_, mc_burst_next_abs_);
            }
            return;
        }

        std::vector<float> cw0_samples = copyFrom(mc_burst_next_pos_, one_cw_samples);
        const float rms = sampleRMS(cw0_samples);
        if (rms < kContinuationRmsFloor) {
            LOG_MODEM(INFO, "[%s] MC-DPSK continuous burst tail: RMS=%.4f, frames=%d",
                      log_prefix_.c_str(), rms, mc_burst_frames_decoded_);
            finishMCDPSKBurstContinuation(mc_burst_next_pos_, mc_burst_next_abs_);
            return;
        }

        if (!mc_waveform->processDataOnly(SampleSpan(cw0_samples.data(), cw0_samples.size()))) {
            LOG_MODEM(INFO, "[%s] MC-DPSK continuation demod failed at pos=%zu",
                      log_prefix_.c_str(), mc_burst_next_pos_);
            finishMCDPSKBurstContinuation(mc_burst_next_pos_, mc_burst_next_abs_);
            return;
        }

        auto soft = mc_waveform->getSoftBits();
        if (soft.size() < LDPC_BLOCK) {
            finishMCDPSKBurstContinuation(mc_burst_next_pos_, mc_burst_next_abs_);
            return;
        }

        frame_decoder_.codec_->setRate(code_rate_);
        std::vector<float> cw0_bits(soft.begin(), soft.begin() + LDPC_BLOCK);
        auto [peek_ok, peek_data] = frame_decoder_.codec_->decode(cw0_bits);
        const size_t bytes_per_cw = v2::getBytesPerCodeword(code_rate_);
        if (!peek_ok || peek_data.size() < bytes_per_cw ||
            peek_data[0] != 0x55 || peek_data[1] != 0x4C) {
            LOG_MODEM(INFO, "[%s] MC-DPSK continuous burst stop: no continuation header at pos=%zu",
                      log_prefix_.c_str(), mc_burst_next_pos_);
            finishMCDPSKBurstContinuation(mc_burst_next_pos_, mc_burst_next_abs_);
            return;
        }
        if (peek_data.size() > bytes_per_cw) {
            peek_data.resize(bytes_per_cw);
        }

        auto hdr = v2::parseHeader(peek_data);
        if (!hdr.valid || !v2::isDataFrame(hdr.type) ||
            hdr.type == v2::FrameType::DATA_REPAIR ||
            hdr.total_cw == 0 || hdr.total_cw > 32) {
            LOG_MODEM(INFO, "[%s] MC-DPSK continuous burst stop: header type=%s valid=%d cw=%d",
                      log_prefix_.c_str(), v2::frameTypeToString(hdr.type),
                      hdr.valid ? 1 : 0, hdr.total_cw);
            finishMCDPSKBurstContinuation(mc_burst_next_pos_, mc_burst_next_abs_);
            return;
        }

        mc_burst_pending_frame_ = true;
        mc_burst_pending_type_ = hdr.type;
        mc_burst_pending_total_cw_ = hdr.total_cw;
        mc_burst_pending_start_pos_ = mc_burst_next_pos_;
        mc_burst_pending_start_abs_ = mc_burst_next_abs_;
        mc_burst_pending_total_samples_ = static_cast<size_t>(
            mc_waveform->getDataOnlySamplesForCWCount(hdr.total_cw));
        mc_burst_pending_consumed_samples_ = one_cw_samples;
        mc_burst_pending_soft_bits_ = std::move(soft);
        mc_burst_next_pos_ = sync_controller_.ring_.wrapRingIndexLocked(mc_burst_next_pos_ + one_cw_samples);
        mc_burst_next_abs_ += one_cw_samples;
        mc_burst_wait_start_time_ = std::chrono::steady_clock::now();
    }

    const size_t remaining_samples =
        mc_burst_pending_total_samples_ > mc_burst_pending_consumed_samples_
            ? mc_burst_pending_total_samples_ - mc_burst_pending_consumed_samples_
            : 0;

    if (remaining_samples > 0) {
        const size_t available = availableFrom(mc_burst_next_pos_);
        if (available < remaining_samples) {
            if (timedOutWaitingFor(remaining_samples)) {
                DecodeResult partial = decodeMCDPSKFrame(
                    mc_burst_pending_soft_bits_, code_rate_,
                    v2::getBytesPerCodeword(code_rate_), mc_burst_snr_, mc_burst_cfo_);
                deliverContinuationResult(partial);
                LOG_MODEM(WARN, "[%s] MC-DPSK continuous burst partial timeout: %s cw=%d",
                          log_prefix_.c_str(), v2::frameTypeToString(mc_burst_pending_type_),
                          mc_burst_pending_total_cw_);
                finishMCDPSKBurstContinuation(mc_burst_next_pos_, mc_burst_next_abs_);
            }
            return;
        }

        std::vector<float> rest = copyFrom(mc_burst_next_pos_, remaining_samples);
        if (!mc_waveform->processDataOnly(SampleSpan(rest.data(), rest.size()))) {
            finishMCDPSKBurstContinuation(frame_start_pos, frame_start_abs);
            return;
        }
        auto rest_soft = mc_waveform->getSoftBits();
        mc_burst_pending_soft_bits_.insert(mc_burst_pending_soft_bits_.end(),
                                          rest_soft.begin(), rest_soft.end());
        mc_burst_next_pos_ = sync_controller_.ring_.wrapRingIndexLocked(mc_burst_next_pos_ + remaining_samples);
        mc_burst_next_abs_ += remaining_samples;
    }

    DecodeResult result = decodeMCDPSKFrame(
        mc_burst_pending_soft_bits_, code_rate_, v2::getBytesPerCodeword(code_rate_),
        mc_burst_snr_, mc_burst_cfo_);
    deliverContinuationResult(result);

    const bool more_frag = decodedFrameHasMoreFrag(result);
    const int total_cw = mc_burst_pending_total_cw_;
    mc_burst_pending_frame_ = false;
    mc_burst_pending_soft_bits_.clear();
    mc_burst_frames_decoded_++;
    mc_burst_wait_start_time_ = std::chrono::steady_clock::now();

    LOG_MODEM(INFO, "[%s] MC-DPSK continuous burst frame %d decoded: %s %d/%d CW",
              log_prefix_.c_str(), mc_burst_frames_decoded_,
              result.success ? "OK" : "partial",
              result.codewords_ok, result.codewords_ok + result.codewords_failed);

    if (!result.success && result.codewords_ok == 0) {
        finishMCDPSKBurstContinuation(frame_start_pos, frame_start_abs);
        return;
    }

    if (!more_frag || mc_burst_frames_decoded_ >= MC_DPSK_MAX_BURST_FRAMES) {
        LOG_MODEM(INFO, "[%s] MC-DPSK continuous burst complete: frames=%d last_cw=%d",
                  log_prefix_.c_str(), mc_burst_frames_decoded_, total_cw);
        finishMCDPSKBurstContinuation(mc_burst_next_pos_, mc_burst_next_abs_);
    }
}
// ============================================================================
// HELPERS
// ============================================================================

float StreamingDecoder::chirpSyncQualityDb(float corr, float /*noise*/) {
    float snr = (corr - 0.15f) / 0.03f;
    return std::max(-5.0f, std::min(30.0f, snr));
}

// ============================================================================
// MC-DPSK: Simple sequential codeword decode (no frame interleaving)
// ============================================================================
DecodeResult StreamingDecoder::decodeMCDPSKFrame(const std::vector<float>& soft_bits,
                                                   CodeRate rate, size_t bytes_per_cw,
                                                   float snr, float cfo) {
    DecodeResult result;
    result.snr_db = snr;
    result.snr_source = SNRSource::SYNC_QUALITY;
    result.sync_quality_db = snr;
    result.cfo_hz = cfo;

    constexpr size_t LDPC_BLOCK = v2::LDPC_CODEWORD_BITS;

    if (soft_bits.size() < LDPC_BLOCK) return result;

    frame_decoder_.codec_->setRate(rate);

    auto tryFixedConnectFrame = [&]() -> DecodeResult {
        DecodeResult fixed;
        fixed.snr_db = snr;
        fixed.snr_source = SNRSource::SYNC_QUALITY;
        fixed.sync_quality_db = snr;
        fixed.cfo_hz = cfo;

        const int cw_count = v2::kDefaultFixedFrameCodewords;
        const size_t frame_bits =
            static_cast<size_t>(fec::FrameInterleaver::totalFrameBits(cw_count));
        if (soft_bits.size() < frame_bits) {
            return fixed;
        }

        auto status = v2::decodeFixedFrame(soft_bits, CodeRate::R1_4, cw_count);
        for (size_t i = 0; i < status.decoded.size(); ++i) {
            if (status.decoded[i]) {
                fixed.codewords_ok++;
            } else {
                fixed.codewords_failed++;
            }
        }
        if (!status.allSuccess()) {
            return fixed;
        }

        Bytes assembled = status.reassemble();
        auto hdr = v2::parseHeader(assembled);
        if (!hdr.valid || !v2::isConnectFrame(hdr.type)) {
            return fixed;
        }

        fixed.success = true;
        fixed.frame_data = std::move(assembled);
        fixed.frame_type = hdr.type;
        LOG_MODEM(INFO, "[%s] MC-DPSK fixed CONNECT decode SUCCESS (%d/%d CWs)",
                  log_prefix_.c_str(), fixed.codewords_ok,
                  fixed.codewords_ok + fixed.codewords_failed);
        return fixed;
    };

    // Decode CW0 (header codeword)
    std::vector<float> cw0_bits(soft_bits.begin(), soft_bits.begin() + LDPC_BLOCK);
    auto [ok0, data0] = frame_decoder_.codec_->decode(cw0_bits);

    if (!ok0 || data0.size() < 2 || data0[0] != 0x55 || data0[1] != 0x4C) {
        auto fixed = tryFixedConnectFrame();
        if (fixed.success) {
            return fixed;
        }
        LOG_MODEM(INFO, "[%s] MC-DPSK: CW0 decode failed (ok=%d, size=%zu, magic=0x%02X%02X)",
                  log_prefix_.c_str(), ok0 ? 1 : 0, data0.size(),
                  data0.size() >= 1 ? data0[0] : 0, data0.size() >= 2 ? data0[1] : 0);
        return result;
    }

    // Truncate to bytes_per_cw if needed
    if (data0.size() > bytes_per_cw) data0.resize(bytes_per_cw);

    // Parse header
    auto hdr = v2::parseHeader(data0);
    if (!hdr.valid) {
        auto fixed = tryFixedConnectFrame();
        if (fixed.success) {
            return fixed;
        }
        // Log CRC details for control frames (ACK, etc.)
        if (data0.size() >= 20) {
            uint16_t received_crc = (static_cast<uint16_t>(data0[18]) << 8) | data0[19];

            // Full 18 bytes
            uint8_t full18[18];
            for (int i = 0; i < 18; i++) full18[i] = data0[i];
            uint16_t calc18 = v2::ControlFrame::calculateCRC(full18, 18);

            // Print all 20 bytes for inspection
            LOG_MODEM(INFO, "[%s] MC-DPSK: CRC fail - rcv=0x%04X calc18=0x%04X",
                      log_prefix_.c_str(), received_crc, calc18);
            LOG_MODEM(INFO, "[%s] MC-DPSK: bytes[0-9]  = %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
                      log_prefix_.c_str(),
                      data0[0], data0[1], data0[2], data0[3], data0[4],
                      data0[5], data0[6], data0[7], data0[8], data0[9]);
            LOG_MODEM(INFO, "[%s] MC-DPSK: bytes[10-19] = %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
                      log_prefix_.c_str(),
                      data0[10], data0[11], data0[12], data0[13], data0[14],
                      data0[15], data0[16], data0[17], data0[18], data0[19]);
        } else {
            LOG_MODEM(INFO, "[%s] MC-DPSK: Header invalid - data too small (%zu bytes)",
                      log_prefix_.c_str(), data0.size());
        }
        return result;
    }

    result.frame_type = hdr.type;
    result.codewords_ok = 1;

    if (hdr.type == v2::FrameType::DATA_REPAIR) {
        auto repair_header = v2::DataRepairFrame::parseHeader(data0);
        if (!repair_header) {
            LOG_MODEM(INFO, "[%s] MC-DPSK: DATA_REPAIR header invalid", log_prefix_.c_str());
            return result;
        }

        const int total_cw = static_cast<int>(repair_header->repair_count) + 1;
        const int avail_cw = static_cast<int>(soft_bits.size() / LDPC_BLOCK);
        auto repair_indices = repair_header->repairIndices();

        result.partial_codewords.type = v2::FrameType::DATA;
        result.partial_codewords.flags = v2::Flags::VERSION_V2;
        result.partial_codewords.seq = repair_header->target_seq;
        result.partial_codewords.src_hash = repair_header->src_hash;
        result.partial_codewords.dst_hash = repair_header->dst_hash;
        result.partial_codewords.total_cw = repair_header->original_total_cw;
        result.partial_codewords.decoded_bitmap = 0;
        result.partial_codewords.from_repair = true;
        result.partial_codewords.data.resize(repair_header->original_total_cw);

        v2::DataRepairFrame decoded_repair = *repair_header;
        decoded_repair.repair_codewords.clear();

        bool all_repair_cws_ok = avail_cw >= total_cw;
        const int decodable_repair_cw = std::min<int>(
            static_cast<int>(repair_indices.size()), std::max(0, avail_cw - 1));
        for (int i = 0; i < decodable_repair_cw; ++i) {
            const int cw_num = i + 1;
            const size_t off = static_cast<size_t>(cw_num) * LDPC_BLOCK;
            std::vector<float> bits(soft_bits.begin() + off, soft_bits.begin() + off + LDPC_BLOCK);
            auto [ok, data] = frame_decoder_.codec_->decode(bits);
            const uint8_t original_cw = repair_indices[static_cast<size_t>(i)];
            if (ok && data.size() >= bytes_per_cw) {
                data.resize(bytes_per_cw);
                result.partial_codewords.decoded_bitmap |= (1u << original_cw);
                result.partial_codewords.data[original_cw] = data;
                decoded_repair.repair_codewords.push_back(data);
                result.codewords_ok++;
            } else {
                all_repair_cws_ok = false;
                result.codewords_failed++;
            }
        }
        if (avail_cw < total_cw) {
            result.codewords_failed += total_cw - avail_cw;
        }

        if (all_repair_cws_ok &&
            decoded_repair.repair_codewords.size() == repair_indices.size()) {
            result.success = true;
            result.frame_data = decoded_repair.serialize();
            LOG_MODEM(INFO, "[%s] MC-DPSK: DATA_REPAIR seq=%d bitmap=0x%04X decoded",
                      log_prefix_.c_str(), repair_header->target_seq,
                      repair_header->repair_bitmap);
        } else {
            result.has_partial_codewords = result.partial_codewords.valid();
            LOG_MODEM(INFO, "[%s] MC-DPSK: DATA_REPAIR seq=%d partial bitmap=0x%08X/%04X",
                      log_prefix_.c_str(), repair_header->target_seq,
                      result.partial_codewords.decoded_bitmap,
                      repair_header->repair_bitmap);
        }
        return result;
    }

    // 1-CW frame (control frame like ACK, PROBE, etc.)
    if (hdr.total_cw == 1) {
        result.success = true;
        result.frame_data = data0;
        LOG_MODEM(DEBUG, "[%s] MC-DPSK: Control frame (1 CW) decoded", log_prefix_.c_str());
        return result;
    }

    // Multi-CW frame - decode remaining codewords sequentially
    int total_cw = hdr.total_cw;
    int avail_cw = static_cast<int>(soft_bits.size() / LDPC_BLOCK);

    result.partial_codewords.type = hdr.type;
    result.partial_codewords.flags = data0.size() >= 4 ? data0[3] : v2::Flags::VERSION_V2;
    result.partial_codewords.seq = hdr.seq;
    result.partial_codewords.src_hash = hdr.src_hash;
    result.partial_codewords.dst_hash = hdr.dst_hash;
    result.partial_codewords.total_cw = static_cast<uint8_t>(std::clamp(total_cw, 0, 32));
    result.partial_codewords.decoded_bitmap = 0x1u;
    result.partial_codewords.data.resize(static_cast<size_t>(std::max(total_cw, 0)));
    result.partial_codewords.data[0] = data0;

    if (avail_cw < total_cw) {
        // Not enough codewords available
        result.frame_data = data0;
        result.codewords_failed = std::max(0, total_cw - avail_cw);
        result.has_partial_codewords = result.partial_codewords.valid();
        LOG_MODEM(DEBUG, "[%s] MC-DPSK: Need %d CWs, have %d - partial", log_prefix_.c_str(), total_cw, avail_cw);
        return result;
    }

    // Set up codeword status for reassembly
    v2::CodewordStatus cw_status;
    cw_status.decoded.resize(total_cw, false);
    cw_status.data.resize(total_cw);
    cw_status.decoded[0] = true;
    cw_status.data[0] = data0;

    // Decode CW1+
    for (int i = 1; i < total_cw; i++) {
        size_t off = i * LDPC_BLOCK;
        std::vector<float> bits(soft_bits.begin() + off, soft_bits.begin() + off + LDPC_BLOCK);

        auto [ok, data] = frame_decoder_.codec_->decode(bits);
        if (ok && data.size() >= bytes_per_cw) {
            data.resize(bytes_per_cw);
            cw_status.decoded[i] = true;
            cw_status.data[i] = data;
            result.partial_codewords.decoded_bitmap |= (1u << i);
            result.partial_codewords.data[static_cast<size_t>(i)] = data;
            result.codewords_ok++;
        } else {
            result.codewords_failed++;
        }
    }

    if (cw_status.allSuccess()) {
        result.success = true;
        result.frame_data = cw_status.reassemble();
        LOG_MODEM(DEBUG, "[%s] MC-DPSK: %d/%d CWs decoded OK", log_prefix_.c_str(), total_cw, total_cw);
    } else {
        result.has_partial_codewords = result.partial_codewords.valid();
        LOG_MODEM(DEBUG, "[%s] MC-DPSK: %d/%d CWs failed", log_prefix_.c_str(),
                  result.codewords_failed, total_cw);
    }

    return result;
}

DecodeResult StreamingDecoder::decodeFrame(const std::vector<float>& soft_bits, float snr, float cfo) {
    DecodeResult result;
    result.snr_db = snr;
    result.snr_source = SNRSource::SYNC_QUALITY;
    result.sync_quality_db = snr;
    result.cfo_hz = cfo;

    constexpr size_t LDPC_BLOCK = v2::LDPC_CODEWORD_BITS;

    if (soft_bits.size() < LDPC_BLOCK) return result;

    CodeRate rate = connected_ ? code_rate_ : CodeRate::R1_4;
    size_t bytes_per_cw = v2::getBytesPerCodeword(rate);
    frame_decoder_.codec_->setRate(rate);

    // Channel interleaving only applies to OFDM modes, NOT MC-DPSK
    bool is_ofdm = protocol::isOFDMMode(mode_);
    bool apply_channel_deinterleave = use_channel_interleave_ && is_ofdm;

    // Helper to deinterleave a codeword if needed
    auto deinterleave_cw = [&](const std::vector<float>& cw) -> std::vector<float> {
        if (apply_channel_deinterleave) {
            return frame_decoder_.interleaver_->deinterleave(cw);
        }
        return cw;
    };

    // ========================================================================
    // MC-DPSK: Simple sequential decode (no frame interleaving ever)
    // OFDM: "Try Both" strategy with frame interleaving for fixed-CW frames
    // ========================================================================

    // For MC-DPSK, ALWAYS use simple sequential decode - skip all frame interleaving logic
    if (mode_ == protocol::WaveformMode::MC_DPSK) {
        return decodeMCDPSKFrame(soft_bits, rate, bytes_per_cw, snr, cfo);
    }

    // ========================================================================
    // OFDM: "Try Both" Strategy for Frame Interleaving
    // ========================================================================
    // 1. First, try to decode first 648 bits as non-interleaved CW0
    // 2. If it's a valid 1-CW control frame → done
    // 3. If decode fails OR it's a fixed-CW frame → try frame-interleaved decode
    // ========================================================================

    // Skip single-CW control probes when we already know this is a fixed-CW data
    // frame. Burst-interleaved logical frames and frames latched by CW0 peek
    // are data-only; probing them as R1/4 control burns LDPC time on the Pi
    // before the real fixed-frame decode starts.
    const bool pending_fixed_cw = isFixedFrameCwCount(pending_total_cw_);
    const bool known_fixed_cw =
        pending_fixed_cw ||
        (use_burst_interleave_ && waveform_ && waveform_->wasBurstInterleaved());

    // R1/4 fast-path: control frames are always encoded at R1/4 (hardened)
    // Try R1/4 first — if it's a valid 1-CW control frame, return immediately.
    if (!known_fixed_cw && rate != CodeRate::R1_4 && soft_bits.size() >= LDPC_BLOCK) {
        frame_decoder_.codec_->setRate(CodeRate::R1_4);
        std::vector<float> cw0_r14(soft_bits.begin(), soft_bits.begin() + LDPC_BLOCK);
        std::pair<bool, Bytes> probe;
        {
            ultra::timing::ScopedTimer _profile_(
                ultra::timing::globalDecoderProfile().ofdm_cw0_probe_decode);
            probe = frame_decoder_.codec_->decode(cw0_r14);
        }
        auto [ok_r14, data_r14] = probe;
        size_t bpc_r14 = v2::getBytesPerCodeword(CodeRate::R1_4);

        if (ok_r14 && data_r14.size() >= 2
            && data_r14[0] == 0x55 && data_r14[1] == 0x4C) {
            if (data_r14.size() > bpc_r14) data_r14.resize(bpc_r14);
            auto hdr_r14 = v2::parseHeader(data_r14);
            if (hdr_r14.valid && hdr_r14.total_cw == 1) {
                LOG_MODEM(INFO, "[%s] R1/4 control fast-path OK", log_prefix_.c_str());
                result.success = true;
                result.codewords_ok = 1;
                result.frame_data = data_r14;
                result.frame_type = hdr_r14.type;
                return result;
            }
        }
        // Restore rate for remaining decode paths
        frame_decoder_.codec_->setRate(rate);
    }

    // Step 1: Try to decode CW0 RAW (no channel deinterleave)
    // Control frames (ACK etc.) are never channel-interleaved, so probe without it.
    // If this is a fixed-CW data frame (which IS interleaved), CW0 will likely fail here
    // and we'll fall through to decodeFixedFrame() which handles deinterleaving internally.
    //
    // Skip the probe entirely when we already know it's a fixed-CW interleaved data frame:
    //   (a) prior CW0 peek set pending_total_cw_ to a fixed-frame CW count, or
    //   (b) burst-interleave marker was latched (frame is part of a burst-interleaved
    //       group, definitely fixed-CW data, never a control frame).
    // This was identified as redundant work in profiling plan v5 + ChatGPT review.
    std::pair<bool, Bytes> raw_probe;
    bool ok0 = false;
    Bytes data0;
    if (known_fixed_cw) {
        ultra::timing::globalDecoderProfile()
            .raw_cw0_probe_skipped.fetch_add(1, std::memory_order_relaxed);
        // Fall through directly to frame-interleaved decode below.
    } else {
        std::vector<float> cw0_bits(soft_bits.begin(), soft_bits.begin() + LDPC_BLOCK);
        {
            ultra::timing::ScopedTimer _profile_(
                ultra::timing::globalDecoderProfile().ofdm_cw0_probe_decode);
            raw_probe = frame_decoder_.codec_->decode(cw0_bits);
        }
        ok0 = raw_probe.first;
        data0 = std::move(raw_probe.second);
    }

    bool try_frame_interleave = known_fixed_cw;  // skip probe → go straight to interleaved decode
    int probed_fixed_cw_count = 0;

    if (ok0 && data0.size() >= 2 && data0[0] == 0x55 && data0[1] == 0x4C) {
        // CW0 decoded and has valid magic - check if it's a control frame
        if (data0.size() > bytes_per_cw) data0.resize(bytes_per_cw);

        auto hdr = v2::parseHeader(data0);
        if (hdr.valid) {
            result.frame_type = hdr.type;

            if (hdr.total_cw == 1) {
                // === Control frame (1 CW) - no frame interleaving ===
                LOG_MODEM(DEBUG, "[%s] Control frame decoded (1 CW)", log_prefix_.c_str());
                result.success = true;
                result.codewords_ok = 1;
                result.frame_data = data0;
                return result;
            } else if (isFixedFrameCwCount(hdr.total_cw)) {
                // === Fixed-CW frame - try frame interleaving ===
                LOG_MODEM(DEBUG, "[%s] Header shows %d fixed CWs - trying frame deinterleave",
                          log_prefix_.c_str(), hdr.total_cw);
                probed_fixed_cw_count = hdr.total_cw;
                try_frame_interleave = true;
            } else {
                // === Larger multi-CW frame - old/variable format, no frame interleaving ===
                // Fall through to legacy decode path
                LOG_MODEM(DEBUG, "[%s] Multi-CW frame (%d CWs) - legacy decode", log_prefix_.c_str(), hdr.total_cw);
            }
        }
    } else {
        // CW0 decode failed or invalid magic - might be interleaved
        LOG_MODEM(DEBUG, "[%s] CW0 decode failed/invalid - trying frame deinterleave", log_prefix_.c_str());
        try_frame_interleave = true;
    }

    // Step 2: Try frame-interleaved decode if needed
    if (try_frame_interleave) {
        int decode_cw_count = pending_fixed_cw
            ? pending_total_cw_
            : (probed_fixed_cw_count > 0)
                ? probed_fixed_cw_count
            : fixed_frame_codewords_;
        size_t bps = static_cast<size_t>(ofdm_data_carriers_) * getBitsPerSymbol(current_modulation_);
        const int active_lifting_z = activeBurstLiftingZ();
        const int active_codeword_bits =
            active_lifting_z == 81 ? 1944 : v2::LDPC_CODEWORD_BITS;

        auto discoverFixedFrameCwCount = [&](int preferred_cw) -> int {
            std::vector<int> candidates;
            auto addCandidate = [&](int cw) {
                if (!isFixedFrameCwCount(cw)) {
                    return;
                }
                if (std::find(candidates.begin(), candidates.end(), cw) == candidates.end()) {
                    candidates.push_back(cw);
                }
            };

            addCandidate(preferred_cw);
            for (int cw = preferred_cw - 1; cw >= v2::kMinFixedFrameCodewords; --cw) {
                addCandidate(cw);
            }
            for (int cw = preferred_cw + 1; cw <= v2::kMaxFixedFrameCodewords; ++cw) {
                addCandidate(cw);
            }

            const size_t bytes_per_fixed_cw =
                v2::getBytesPerCodewordZ(rate, active_lifting_z);
            for (int candidate_cw : candidates) {
                const size_t frame_bits =
                    static_cast<size_t>(fec::FrameInterleaver::totalFrameBits(
                        candidate_cw, active_codeword_bits));
                if (soft_bits.size() < frame_bits) {
                    continue;
                }

                std::vector<float> cw0_bits;
                try {
                    auto cw_soft = fec::FrameInterleaver::deinterleave(
                        soft_bits, candidate_cw, active_codeword_bits);
                    if (cw_soft.empty() ||
                        cw_soft[0].size() < static_cast<size_t>(active_codeword_bits)) {
                        continue;
                    }
                    cw0_bits = std::move(cw_soft[0]);
                } catch (const std::exception&) {
                    continue;
                }

                if (apply_channel_deinterleave) {
                    // Z-aware interleaver block size from the active BURST_HEADER
                    // descriptor (single RX source of truth — activeBurstLiftingZ()).
                    // Was env-only: a latent mismatch if a Z=81 descriptor arrived
                    // without ULTRA_LDPC_Z set.
                    ChannelInterleaver channel_deinterleaver(
                        bps, static_cast<size_t>(active_codeword_bits));
                    cw0_bits = channel_deinterleaver.deinterleave(cw0_bits);
                }

                auto [peek_ok, peek_data] = robustDecodeSingleCW(
                    cw0_bits.data(), cw0_bits.size(), rate, log_prefix_.c_str(),
                    ultra::timing::SingleCWCallSite::Cw0Peek,
                    /*max_retries=*/4, active_lifting_z);
                if (!peek_ok || peek_data.size() < bytes_per_fixed_cw) {
                    continue;
                }
                if (peek_data.size() > bytes_per_fixed_cw) {
                    peek_data.resize(bytes_per_fixed_cw);
                }

                auto hdr = v2::parseHeader(peek_data);
                if (!hdr.valid || hdr.is_control || !isFixedFrameCwCount(hdr.total_cw)) {
                    continue;
                }

                if (hdr.total_cw != candidate_cw) {
                    LOG_MODEM(WARN, "[%s] Fixed-frame CW discovery rejected mismatch: candidate=%d header=%d",
                              log_prefix_.c_str(), candidate_cw, hdr.total_cw);
                    continue;
                }

                LOG_MODEM(INFO, "[%s] Fixed-frame CW discovery: header total_cw=%d",
                          log_prefix_.c_str(), hdr.total_cw);
                return hdr.total_cw;
            }

            return 0;
        };

        if (fixed_frame_header_discovery_ && probed_fixed_cw_count == 0) {
            const int header_cw_count = discoverFixedFrameCwCount(decode_cw_count);
            if (header_cw_count > 0 && header_cw_count != decode_cw_count) {
                LOG_MODEM(INFO, "[%s] Fixed-frame CW discovery selected %d CWs (was %d)",
                          log_prefix_.c_str(), header_cw_count, decode_cw_count);
                decode_cw_count = header_cw_count;
                pending_total_cw_ = header_cw_count;
            }
        }

        size_t frame_interleave_bits =
            static_cast<size_t>(fec::FrameInterleaver::totalFrameBits(
                decode_cw_count, active_codeword_bits));
        if (soft_bits.size() < frame_interleave_bits) {
            return result;
        }
        LOG_MODEM(DEBUG, "[%s] Attempting %d-CW frame deinterleave decode",
                  log_prefix_.c_str(), decode_cw_count);

        // Use v2::decodeFixedFrame which handles frame + channel deinterleaving + LDPC decode
        // Channel deinterleaving restores the original bit order within each CW
        // Only enable for OFDM modes (MC-DPSK doesn't use channel interleaving)
        bool built_key_provisional = false;  // set by buildHarqKey's fallback path
        bool harq_key_intentionally_ineligible = false;
        std::optional<fec::SoftCombineBuffer::Key> provisional_key_used;
        auto validateHarqPrediction =
            [&](uint32_t actual_sender_hash, uint32_t actual_dst_hash,
                uint16_t actual_seq,
                int logical_index, bool count_mismatch) -> bool {
            if (logical_index < 0 || !burst_harq_ctx_pulled_ ||
                !burst_harq_ctx_ || !burst_harq_ctx_->valid() ||
                static_cast<size_t>(logical_index) >=
                    burst_harq_ctx_->predicted_seqs.size()) {
                return false;
            }
            const uint16_t predicted_seq =
                burst_harq_ctx_->predicted_seqs[static_cast<size_t>(logical_index)];
            const bool matches =
                streaming_decode_policy::provisionalHarqPredictionMatchesHeader(
                    burst_harq_ctx_->sender_hash, burst_harq_ctx_->dst_hash,
                    predicted_seq, actual_sender_hash, actual_dst_hash,
                    actual_seq);
            if (!matches) {
                if (!burst_harq_prediction_invalid_) {
                    burst_harq_prediction_invalid_ = true;
                    if (count_mismatch) {
                        ultra::timing::globalDecoderProfile()
                            .harq_prediction_mismatch.fetch_add(
                                1, std::memory_order_relaxed);
                    }
                    LOG_MODEM(INFO,
                              "[%s] HARQ prediction mismatch at pos=%d: "
                              "predicted src=0x%06X dst=0x%06X seq=%u, "
                              "actual src=0x%06X dst=0x%06X seq=%u — "
                              "provisional keys disabled for this group",
                              log_prefix_.c_str(), logical_index,
                              burst_harq_ctx_->sender_hash,
                              burst_harq_ctx_->dst_hash, predicted_seq,
                              actual_sender_hash, actual_dst_hash, actual_seq);
                }
                return false;
            }
            burst_harq_prediction_validated_ = true;
            return true;
        };
        auto buildHarqKey = [&](int cw_count, fec::SoftCombineBuffer::Key& out_key) -> bool {
            if (!harq_buffer_ || !harq_buffer_->enabled()) {
                return false;
            }

            cw_count = v2::sanitizeFixedFrameCodewords(cw_count);
            const size_t frame_bits =
                static_cast<size_t>(fec::FrameInterleaver::totalFrameBits(
                    cw_count, active_codeword_bits));
            if (soft_bits.size() < frame_bits) {
                return false;
            }

            auto cw_soft = fec::FrameInterleaver::deinterleave(
                soft_bits, cw_count, active_codeword_bits);
            if (cw_soft.empty() ||
                cw_soft[0].size() < static_cast<size_t>(active_codeword_bits)) {
                return false;
            }

            auto fillKey = [&](uint32_t sender_hash, uint32_t dst_hash,
                               uint16_t seq, int total_cw,
                               bool physical_burst_end) -> bool {
                fec::SoftCombineBuffer::HarqKeyInputs ki;
                ki.sender_hash = sender_hash;
                ki.dst_hash = dst_hash;
                ki.seq = seq;
                ki.rate = rate;
                ki.cw_count = static_cast<uint8_t>(v2::sanitizeFixedFrameCodewords(total_cw));
                ki.lifting_z = active_lifting_z;
                ki.modulation = current_modulation_;
                ki.channel_interleave = apply_channel_deinterleave;
                ki.physical_burst_end = physical_burst_end;
                ki.waveform_mode = static_cast<int>(mode_);
                ki.ofdm_data_carriers = ofdm_data_carriers_;
                out_key = fec::SoftCombineBuffer::makeKey(ki);
                return out_key.sender_hash != 0;
            };

            std::vector<float> cw0_bits = std::move(cw_soft[0]);
            if (apply_channel_deinterleave) {
                // Z-aware interleaver block size from the active BURST_HEADER
                // descriptor (single RX source of truth — activeBurstLiftingZ()).
                ChannelInterleaver channel_deinterleaver(
                    bps, static_cast<size_t>(active_codeword_bits));
                cw0_bits = channel_deinterleaver.deinterleave(cw0_bits);
            }

            static const bool kProvisionalEnabled = [] {
                const char* e = std::getenv("ULTRA_HARQ_PROVISIONAL");
                return e && *e == '1';  // opt-IN
            }();
            // Exact DESCRIPTOR provenance only. A late-join group has a known
            // declared size but not an independently proven absolute head
            // position, so it is intentionally outside this experiment.
            const bool exact_descriptor_proven =
                burst_arm_provenance_ == BurstArmProvenance::DESCRIPTOR;
            const bool provisional_harq_context_allowed =
                streaming_decode_policy::allowProvisionalHarqContext(
                    kProvisionalEnabled, burst_transport_rx_,
                    exact_descriptor_proven, use_burst_interleave_,
                    burst_harq_cadence_blocked_,
                    burst_harq_prediction_invalid_,
                    static_cast<bool>(harq_context_callback_),
                    current_modulation_, rate, cw_count, active_lifting_z,
                    burst_logical_index_, burst_group_size_);
            // Pull before the first CW0 peek, not only after a failure. Every
            // earlier header-verified position can then falsify a stale ARQ
            // mirror before a later CW0-dead position would use it as a key.
            if (provisional_harq_context_allowed && !burst_harq_ctx_pulled_) {
                burst_harq_ctx_ = harq_context_callback_();
                burst_harq_ctx_pulled_ = true;
                LOG_MODEM(DEBUG,
                          "[%s] [HARQKEY] ctx pulled: have=%d valid=%d pred_n=%zu "
                          "src=0x%06X dst=0x%06X desc_src=0x%06X",
                          log_prefix_.c_str(), burst_harq_ctx_ ? 1 : 0,
                          (burst_harq_ctx_ && burst_harq_ctx_->valid()) ? 1 : 0,
                          burst_harq_ctx_ ? burst_harq_ctx_->predicted_seqs.size()
                                          : size_t{0},
                          burst_harq_ctx_ ? burst_harq_ctx_->sender_hash : 0u,
                          burst_harq_ctx_ ? burst_harq_ctx_->dst_hash : 0u,
                          last_burst_src_hash_);
            }

            auto [peek_ok, peek_data] = robustDecodeSingleCW(
                cw0_bits.data(), cw0_bits.size(), rate, log_prefix_.c_str(),
                ultra::timing::SingleCWCallSite::Cw0Peek,
                /*max_retries=*/4, active_lifting_z);
            const size_t bytes_per_fixed_cw =
                v2::getBytesPerCodewordZ(rate, active_lifting_z);
            if (!peek_ok || peek_data.size() < bytes_per_fixed_cw) {
                // CW0 undecodable -> no header-verified key. RESTRICTED provisional
                // fallback (2026-07-01, design-review verdict — fable_analysis/09
                // §3.4): key by the receiver's ARQ-mirror-predicted seq for this
                // burst position so the resend can chase-combine. Blind
                // receive-order guesses stay rejected (the original objection
                // stands); the mirror is exact whenever the sender acted on our
                // last SACK, and the gates + the finalize seq-match guard bound
                // every divergence case. The experiment is restricted to the two
                // measured constant-envelope profiles (QPSK R3/4 and 8PSK R2/3
                // cw4/Z81), descriptor-proven, non-interleaved, and excludes the
                // final physical member (whose hidden PHYSICAL_BURST_END bit changes
                // the protected DATA bytes). Warm cadence excludes escalated timeout
                // batches, prediction mismatch disables the rest of the group, and
                // the callback supplies both session endpoint identities.
                // DEFAULT-OFF (2026-07-01 rig verdict, flipped the same evening it
                // shipped): on live IONOS MPG@20 the provisional path POISON-LOOPED —
                // the stuck group re-failed 0/8 with 285 combines feeding it, while
                // the ULTRA_HARQ_PROVISIONAL=0 falsification run made forward
                // progress on the same channel. Mechanism: real-rig first-attempt
                // fade LLRs are confidently-WRONG (not sim's near-zero nulls), the
                // combine path REPLACES fresh LLRs with the sum (no standalone
                // fallback, frame_v2.cpp decode loop), and mismatch=0 cannot
                // exonerate an 0/N loop (detection requires a decode). Sim measured
                // 0/212 mispredictions — the divergence is the LLR character, a
                // sim-fidelity gap. The structural fix now exists:
                // combine-then-fail retries the CW standalone (fresh-only), and a
                // provisional double-fail retains fresh rather than a suspect sum.
                // The knob remains opt-in until a matched real-rig A/B proves value.
                // Gate trace ([HARQKEY]): three gate designs in a row measured
                // provisional=0 — never debug this blind again.
                LOG_MODEM(DEBUG,
                          "[%s] [HARQKEY] cw0-peek FAIL pos=%d gates: en=%d cadence_block=%d "
                          "invalid=%d validated=%d burst=%d exact_desc=%d interleave=%d mod=%d "
                          "rate=%d group=%d cb=%d",
                          log_prefix_.c_str(), burst_logical_index_,
                          kProvisionalEnabled ? 1 : 0,
                          burst_harq_cadence_blocked_ ? 1 : 0,
                          burst_harq_prediction_invalid_ ? 1 : 0,
                          burst_harq_prediction_validated_ ? 1 : 0,
                          burst_transport_rx_ ? 1 : 0,
                          exact_descriptor_proven ? 1 : 0,
                          use_burst_interleave_ ? 1 : 0,
                          static_cast<int>(current_modulation_),
                          static_cast<int>(rate), burst_group_size_,
                          harq_context_callback_ ? 1 : 0);
                if (streaming_decode_policy::allowProvisionalHarqKey(
                        provisional_harq_context_allowed,
                        burst_harq_prediction_validated_, burst_logical_index_,
                        burst_group_size_)) {
                    // Source and destination hashes come from the SESSION context
                    // (Connection remote/local calls). A descriptor-side src cross-check
                    // was tried and REMOVED: the BURST_HEADER is a compact
                    // ControlFrame whose bytes parse differently from a data
                    // header, so the captured value never matched (measured
                    // desc_src=0x001505 vs session 0x536C71 -> vetoed every key).
                    if (streaming_decode_policy::
                            provisionalHarqContextCoversPosition(
                                burst_harq_ctx_ && burst_harq_ctx_->valid(),
                                burst_logical_index_,
                                burst_harq_ctx_
                                    ? burst_harq_ctx_->predicted_seqs.size()
                                    : size_t{0})) {
                        const uint16_t predicted_seq =
                            burst_harq_ctx_->predicted_seqs
                                [static_cast<size_t>(burst_logical_index_)];
                        if (fillKey(burst_harq_ctx_->sender_hash,
                                    burst_harq_ctx_->dst_hash, predicted_seq,
                                    cw_count, /*physical_burst_end=*/false)) {
                            built_key_provisional = true;
                            ultra::timing::globalDecoderProfile()
                                .harq_key_build_provisional.fetch_add(
                                    1, std::memory_order_relaxed);
                            LOG_MODEM(DEBUG,
                                      "[%s] HARQ provisional key: pos=%d seq=%u "
                                      "(CW0 peek failed; ARQ-mirror predicted)",
                                      log_prefix_.c_str(), burst_logical_index_,
                                      predicted_seq);
                            return true;
                        }
                    }
                }
                return false;
            }
            if (peek_data.size() > bytes_per_fixed_cw) {
                peek_data.resize(bytes_per_fixed_cw);
            }

            auto hdr = v2::parseHeader(peek_data);
            if (!hdr.valid || hdr.is_control || !isFixedFrameCwCount(hdr.total_cw)) {
                return false;
            }
            if (hdr.total_cw != cw_count) {
                LOG_MODEM(WARN,
                          "[%s] HARQ key rejected: header total_cw=%d does not match decode cw_count=%d",
                          log_prefix_.c_str(), hdr.total_cw, cw_count);
                return false;
            }

            // ULPAD is physical geometry, not an SR-ARQ member. It must not
            // validate or invalidate the ARQ mirror and has no useful HARQ
            // history of its own. Decode it fresh and keep all HARQ telemetry
            // and storage untouched.
            if (v2::isOFDMBurstPadHeader(hdr)) {
                harq_key_intentionally_ineligible = true;
                LOG_MODEM(DEBUG,
                          "[%s] [HARQKEY] physical ULPAD ignored at pos=%d "
                          "seq=%u dst=0x%06X",
                          log_prefix_.c_str(), burst_logical_index_, hdr.seq,
                          hdr.dst_hash);
                return false;
            }

            // Prefix consistency (provisional-key gate): a header-verified seq that
            // contradicts the position prediction proves the mirror is wrong for
            // this group (lost SACK / stale-ack divergence) — disable provisional
            // keys for the REMAINING positions.
            validateHarqPrediction(hdr.src_hash, hdr.dst_hash, hdr.seq,
                                   burst_logical_index_,
                                   /*count_mismatch=*/true);

            const bool physical_burst_end =
                (peek_data[3] & v2::Flags::PHYSICAL_BURST_END) != 0;
            return fillKey(hdr.src_hash, hdr.dst_hash, hdr.seq, hdr.total_cw,
                           physical_burst_end);
        };
        const auto _profile_fs_start_ = std::chrono::steady_clock::now();
        const bool turbo_data_fec = v2::ofdmDataFrameFec() == v2::DataFrameFec::TURBO;
        auto decodeFixed = [&](int cw_count) {
            if (turbo_data_fec) {
                // One turbo codeword over the whole frame (Stage 1). No CW0 peek / HARQ
                // key: there is no independently decodable codeword 0 to read a header
                // from. Same air bits, same status vector shape (see frame_v2.hpp).
                built_key_provisional = false;
                harq_key_intentionally_ineligible = true;
                provisional_key_used.reset();
                Bytes hard_info;
                auto st = v2::decodeTurboFixedFrame(soft_bits, rate, cw_count,
                                                    active_lifting_z, &hard_info);
                // Whole-frame chase combining. FRESH decode first (above), so a stored
                // history can never block a decode the fresh reception manages alone.
                const size_t air_bits =
                    static_cast<size_t>(v2::sanitizeFixedFrameCodewords(cw_count)) * 24u *
                    static_cast<size_t>(active_lifting_z);
                if (!harq_buffer_ || !harq_buffer_->enabled() || soft_bits.size() < air_bits) {
                    return st;
                }
                // Key from the decoded header on success, or from the failed decode's hard
                // decisions when their header passes its own HCRC (magic + 16-bit CRC).
                const Bytes& hsrc = st.allSuccess() ? st.data[0] : hard_info;
                if (hsrc.size() < v2::DataFrame::HEADER_SIZE) {
                    return st;
                }
                const auto hdr = v2::parseHeader(hsrc);
                if (!hdr.valid || hdr.is_control ||
                    hdr.total_cw != v2::sanitizeFixedFrameCodewords(cw_count) ||
                    v2::isOFDMBurstPadHeader(hdr)) {
                    return st;
                }
                fec::SoftCombineBuffer::HarqKeyInputs ki;
                ki.sender_hash = hdr.src_hash;
                ki.dst_hash = hdr.dst_hash;
                ki.seq = hdr.seq;
                ki.rate = rate;
                ki.cw_count = static_cast<uint8_t>(v2::sanitizeFixedFrameCodewords(cw_count));
                ki.cw_index = fec::SoftCombineBuffer::kWholeFrameCwIndex;
                ki.lifting_z = active_lifting_z;
                ki.modulation = current_modulation_;
                ki.channel_interleave = false;
                ki.physical_burst_end = (hsrc[3] & v2::Flags::PHYSICAL_BURST_END) != 0;
                ki.waveform_mode = static_cast<int>(mode_);
                ki.ofdm_data_carriers = ofdm_data_carriers_;
                ki.header_crc = static_cast<uint16_t>((static_cast<uint16_t>(hsrc[15]) << 8) | hsrc[16]);
                const auto key = fec::SoftCombineBuffer::makeKey(ki);
                if (st.allSuccess()) {
                    harq_buffer_->drop(key);
                    return st;
                }
                constexpr auto kTurboHarqSameReceptionMs = std::chrono::milliseconds(3000);
                const auto now = std::chrono::steady_clock::now();
                if (key == turbo_harq_last_key_ &&
                    now - turbo_harq_last_time_ < kTurboHarqSameReceptionMs) {
                    return st;  // same reception (timing retry) — never self-combine
                }
                turbo_harq_last_key_ = key;
                turbo_harq_last_time_ = now;
                std::vector<float> fresh(soft_bits.begin(),
                                         soft_bits.begin() + static_cast<std::ptrdiff_t>(air_bits));
                std::vector<float> combined;
                const int attempts = harq_buffer_->combine(key, fresh, combined);
                if (attempts > 1) {
                    auto st2 = v2::decodeTurboFixedFrame(combined, rate, cw_count, active_lifting_z);
                    if (st2.allSuccess()) {
                        const auto h2 = v2::parseHeader(st2.data[0]);
                        if (h2.valid && h2.seq == hdr.seq && h2.src_hash == hdr.src_hash &&
                            h2.dst_hash == hdr.dst_hash) {
                            harq_buffer_->drop(key);
                            std::fill(st2.harq_attempts.begin(), st2.harq_attempts.end(), attempts);
                            LOG_MODEM(INFO, "[%s] Turbo HARQ: seq=%u recovered from %d combined receptions",
                                      log_prefix_.c_str(), static_cast<unsigned>(hdr.seq), attempts);
                            return st2;
                        }
                    }
                    harq_buffer_->retain(key, std::move(combined));
                } else {
                    harq_buffer_->retain(key, std::move(fresh));
                }
                return st;
            }
            fec::SoftCombineBuffer::Key harq_key;
            fec::SoftCombineBuffer::Key* harq_key_ptr = nullptr;
            fec::SoftCombineBuffer* harq_buffer = nullptr;
            built_key_provisional = false;
            harq_key_intentionally_ineligible = false;
            provisional_key_used.reset();
            // Only count when HARQ is actually wanted (buffer enabled).
            // A "failed key build" only matters if HARQ would otherwise
            // have engaged — counting when HARQ is off would make every
            // MC-DPSK or non-fixed-frame decode look like a HARQ miss.
            const bool harq_active = harq_buffer_ && harq_buffer_->enabled();
            if (buildHarqKey(cw_count, harq_key)) {
                harq_key_ptr = &harq_key;
                harq_buffer = harq_buffer_;
                if (built_key_provisional) {
                    provisional_key_used = harq_key;
                }
                if (harq_active && !built_key_provisional) {
                    ultra::timing::globalDecoderProfile()
                        .harq_key_build_success.fetch_add(
                            1, std::memory_order_relaxed);
                }
            } else if (harq_active && !harq_key_intentionally_ineligible) {
                // Codex review #17: HARQ misses every frame whose
                // first-attempt CW0 fails. Count these so we can
                // measure whether a session-context fallback key
                // would actually move the needle.
                ultra::timing::globalDecoderProfile()
                    .harq_key_build_failed.fetch_add(
                        1, std::memory_order_relaxed);
            }
            // LDPC lifting Z sourced from BURST_HEADER payload[5] (cached on
            // sync_controller_.last_burst_descriptor_.lifting_z). When the sender announced Z=81,
            // the data group's codewords are N=1944 and the decoder must match;
            // falls back to Z=27 outside a burst. Single RX source of truth.
            const int ldpc_z = active_lifting_z;
            return v2::decodeFixedFrame(soft_bits, rate, cw_count,
                                        apply_channel_deinterleave, bps,
                                        harq_buffer, harq_key_ptr, ldpc_z,
                                        built_key_provisional);
        };
        auto cw_status = decodeFixed(decode_cw_count);

        auto validateRecoveredProvisionalHeader =
            [&](const v2::CodewordStatus& status) {
            if (!provisional_key_used || status.decoded.empty() ||
                status.data.empty() || !status.decoded[0] ||
                status.data[0].empty()) {
                return;
            }
            const auto& cw0 = status.data[0];
            const auto hdr = v2::parseHeader(cw0);
            if (v2::isOFDMBurstPadHeader(hdr)) {
                LOG_MODEM(INFO,
                          "[%s] HARQ recovered physical ULPAD at pos=%d — "
                          "ignored by ARQ prediction; provisional storage suppressed",
                          log_prefix_.c_str(), burst_logical_index_);
                return;
            }
            const bool actual_tail =
                cw0.size() > 3 &&
                (cw0[3] & v2::Flags::PHYSICAL_BURST_END) != 0;
            const bool identity_matches =
                hdr.valid && !hdr.is_control &&
                fec::SoftCombineBuffer::provisionalHeaderIdentityMatchesKey(
                    *provisional_key_used, hdr.src_hash, hdr.dst_hash, hdr.seq,
                    hdr.total_cw, actual_tail);
            if (!identity_matches) {
                if (!burst_harq_prediction_invalid_) {
                    burst_harq_prediction_invalid_ = true;
                    // decodeFixedFrame's finalize guard owns the metric for a
                    // recovered provisional header; do not double-count here.
                    LOG_MODEM(INFO,
                              "[%s] HARQ recovered-header identity mismatch at "
                              "pos=%d — provisional keys disabled for this group",
                              log_prefix_.c_str(), burst_logical_index_);
                }
                return;
            }
            burst_harq_prediction_validated_ = true;
        };
        validateRecoveredProvisionalHeader(cw_status);

        auto headerCwCount = [](const v2::CodewordStatus& status) -> int {
            if (status.decoded.empty() || status.data.empty() ||
                !status.decoded[0] || status.data[0].empty()) {
                return 0;
            }

            auto hdr = v2::parseHeader(status.data[0]);
            if (hdr.valid && !hdr.is_control && isFixedFrameCwCount(hdr.total_cw)) {
                return hdr.total_cw;
            }
            return 0;
        };

        const int header_cw_count = headerCwCount(cw_status);
        if (header_cw_count > 0 && header_cw_count != decode_cw_count) {
            const size_t header_frame_bits =
                static_cast<size_t>(fec::FrameInterleaver::totalFrameBits(
                    header_cw_count, active_codeword_bits));
            if (soft_bits.size() >= header_frame_bits) {
                LOG_MODEM(INFO, "[%s] Fixed-frame header says %d CWs; retrying decode (was %d)",
                          log_prefix_.c_str(), header_cw_count, decode_cw_count);
                decode_cw_count = header_cw_count;
                cw_status = decodeFixed(decode_cw_count);
                validateRecoveredProvisionalHeader(cw_status);
            }
        }
        copyCodewordDiagnostics(result, cw_status);
        if (turbo_data_fec && !cw_status.allSuccess() && waveform_) {
            // chi ~ Exp(1) under H0 (no aligned frame): P(chi >= T) = e^-T. T = ln(1e3)
            // => 1e-3 chance of calling noise a real frame (a probability constant).
            constexpr float kFrameEvidenceChi = 6.9077553f;  // ln(1000)
            result.frame_evidence = waveform_->getLastLTSPairChi() >= kFrameEvidenceChi;
        }

        // Record duration into failed_4cw_after_peek if this attempt failed.
        // Note: this duration also counts inside decode_fixed_frame_total
        // (and ldpc_cw_total inside that) — it's an approximate subset.
        if (!cw_status.allSuccess()) {
            const uint64_t _profile_fs_us_ = static_cast<uint64_t>(
                std::chrono::duration_cast<std::chrono::microseconds>(
                    std::chrono::steady_clock::now() - _profile_fs_start_).count());
            ultra::timing::globalDecoderProfile()
                .failed_4cw_after_peek.addSample(_profile_fs_us_);
        }

        result.codewords_ok = 0;
        result.codewords_failed = 0;
        for (size_t i = 0; i < cw_status.decoded.size(); ++i) {
            if (cw_status.decoded[i]) {
                result.codewords_ok++;
            } else {
                result.codewords_failed++;
            }
        }

        if (cw_status.allSuccess()) {
            result.success = true;
            result.frame_data = cw_status.reassemble();

            if (result.frame_data.empty()) {
                // LDPC said all CWs were OK but reassemble failed — likely LDPC false positive
                result.success = false;
                LOG_MODEM(WARN, "[%s] Frame deinterleave: %d/%zu CWs OK but reassemble FAILED (LDPC false positive?)",
                          log_prefix_.c_str(), result.codewords_ok, cw_status.decoded.size());
            } else {
                // Parse header to get frame type
                if (result.frame_data.size() >= 3) {
                    result.frame_type = static_cast<v2::FrameType>(result.frame_data[2]);
                }

                // ULTRA_ITERATIVE_CHEST (default OFF): this frame is now VERIFIED —
                // every codeword satisfied its LDPC parity check AND the frame CRC
                // deserialised (decodeFixedFrame's own false-positive block nulls
                // decoded[] otherwise, and reassemble() returns empty on a bad
                // header). So we know EXACTLY what the transmitter sent, and
                // re-running the production encoder on the reassembled bytes
                // reproduces its air stream bit-for-bit: encodeFixedFrame pads to
                // the same length with the same deterministic PRBS, so no pad
                // bookkeeping is needed. Anything less than all-CWs-good is
                // deliberately NOT fed back — a partially decoded frame determines
                // ZERO 16QAM carriers anyway (FrameInterleaver spreads each carrier
                // across all four codewords), and an unverified codeword is
                // confident poison for the estimate.
                if (data_aided_chest_enabled_ && burst_logical_index_ >= 0 &&
                    !cw_status.usedAnyPerturbation()) {
                    const int ldpc_z_da = activeBurstLiftingZ();
                    result.data_aided_air_bytes = turbo_data_fec
                        ? v2::encodeTurboFixedFrame(result.frame_data, rate,
                                                    decode_cw_count, ldpc_z_da)
                        : v2::encodeFixedFrame(result.frame_data, rate, decode_cw_count,
                                               apply_channel_deinterleave, bps, ldpc_z_da);
                }
            }

            LOG_MODEM(INFO, "[%s] Frame deinterleave decode SUCCESS (%d/%zu CWs, data=%zu bytes)",
                      log_prefix_.c_str(), result.codewords_ok, cw_status.decoded.size(),
                      result.frame_data.size());
            return result;
        } else {
            LOG_MODEM(DEBUG, "[%s] Frame deinterleave decode FAILED (%d/%zu CWs)",
                      log_prefix_.c_str(), result.codewords_ok, cw_status.decoded.size());

            // Step 2b: If frame deinterleave failed, check if it's a 1-CW control frame
            // Catches ACK frames that were escalated to fixed-frame decode by a failed peek
            // Try R1/4 first (control frames hardened), then code_rate_ fallback
            {
                auto trySalvage = [&](CodeRate sr) -> bool {
                    size_t bpc = v2::getBytesPerCodeword(sr);
                    auto [rec_ok, rec_data] = robustDecodeSingleCW(
                        soft_bits.data(), LDPC_BLOCK, sr, log_prefix_.c_str());
                    if (rec_ok && rec_data.size() >= 2
                        && rec_data[0] == 0x55 && rec_data[1] == 0x4C) {
                        if (rec_data.size() > bpc) rec_data.resize(bpc);
                        auto hdr = v2::parseHeader(rec_data);
                        if (hdr.valid && hdr.total_cw == 1) {
                            g_salvage_hits.fetch_add(1, std::memory_order_relaxed);
                            LOG_MODEM(INFO, "[%s] Salvaged 1-CW control (rate=%d, total_hits=%d)",
                                      log_prefix_.c_str(), static_cast<int>(sr),
                                      g_salvage_hits.load(std::memory_order_relaxed));
                            result.success = true;
                            result.codewords_ok = 1;
                            result.codewords_failed = 0;
                            result.frame_data = rec_data;
                            result.frame_type = hdr.type;
                            return true;
                        }
                    }
                    return false;
                };

                if (trySalvage(CodeRate::R1_4))
                    return result;
                if (rate != CodeRate::R1_4 && trySalvage(rate))
                    return result;
            }
        }
    }

    // Step 3: Legacy decode path (non-interleaved multi-CW frames)
    // This handles old-format frames or when frame interleaving is disabled
    if (ok0 && data0.size() >= 2 && data0[0] == 0x55 && data0[1] == 0x4C) {
        if (data0.size() > bytes_per_cw) data0.resize(bytes_per_cw);
        result.codewords_ok = 1;

        auto hdr = v2::parseHeader(data0);
        if (!hdr.valid) return result;

        result.frame_type = hdr.type;
        int total_cw = hdr.total_cw;
        int avail_cw = static_cast<int>(soft_bits.size() / LDPC_BLOCK);
        const bool variable_ofdm_frame =
            is_ofdm && total_cw > v2::kMaxFixedFrameCodewords;

        if (avail_cw < total_cw) {
            result.frame_data = data0;
            return result;
        }

        v2::CodewordStatus cw_status;
        cw_status.decoded.resize(total_cw, false);
        cw_status.data.resize(total_cw);
        cw_status.decoded[0] = true;
        cw_status.data[0] = data0;

        for (int i = 1; i < total_cw; i++) {
            size_t off = i * LDPC_BLOCK;
            std::vector<float> bits(soft_bits.begin() + off, soft_bits.begin() + off + LDPC_BLOCK);
            if (!variable_ofdm_frame) {
                bits = deinterleave_cw(bits);
            }

            auto [ok, data] = frame_decoder_.codec_->decode(bits);
            if (ok && data.size() >= bytes_per_cw) {
                data.resize(bytes_per_cw);
                cw_status.decoded[i] = true;
                cw_status.data[i] = data;
                result.codewords_ok++;
            } else {
                result.codewords_failed++;
            }
        }

        if (cw_status.allSuccess()) {
            result.success = true;
            result.frame_data = cw_status.reassemble();
        }
    }

    return result;
}

} // namespace gui
} // namespace ultra
