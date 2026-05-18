// StreamingDecoder module

#include "streaming_decoder.hpp"
#include "streaming_buffer_policy.hpp"
#include "streaming_decode_policy.hpp"
#include "streaming_frame_policy.hpp"
#include "streaming_signal_policy.hpp"
#include "gui/startup_trace.hpp"
#include "waveform/mc_dpsk_waveform.hpp"
#include "waveform/ofdm_cox_waveform.hpp"
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
#include <exception>
#include <fstream>
#include <stdexcept>

namespace ultra {
namespace gui {

namespace v2 = protocol::v2;
namespace buffer_policy = streaming_buffer_policy;
namespace decode_policy = streaming_decode_policy;
namespace frame_policy = streaming_frame_policy;
namespace signal_policy = streaming_signal_policy;

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
    int max_retries = 4)
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
    };
    static thread_local CachedDecoder cache;
    if (!cache.decoder || cache.rate != rate) {
        cache.decoder = std::make_unique<LDPCDecoder>(rate);
        cache.rate = rate;
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

void StreamingDecoder::decodeCurrentFrame() {
    if (!waveform_) {
        {
            std::lock_guard<std::mutex> lock(buffer_mutex_);
            correlation_pos_ = wrapRingIndexLocked(sync_position_ + 4800);
        }
        state_ = DecoderState::SEARCHING;
        return;
    }

    const uint32_t gen_at_start = reset_generation_.load(std::memory_order_acquire);
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
    const bool burst_latched = use_burst_interleave_ && waveform_ && waveform_->wasBurstInterleaved();
    const size_t pending_samples = pending_total_cw_ > 0
        ? static_cast<size_t>(waveform_->getMinSamplesForCWCount(pending_total_cw_))
        : 0;
    const size_t ofdm_control_samples = (is_ofdm && connected_)
        ? getOFDMControlFrameSamplesForCurrentMode()
        : 0;
    const size_t full_frame_samples = (is_ofdm && connected_)
        ? static_cast<size_t>(waveform_->getMinSamplesForCWCount(fixed_frame_codewords_))
        : 0;
    const size_t control_frame_samples =
        static_cast<size_t>(waveform_->getMinSamplesForControlFrame());
    auto requirement = decode_policy::selectDecodeSampleRequirement(
        pending_total_cw_,
        is_ofdm,
        connected_,
        use_burst_interleave_,
        burst_latched,
        pending_samples,
        ofdm_control_samples,
        full_frame_samples,
        control_frame_samples);
    size_t frame_len = requirement.samples;

    // Copy frame samples from buffer
    std::vector<float> frame_buffer;
    size_t frame_sync_abs = 0;
    {
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        frame_sync_abs = ringPosToAbsoluteLocked(sync_position_);

        size_t available;
        if (write_pos_ >= sync_position_) {
            available = write_pos_ - sync_position_;
        } else {
            available = buffer_capacity_samples_ - sync_position_ + write_pos_;
        }

        frame_len = std::min(frame_len, available);

        frame_buffer.resize(frame_len);
        for (size_t i = 0; i < frame_len; i++) {
            frame_buffer[i] = buffer_[wrapRingIndexLocked(sync_position_ + i)];
        }
    }

    // CFO pre-correction: remove known CFO from raw samples so the entire
    // demodulation chain sees a clean, CFO-free signal.  The waveform/demodulator
    // is then told CFO=0 and only needs to handle small residuals.
    if (is_ofdm && !frame_buffer.empty()) {
        applyCFOPreCorrection(frame_buffer, sync_cfo_, frame_sync_abs);
    }

    // After pre-correction, tell waveform CFO=0 (already removed from samples).
    // For non-OFDM (MC-DPSK), no pre-correction — pass original sync_cfo_.
    const float decode_cfo = (is_ofdm && std::abs(pre_correction_cfo_) > 0.01f)
                              ? 0.0f : sync_cfo_;

    if (frame_buffer.empty()) {
        {
            std::lock_guard<std::mutex> lock(buffer_mutex_);
            correlation_pos_ = wrapRingIndexLocked(sync_position_ + 4800);
            setSearchFloorLocked(frame_sync_abs + 4800);
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

        std::lock_guard<std::mutex> lock(buffer_mutex_);
        correlation_pos_ = wrapRingIndexLocked(sync_position_ + advance);
        setSearchFloorLocked(frame_sync_abs + advance);
    };

    // PING is a disconnected MC-DPSK chirp-only presence probe. Do not run this
    // RMS heuristic for connected OFDM light-preamble frames: valid data/control
    // frames must be accepted or rejected by the demodulator + CRC/FEC path.
    const bool allow_ping_detection = !connected_ && mode_ == protocol::WaveformMode::MC_DPSK;
    const size_t mc_dpsk_symbol_samples =
        (allow_ping_detection && waveform_)
            ? static_cast<size_t>(std::max(1, waveform_->getSamplesPerSymbol()))
            : 512;
    const size_t ping_training_skip = 9 * mc_dpsk_symbol_samples;
    const size_t ping_check_samples = std::max(
        frame_policy::kPingRMSCheckSamples, 3 * mc_dpsk_symbol_samples);
    auto evaluatePingDecision = [&](bool ldpc_decode_succeeded,
                                    bool ldpc_magic_valid) {
        return frame_policy::evaluatePingFrame(
            frame_buffer.data(), frame_buffer.size(), ping_training_skip,
            ping_check_samples, sync_correlation_, sync_gap_error_samples_,
            ldpc_decode_succeeded, ldpc_magic_valid);
    };
    auto emitPingFrame = [&](const frame_policy::PingFrameDecision& ping_decision,
                             bool ldpc_attempted) {
        LOG_MODEM(INFO, "[%s] PING detected: path1=%d path2=%d ratio=%.3f "
                  "chirp_corr=%.3f gap_error=%.1f ldpc_attempted=%d "
                  "ldpc_ok=%s magic=%s "
                  "SNR=%.1f dB CFO=%.1f Hz",
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
                  sync_snr_, sync_cfo_);

        DecodeResult ping;
        ping.success = true;
        ping.is_ping = true;
        ping.frame_type = v2::FrameType::PING;
        ping.snr_db = sync_snr_;
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
            std::lock_guard<std::mutex> lock(buffer_mutex_);
            size_t min_frame = static_cast<size_t>(waveform_->getMinSamplesForFrame());
            correlation_pos_ = wrapRingIndexLocked(sync_position_ + min_frame);
            setSearchFloorLocked(frame_sync_abs + min_frame);
            last_decoded_sync_pos_ = sync_position_;
        }

        state_ = DecoderState::SEARCHING;
        return true;
    };

    if (allow_ping_detection) {
        // PATH 1: clean-cable/AWGN silence after training. LDPC inputs are set
        // true here only to keep the PATH 2 fallback disabled until LDPC has
        // actually been attempted below.
        const auto ping_decision = evaluatePingDecision(true, true);

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
    // Try demodulating as DQPSK R1/4 control before using data profile.
    // This protects ACK/NACK decode when data modulation is higher order.
    const bool first_pass_ofdm_peek = frame_policy::shouldRunControlFirstOFDMPeek(
        pending_total_cw_, is_ofdm, connected_, frame_len,
        getOFDMControlFrameSamplesForCurrentMode());
    if (first_pass_ofdm_peek) {
        constexpr size_t CONTROL_LDPC_BLOCK = v2::LDPC_CODEWORD_BITS;
        Modulation saved_mod = current_modulation_;
        CodeRate saved_rate = code_rate_;
        bool switched_profile = (saved_mod != Modulation::DQPSK || saved_rate != CodeRate::R1_4);

        if (switched_profile) {
            waveform_->configure(Modulation::DQPSK, CodeRate::R1_4);
        }

        waveform_->setFrequencyOffset(decode_cfo);
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

                        last_fading_index_.store(waveform_->getFadingIndex());

                        if (switched_profile) {
                            waveform_->configure(saved_mod, saved_rate);
                        }

                        {
                            std::lock_guard<std::mutex> lock(buffer_mutex_);
                            correlation_pos_ = wrapRingIndexLocked(sync_position_ + frame_len);
                            setSearchFloorLocked(frame_sync_abs + frame_len);
                            last_decoded_sync_pos_ = sync_position_;
                        }

                        LOG_MODEM(INFO, "[%s] OFDM control-profile decode SUCCESS (%s seq=%d)",
                                  log_prefix_.c_str(), v2::frameTypeToString(hdr.type), hdr.seq);
                        state_ = DecoderState::SEARCHING;
                        return;
                    }
                }
            }
        }

        if (switched_profile) {
            waveform_->configure(saved_mod, saved_rate);
        }
    }

    // Data frame - process audio to get soft bits
    waveform_->setFrequencyOffset(decode_cfo);

    auto decode_start = std::chrono::steady_clock::now();
    const int expected_codewords = expectedOFDMCodewordsForSamples(frame_buffer.size());
    bool ok = processWaveformForCodewords(
        SampleSpan(frame_buffer.data(), frame_buffer.size()), expected_codewords);

    if (!ok) {
        LOG_MODEM(DEBUG, "[%s] process() failed", log_prefix_.c_str());
        {
            std::lock_guard<std::mutex> lock(buffer_mutex_);
            correlation_pos_ = wrapRingIndexLocked(sync_position_ + frame_len);
            setSearchFloorLocked(frame_sync_abs + frame_len);
        }
        state_ = DecoderState::SEARCHING;
        return;
    }

    // If the LTS detector locked early on a marked burst group, the first
    // physical block is the one most likely to poison the deinterleaver. Retry
    // that block from the timing-slope-corrected origin before collecting LLRs.
    bool burst_marker = use_burst_interleave_ && connected_ && is_ofdm
                        && mode_ == protocol::WaveformMode::OFDM_CHIRP
                        && waveform_->wasBurstInterleaved();
    if (burst_marker) {
        const float burst_timing_offset = waveform_->getLastTimingOffsetSamples();
        constexpr float kBurstFrameRetryThreshold = 64.0f;
        constexpr float kBurstFrameRetryMax = 320.0f;
        if (std::abs(burst_timing_offset) >= kBurstFrameRetryThreshold &&
            std::abs(burst_timing_offset) <= kBurstFrameRetryMax) {
            const int sample_correction = static_cast<int>(std::lround(burst_timing_offset));
            const size_t corrected_sync_pos = wrapRingIndexLocked(
                sync_position_ + buffer_capacity_samples_ + sample_correction);
            const size_t corrected_sync_abs =
                (sample_correction >= 0)
                    ? frame_sync_abs + static_cast<size_t>(sample_correction)
                    : (frame_sync_abs > static_cast<size_t>(-sample_correction)
                           ? frame_sync_abs - static_cast<size_t>(-sample_correction)
                           : 0);

            bool have_corrected_frame = false;
            {
                std::lock_guard<std::mutex> lock(buffer_mutex_);
                size_t corrected_available;
                if (write_pos_ >= corrected_sync_pos) {
                    corrected_available = write_pos_ - corrected_sync_pos;
                } else {
                    corrected_available = buffer_capacity_samples_ - corrected_sync_pos + write_pos_;
                }
                have_corrected_frame = corrected_available >= frame_len;
                if (have_corrected_frame) {
                    frame_buffer.assign(frame_len, 0.0f);
                    for (size_t i = 0; i < frame_len; i++) {
                        frame_buffer[i] = buffer_[wrapRingIndexLocked(corrected_sync_pos + i)];
                    }
                }
            }

            if (have_corrected_frame) {
                applyCFOPreCorrection(frame_buffer, sync_cfo_, corrected_sync_abs);

                // The marker flag was consumed by the first process() call.
                // Normalize the first LTS symbol manually for this retry so
                // channel estimation sees two same-polarity training symbols.
                const size_t lts_sym_len = static_cast<size_t>(waveform_->getSamplesPerSymbol());
                for (size_t i = 0; i < lts_sym_len && i < frame_buffer.size(); ++i) {
                    frame_buffer[i] = -frame_buffer[i];
                }

                waveform_->setAbsoluteTrainingPosition(corrected_sync_abs);
                waveform_->setFrequencyOffset(decode_cfo);
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
            std::lock_guard<std::mutex> lock(buffer_mutex_);
            correlation_pos_ = wrapRingIndexLocked(sync_position_ + frame_len);
            setSearchFloorLocked(frame_sync_abs + frame_len);
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
            LOG_MODEM(INFO, "[%s] False chirp lock rejected: |llr|_avg=%.2f, "
                      "near_zero=%zu/%zu (%.1f%%), soft_bits=%zu — re-searching",
                      log_prefix_.c_str(), llr_quality.mean_abs,
                      llr_quality.near_zero_count, llr_quality.count,
                      llr_quality.near_zero_fraction * 100.0f, soft_bits.size());
            advancePastFalseOFDMLock();
            state_ = DecoderState::SEARCHING;
            return;
        }
    }

    LOG_MODEM(INFO, "[%s] Got %zu soft bits (%zu samples), proceeding to decode",
              log_prefix_.c_str(), soft_bits.size(), frame_buffer.size());

    last_fading_index_.store(waveform_->getFadingIndex());

    if (burst_marker) {
        LOG_MODEM(INFO, "[%s] Burst interleave marker detected, entering accumulation",
                  log_prefix_.c_str());

        // Initialize accumulation state with first frame's soft bits
        burst_soft_buffer_.clear();
        burst_metric_templates_.clear();
        burst_soft_buffer_.push_back(std::move(soft_bits));
        burst_min_block_ = static_cast<size_t>(
            waveform_->getMinSamplesForCWCount(fixed_frame_codewords_));
        burst_next_pos_ = wrapRingIndexLocked(sync_position_ + frame_len);
        burst_snr_ = sync_snr_;
        burst_cfo_ = sync_cfo_;
        burst_start_time_ = std::chrono::steady_clock::now();

        // Feed back CFO from first frame (add pre-correction amount back)
        const float residual_cfo = waveform_->estimatedCFO();
        DecodeResult first_metrics;
        populateDecodeMetrics(first_metrics, is_ofdm, residual_cfo);
        burst_metric_templates_.push_back(first_metrics);
        const float current_cfo = last_cfo_.load();
        const auto cfo_update = signal_policy::combinePilotCFO(
            pre_correction_cfo_, residual_cfo, current_cfo, /*clamp_drift=*/true);
        last_cfo_.store(cfo_update.accepted_cfo);
        burst_cfo_ = cfo_update.accepted_cfo;

        // LTS autocorrelation can lock early on later marked groups inside a
        // long burst. The marker frame is retried above; keep this as a guard
        // for continuation slicing if residual timing is still large.
        const float burst_timing_offset = waveform_->getLastTimingOffsetSamples();
        constexpr float kBurstTimingCorrectionThreshold = 80.0f;
        constexpr float kBurstMaxTimingCorrection = 320.0f;
        if (std::abs(burst_timing_offset) >= kBurstTimingCorrectionThreshold &&
            std::abs(burst_timing_offset) <= kBurstMaxTimingCorrection) {
            const int sample_correction = static_cast<int>(std::lround(burst_timing_offset));
            burst_next_pos_ = wrapRingIndexLocked(
                burst_next_pos_ + buffer_capacity_samples_ + sample_correction);
            LOG_MODEM(WARN, "[%s] Burst group timing correction: %.1f samples, next_pos=%zu",
                      log_prefix_.c_str(), burst_timing_offset, burst_next_pos_);
        }

        state_ = DecoderState::BURST_ACCUMULATING;
        return;  // processBuffer() will call accumulateBurstFrames() on next iteration
    }

    // Feed back pilot-corrected CFO to cached value (OFDM only).
    // MC-DPSK does not have pilot-based tracking; keep chirp-derived CFO.
    if (is_ofdm) {
        // After pre-correction, the demodulator sees near-zero CFO.
        // waveform_->estimatedCFO() returns the RESIDUAL (pre-correction error).
        // Add back the pre-correction amount to get the true total CFO.
        const float residual_cfo = waveform_->estimatedCFO();
        const float current_cfo = last_cfo_.load();
        const auto cfo_update = signal_policy::combinePilotCFO(
            pre_correction_cfo_, residual_cfo, current_cfo, connected_);
        if (cfo_update.clamped) {
            LOG_MODEM(WARN, "[%s] Pilot CFO drift clamped: %.2f → %.2f Hz (drift=%.2f, max=%.1f)",
                      log_prefix_.c_str(), current_cfo, cfo_update.unclamped_cfo,
                      cfo_update.drift_hz, signal_policy::kMaxPilotCFODriftHz);
        }

        if (std::abs(cfo_update.accepted_cfo - current_cfo) > 0.1f) {
            LOG_MODEM(INFO, "[%s] CFO updated: %.2f → %.2f Hz (pre_corr=%.2f + residual=%.2f)",
                      log_prefix_.c_str(), current_cfo, cfo_update.accepted_cfo,
                      pre_correction_cfo_, residual_cfo);
        }
        last_cfo_.store(cfo_update.accepted_cfo);
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
        codec_->setRate(rate);
        auto [peek_ok, peek_data] = codec_->decode(cw0);

        if (peek_ok && peek_data.size() >= 4 && peek_data[0] == 0x55 && peek_data[1] == 0x4C) {
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
    // For high-order OFDM_COX data modes, the robust control-sized window can
    // demap more than one full LDPC block while still being short of the fixed
    // frame. QAM16 R1/2 on 59 carriers produces exactly two CWs from the first
    // 10368-sample peek, so the old "< 2 CW" guard skipped escalation and the
    // fixed-frame decoder returned cw_ok=0/cw_fail=0 with insufficient bits.
    // ========================================================================
    const bool legacy_single_cw_peek =
        soft_bits.size() >= LDPC_BLOCK && soft_bits.size() < 2 * LDPC_BLOCK;
    const bool cox_subfixed_peek =
        mode_ == protocol::WaveformMode::OFDM_COX &&
        decode_policy::hasSubFixedFrameSoftBits(
            soft_bits.size(), fixed_frame_codewords_, LDPC_BLOCK);
    if (pending_total_cw_ == 0 && is_ofdm && connected_
        && (legacy_single_cw_peek || cox_subfixed_peek)) {

        // Large OFDM FILE_BLOCK frames use variable-CW encoding without the
        // fixed-frame interleaver. Give raw CW0 one chance to declare the
        // true frame length before defaulting to the fixed-frame path.
        codec_->setRate(rate);
        {
            ultra::timing::ScopedTimer _profile_(
                ultra::timing::globalDecoderProfile().ofdm_cw0_probe_decode);
            auto [peek_ok, peek_data] = codec_->decode(
                std::vector<float>(soft_bits.begin(), soft_bits.begin() + LDPC_BLOCK));
            const size_t bytes_per_cw = v2::getBytesPerCodeword(rate);
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
        const size_t llr_count = std::min(soft_bits.size(), LDPC_BLOCK);
        const float llr_abs_avg = signal_policy::meanAbsLLR(soft_bits.data(), llr_count);
        if (llr_abs_avg < signal_policy::kMinLLRForEscalation) {
            ultra::timing::globalDecoderProfile()
                .low_llr_escalation_skipped.fetch_add(1, std::memory_order_relaxed);
            LOG_MODEM(INFO, "[%s] OFDM CW0 peek: |llr|_avg=%.1f too low — skipping fixed-frame escalation (likely false sync)",
                      log_prefix_.c_str(), llr_abs_avg);
            advancePastFalseOFDMLock();
            state_ = DecoderState::SEARCHING;
            return;
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
            if (total_fed_ < buffer_capacity_samples_) {
                return ring_pos;
            }
            const size_t oldest_abs = total_fed_ - buffer_capacity_samples_;
            const size_t oldest_pos = write_pos_;
            const size_t offset = (ring_pos >= oldest_pos)
                ? (ring_pos - oldest_pos)
                : (buffer_capacity_samples_ - oldest_pos + ring_pos);
            return oldest_abs + offset;
        };

        if (allow_sync_recovery) {
            for (size_t retry_idx = 0; retry_idx < retry_delta_count; ++retry_idx) {
                const int delta = retry_deltas[retry_idx];
                if (delta < 0 && total_fed_ < buffer_capacity_samples_ &&
                    sync_position_ < static_cast<size_t>(-delta)) {
                    continue;
                }
                recovery_attempts++;
                size_t retry_sync = wrapRingIndexLocked(sync_position_ + buffer_capacity_samples_ + delta);

                std::vector<float> retry_buffer;
                size_t retry_len = frame_len;
                {
                    std::lock_guard<std::mutex> lock(buffer_mutex_);
                    size_t available;
                    if (write_pos_ >= retry_sync) {
                        available = write_pos_ - retry_sync;
                    } else {
                        available = buffer_capacity_samples_ - retry_sync + write_pos_;
                    }
                    retry_len = std::min(retry_len, available);
                    if (retry_len == 0) {
                        continue;
                    }
                    retry_buffer.resize(retry_len);
                    for (size_t i = 0; i < retry_len; ++i) {
                        retry_buffer[i] = buffer_[wrapRingIndexLocked(retry_sync + i)];
                    }
                }

                // Pre-correct CFO on retry buffer too
                if (is_ofdm && std::abs(pre_correction_cfo_) > 0.01f) {
                    applyCFOPreCorrection(retry_buffer, sync_cfo_, ringPosToAbsolute(retry_sync));
                }

                waveform_->reset();
                waveform_->setAbsoluteTrainingPosition(ringPosToAbsolute(retry_sync));
                waveform_->setFrequencyOffset(decode_cfo);
                const int retry_expected_codewords =
                    expectedOFDMCodewordsForSamples(retry_buffer.size());
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
                const float current_cfo = last_cfo_.load();
                const auto cfo_update = signal_policy::combinePilotCFO(
                    pre_correction_cfo_, residual_cfo, current_cfo, connected_);
                last_cfo_.store(cfo_update.accepted_cfo);
                sync_cfo_ = cfo_update.accepted_cfo;
                last_fading_index_.store(waveform_->getFadingIndex());

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
            evaluatePingDecision(ldpc_decode_succeeded, ldpc_magic_valid);

        LOG_MODEM(INFO, "[%s] PING check PATH2: ratio=%.3f, "
                  "chirp_corr=%.3f (floor=%.2f), gap_error=%.1f (max=%.0f), "
                  "ldpc_ok=%d, magic=%d, path1=%d, path2=%d",
                  log_prefix_.c_str(), ping_decision.ratio,
                  ping_decision.chirp_corr, frame_policy::kPingCorrFloor,
                  ping_decision.gap_error_samples, frame_policy::kPingMaxGapError,
                  ping_decision.ldpc_decode_succeeded ? 1 : 0,
                  ping_decision.ldpc_magic_valid ? 1 : 0,
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

    const bool deliver_partial_mc_dpsk =
        !result.success && mode_ == protocol::WaveformMode::MC_DPSK &&
        result.has_partial_codewords;
    if (result.success || result.codewords_ok > 0) {
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

        LOG_MODEM(INFO, "[%s] StreamingDecoder: Frame decoded, %d/%d CWs, SNR=%.1f dB, CFO=%.1f Hz",
                  log_prefix_.c_str(), result.codewords_ok, result.codewords_ok + result.codewords_failed,
                  result.snr_db, result.cfo_hz);
    } else {
        LOG_MODEM(WARN, "[%s] StreamingDecoder: Decode failed (cw_ok=%d, cw_fail=%d, is_ping=%d)",
                  log_prefix_.c_str(), result.codewords_ok, result.codewords_failed, result.is_ping ? 1 : 0);
    }

    // Calculate consumed samples based on actual frame content
    // For non-data frames (control/connect), use exact sample count to avoid eating into next frame
    const bool is_non_data_frame = frame_policy::isNonDataFrame(
        result.success, result.frame_data.data(), result.frame_data.size());

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
    size_t next_block_pos = wrapRingIndexLocked(sync_position_ + consumed);
    size_t next_search_abs = frame_sync_abs + consumed;

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
                std::lock_guard<std::mutex> lock(buffer_mutex_);
                if (write_pos_ >= next_block_pos) {
                    next_available = write_pos_ - next_block_pos;
                } else {
                    next_available = buffer_capacity_samples_ - next_block_pos + write_pos_;
                }
            }

            if (next_available < min_block) break;  // Not enough samples, burst over

            // Copy next block samples
            std::vector<float> next_block(min_block);
            {
                std::lock_guard<std::mutex> lock(buffer_mutex_);
                for (size_t i = 0; i < min_block; i++) {
                    next_block[i] = buffer_[wrapRingIndexLocked(next_block_pos + i)];
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
            const size_t burst_llr_n = std::min(next_soft_bits.size(), size_t(648));
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
            const auto cfo_update = signal_policy::combinePilotCFO(
                0.0f, residual_cfo, sync_cfo_, /*clamp_drift=*/true);
            sync_cfo_ = cfo_update.accepted_cfo;
            last_cfo_.store(cfo_update.accepted_cfo);
            last_fading_index_.store(waveform_->getFadingIndex());

            // Decode the continuation block
            DecodeResult next_result = decodeFrame(next_soft_bits, sync_snr_, sync_cfo_);
            populateDecodeMetrics(next_result, true, residual_cfo);

            {
                std::lock_guard<std::mutex> slock(stats_mutex_);
                if (next_result.success) stats_.frames_decoded++;
                else stats_.frames_failed++;
            }

            if (next_result.success || next_result.codewords_ok > 0) {
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

            // Advance position for next iteration (or final correlation_pos_)
            sync_position_ = next_block_pos;
            next_block_pos = wrapRingIndexLocked(next_block_pos + min_block);
            next_search_abs += min_block;

            // If decode failed completely, stop the burst
            if (!next_result.success && next_result.codewords_ok == 0) break;
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
    burst_blocks_decoded_ = 0;
    {
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        correlation_pos_ = wrapRingIndexLocked(next_block_pos);
        setSearchFloorLocked(next_search_abs);
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
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        correlation_pos_ = search_pos;
        setSearchFloorLocked(search_abs);
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
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        if (write_pos_ >= pos) {
            return write_pos_ - pos;
        }
        return buffer_capacity_samples_ - pos + write_pos_;
    };

    auto copyFrom = [&](size_t pos, size_t len) -> std::vector<float> {
        std::vector<float> out(len);
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        for (size_t i = 0; i < len; ++i) {
            out[i] = buffer_[wrapRingIndexLocked(pos + i)];
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

        codec_->setRate(code_rate_);
        std::vector<float> cw0_bits(soft.begin(), soft.begin() + LDPC_BLOCK);
        auto [peek_ok, peek_data] = codec_->decode(cw0_bits);
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
        mc_burst_next_pos_ = wrapRingIndexLocked(mc_burst_next_pos_ + one_cw_samples);
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
        mc_burst_next_pos_ = wrapRingIndexLocked(mc_burst_next_pos_ + remaining_samples);
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

float StreamingDecoder::estimateSNRFromChirp(float corr, float /*noise*/) {
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
    result.cfo_hz = cfo;

    constexpr size_t LDPC_BLOCK = v2::LDPC_CODEWORD_BITS;

    if (soft_bits.size() < LDPC_BLOCK) return result;

    codec_->setRate(rate);

    // Decode CW0 (header codeword)
    std::vector<float> cw0_bits(soft_bits.begin(), soft_bits.begin() + LDPC_BLOCK);
    auto [ok0, data0] = codec_->decode(cw0_bits);

    if (!ok0 || data0.size() < 2 || data0[0] != 0x55 || data0[1] != 0x4C) {
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
            auto [ok, data] = codec_->decode(bits);
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

        auto [ok, data] = codec_->decode(bits);
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
    result.cfo_hz = cfo;

    constexpr size_t LDPC_BLOCK = v2::LDPC_CODEWORD_BITS;

    if (soft_bits.size() < LDPC_BLOCK) return result;

    CodeRate rate = connected_ ? code_rate_ : CodeRate::R1_4;
    size_t bytes_per_cw = v2::getBytesPerCodeword(rate);
    codec_->setRate(rate);

    // Channel interleaving only applies to OFDM modes, NOT MC-DPSK
    bool is_ofdm = protocol::isOFDMMode(mode_);
    bool apply_channel_deinterleave = use_channel_interleave_ && is_ofdm;

    // Helper to deinterleave a codeword if needed
    auto deinterleave_cw = [&](const std::vector<float>& cw) -> std::vector<float> {
        if (apply_channel_deinterleave) {
            return interleaver_->deinterleave(cw);
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
        codec_->setRate(CodeRate::R1_4);
        std::vector<float> cw0_r14(soft_bits.begin(), soft_bits.begin() + LDPC_BLOCK);
        std::pair<bool, Bytes> probe;
        {
            ultra::timing::ScopedTimer _profile_(
                ultra::timing::globalDecoderProfile().ofdm_cw0_probe_decode);
            probe = codec_->decode(cw0_r14);
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
        codec_->setRate(rate);
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
            raw_probe = codec_->decode(cw0_bits);
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

            const size_t bytes_per_fixed_cw = v2::getBytesPerCodeword(rate);
            for (int candidate_cw : candidates) {
                const size_t frame_bits =
                    static_cast<size_t>(fec::FrameInterleaver::totalFrameBits(candidate_cw));
                if (soft_bits.size() < frame_bits) {
                    continue;
                }

                std::vector<float> cw0_bits;
                try {
                    auto cw_soft = fec::FrameInterleaver::deinterleave(soft_bits, candidate_cw);
                    if (cw_soft.empty() || cw_soft[0].size() < LDPC_BLOCK) {
                        continue;
                    }
                    cw0_bits = std::move(cw_soft[0]);
                } catch (const std::exception&) {
                    continue;
                }

                if (apply_channel_deinterleave) {
                    ChannelInterleaver channel_deinterleaver(bps, v2::LDPC_CODEWORD_BITS);
                    cw0_bits = channel_deinterleaver.deinterleave(cw0_bits);
                }

                auto [peek_ok, peek_data] = robustDecodeSingleCW(
                    cw0_bits.data(), cw0_bits.size(), rate, log_prefix_.c_str(),
                    ultra::timing::SingleCWCallSite::Cw0Peek);
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
            static_cast<size_t>(fec::FrameInterleaver::totalFrameBits(decode_cw_count));
        if (soft_bits.size() < frame_interleave_bits) {
            return result;
        }
        LOG_MODEM(DEBUG, "[%s] Attempting %d-CW frame deinterleave decode",
                  log_prefix_.c_str(), decode_cw_count);

        // Use v2::decodeFixedFrame which handles frame + channel deinterleaving + LDPC decode
        // Channel deinterleaving restores the original bit order within each CW
        // Only enable for OFDM modes (MC-DPSK doesn't use channel interleaving)
        auto buildHarqKey = [&](int cw_count, fec::SoftCombineBuffer::Key& out_key) -> bool {
            if (!harq_buffer_ || !harq_buffer_->enabled()) {
                return false;
            }

            cw_count = v2::sanitizeFixedFrameCodewords(cw_count);
            const size_t frame_bits =
                static_cast<size_t>(fec::FrameInterleaver::totalFrameBits(cw_count));
            if (soft_bits.size() < frame_bits) {
                return false;
            }

            auto cw_soft = fec::FrameInterleaver::deinterleave(soft_bits, cw_count);
            if (cw_soft.empty() || cw_soft[0].size() < LDPC_BLOCK) {
                return false;
            }

            std::vector<float> cw0_bits = std::move(cw_soft[0]);
            if (apply_channel_deinterleave) {
                ChannelInterleaver channel_deinterleaver(bps, v2::LDPC_CODEWORD_BITS);
                cw0_bits = channel_deinterleaver.deinterleave(cw0_bits);
            }

            auto [peek_ok, peek_data] = robustDecodeSingleCW(
                cw0_bits.data(), cw0_bits.size(), rate, log_prefix_.c_str(),
                ultra::timing::SingleCWCallSite::Cw0Peek);
            const size_t bytes_per_fixed_cw = v2::getBytesPerCodeword(rate);
            if (!peek_ok || peek_data.size() < bytes_per_fixed_cw) {
                return false;
            }
            if (peek_data.size() > bytes_per_fixed_cw) {
                peek_data.resize(bytes_per_fixed_cw);
            }

            auto hdr = v2::parseHeader(peek_data);
            if (!hdr.valid || hdr.is_control || !isFixedFrameCwCount(hdr.total_cw)) {
                return false;
            }

            fec::SoftCombineBuffer::HarqKeyInputs ki;
            ki.sender_hash = hdr.src_hash;
            ki.seq = hdr.seq;
            ki.rate = rate;
            ki.cw_count = hdr.total_cw;
            ki.modulation = current_modulation_;
            ki.channel_interleave = apply_channel_deinterleave;
            ki.waveform_mode = static_cast<int>(mode_);
            ki.ofdm_data_carriers = ofdm_data_carriers_;
            out_key = fec::SoftCombineBuffer::makeKey(ki);
            return out_key.sender_hash != 0;
        };
        const auto _profile_fs_start_ = std::chrono::steady_clock::now();
        auto decodeFixed = [&](int cw_count) {
            fec::SoftCombineBuffer::Key harq_key;
            fec::SoftCombineBuffer::Key* harq_key_ptr = nullptr;
            fec::SoftCombineBuffer* harq_buffer = nullptr;
            // Only count when HARQ is actually wanted (buffer enabled).
            // A "failed key build" only matters if HARQ would otherwise
            // have engaged — counting when HARQ is off would make every
            // MC-DPSK or non-fixed-frame decode look like a HARQ miss.
            const bool harq_active = harq_buffer_ && harq_buffer_->enabled();
            if (buildHarqKey(cw_count, harq_key)) {
                harq_key_ptr = &harq_key;
                harq_buffer = harq_buffer_;
                if (harq_active) {
                    ultra::timing::globalDecoderProfile()
                        .harq_key_build_success.fetch_add(
                            1, std::memory_order_relaxed);
                }
            } else if (harq_active) {
                // Codex review #17: HARQ misses every frame whose
                // first-attempt CW0 fails. Count these so we can
                // measure whether a session-context fallback key
                // would actually move the needle.
                ultra::timing::globalDecoderProfile()
                    .harq_key_build_failed.fetch_add(
                        1, std::memory_order_relaxed);
            }
            return v2::decodeFixedFrame(soft_bits, rate, cw_count,
                                        apply_channel_deinterleave, bps,
                                        harq_buffer, harq_key_ptr);
        };
        auto cw_status = decodeFixed(decode_cw_count);

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
                static_cast<size_t>(fec::FrameInterleaver::totalFrameBits(header_cw_count));
            if (soft_bits.size() >= header_frame_bits) {
                LOG_MODEM(INFO, "[%s] Fixed-frame header says %d CWs; retrying decode (was %d)",
                          log_prefix_.c_str(), header_cw_count, decode_cw_count);
                decode_cw_count = header_cw_count;
                cw_status = decodeFixed(decode_cw_count);
            }
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

            auto [ok, data] = codec_->decode(bits);
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
