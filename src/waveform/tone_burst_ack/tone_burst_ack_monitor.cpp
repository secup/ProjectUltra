// tone_burst_ack_monitor.cpp — sliding-window ACK detection driver.

#include "tone_burst_ack_monitor.hpp"

#include <cstdlib>

#include "ultra/logging.hpp"

#include <algorithm>
#include <cstring>

namespace ultra {
namespace waveform {
namespace tone_burst_ack {

ToneBurstAckMonitor::ToneBurstAckMonitor() : ToneBurstAckMonitor(Config{}) {}

ToneBurstAckMonitor::ToneBurstAckMonitor(Config cfg) : cfg_(std::move(cfg)) {
    buffer_.reserve(cfg_.buffer_capacity_samples);
}

float ToneBurstAckMonitor::idealCostasPeak(uint32_t symbol_ms) {
    const uint32_t spp = (kSampleRate * symbol_ms) / 1000u;
    // Goertzel magnitude for matched tone of amplitude A over N samples: N·A/2.
    const float per_symbol = (static_cast<float>(spp) / 2.0f) * kToneAmplitude;
    // Diversity ACK (equal peak, two half-amplitude tones, square-law combined): the clean
    // combined magnitude is sqrt(2 (N A/4)^2) = (N A/2) / sqrt(2).
    const float combine = toneAckDiversityEnabled() ? 0.70710678f : 1.0f;
    return per_symbol * static_cast<float>(kCostasSymbols) * combine;
}

void ToneBurstAckMonitor::reset() {
    buffer_.clear();
    buffer_start_stream_offset_ = total_samples_fed_;
    last_detect_pos_in_buffer_ = 0;
    last_detection_stream_offset_ = static_cast<uint64_t>(-1);
    last_semantic_rejection_stream_offset_ = static_cast<uint64_t>(-1);
    armed_ = false;
    arm_deadline_stream_offset_ = 0;
}

void ToneBurstAckMonitor::arm(size_t window_samples) {
    const uint64_t new_deadline = total_samples_fed_ + window_samples;
    if (armed_ && new_deadline < arm_deadline_stream_offset_) {
        // Calling arm() with a SHORTER window than current — keep the
        // larger one (the caller may already be expecting an ACK from a
        // previous burst with a later deadline).
        return;
    }
    armed_ = true;
    arm_deadline_stream_offset_ = new_deadline;
    // BUG-POSTTX-ACK-MISS forensics (2026-07-05): reset the per-window counters +
    // the gapless anchor (nothing before arm-time can be our ACK — same argument
    // as the last_detect_pos skip-forward below). INFO (was DEBUG) so a standard
    // rig log shows every armed window — an undetected ACK is then diagnosable
    // from the log alone (armed? expired? max chunk?).
    scan_high_water_stream_ = total_samples_fed_;
    max_feed_chunk_while_armed_ = 0;
    passes_while_armed_ = 0;
    armed_at_stream_ = total_samples_fed_;
    LOG_MODEM(INFO, "ToneBurstAckMonitor armed: window_samples=%zu fed=%llu deadline=%llu buf_size=%zu "
              "cfo_prior=%.2f cfo_halfwidth=%.1f",
              window_samples, (unsigned long long)total_samples_fed_,
              (unsigned long long)arm_deadline_stream_offset_, buffer_.size(),
              detector_.frequencyPriorHz(), detector_.frequencyHalfWidthHz());
    // Reset the cadence counter so the FIRST detection pass happens at the
    // current sample position + armed-cadence interval, not whenever the
    // background cadence next fires. This gives the first detection a
    // bounded latency right after arm().
    last_detect_pos_in_buffer_ =
        (total_samples_fed_ >= buffer_start_stream_offset_)
            ? static_cast<size_t>(total_samples_fed_ - buffer_start_stream_offset_)
            : 0;
}

void ToneBurstAckMonitor::rearm(size_t window_samples) {
    if (!cfg_.armed_only) return;
    armed_ = false;
    arm_deadline_stream_offset_ = 0;
    arm(window_samples);
}

void ToneBurstAckMonitor::feedAudio(const float* samples, size_t count) {
    if (count == 0 || samples == nullptr) return;

    // Append to buffer.
    buffer_.insert(buffer_.end(), samples, samples + count);
    total_samples_fed_ += count;

    // If we exceeded capacity, drop the oldest samples. Maintain the
    // invariant that buffer_start_stream_offset_ tracks buffer_[0]'s
    // position in the overall feed stream.
    if (buffer_.size() > cfg_.buffer_capacity_samples) {
        const size_t excess = buffer_.size() - cfg_.buffer_capacity_samples;
        buffer_.erase(buffer_.begin(), buffer_.begin() + excess);
        buffer_start_stream_offset_ += excess;
        // last_detect_pos_in_buffer_ is relative to buffer_[0]; shift it.
        if (last_detect_pos_in_buffer_ > excess) {
            last_detect_pos_in_buffer_ -= excess;
        } else {
            last_detect_pos_in_buffer_ = 0;
        }
    }

    // BUG-POSTTX-ACK-MISS forensics: track the largest single append while
    // armed — a chunk larger than a bin's tail window is the tail-hole trigger.
    if (armed_ && count > max_feed_chunk_while_armed_) {
        max_feed_chunk_while_armed_ = count;
    }

    // §15 step 4d-late: pick the active cadence based on armed state.
    //   armed:    detect_interval_samples_armed (tight, low latency)
    //   not armed (and armed_only): no cadence — runDetectionPass is gated
    //   not armed (always-on): detect_interval_samples (background polling)
    if (armed_ && total_samples_fed_ >= arm_deadline_stream_offset_) {
        // Window elapsed without a successful decode — disarm. The
        // protocol layer's existing ack_timeout will handle the
        // retransmit; we go idle until armed again.
        armed_ = false;
        // Forensic INFO: an expired-undetected window is the missed-ACK
        // signature — log enough to classify it from a standard rig log
        // (fed_in_window ~0 = capture never resumed / deaf at audio layer;
        // max_chunk > ~25k = the tail-hole class; healthy fed + small chunks
        // + many passes = the tone genuinely wasn't decodable = fade).
        LOG_MODEM(INFO,
                  "ToneBurstAckMonitor: armed window EXPIRED undetected — "
                  "fed_in_window=%llu max_chunk=%zu passes=%llu buf=%zu",
                  (unsigned long long)(total_samples_fed_ - armed_at_stream_),
                  max_feed_chunk_while_armed_,
                  (unsigned long long)passes_while_armed_, buffer_.size());
    }
    if (!armed_ && cfg_.armed_only) {
        // Idle: skip detection entirely. No CPU work on the audio thread.
        return;
    }
    const size_t cadence = armed_ ? cfg_.detect_interval_samples_armed
                                   : cfg_.detect_interval_samples;
    if (cadence == 0) return;

    // Trigger detection passes at the active cadence. The "position" we
    // compare against is the cumulative samples fed (absolute, not
    // buffer-relative, so backlogged audio doesn't trigger a flood).
    while (true) {
        const uint64_t next_trigger_stream =
            buffer_start_stream_offset_ + last_detect_pos_in_buffer_ +
            cadence;
        if (next_trigger_stream > total_samples_fed_) break;
        last_detect_pos_in_buffer_ =
            next_trigger_stream - buffer_start_stream_offset_;
        runDetectionPass();
        // If a successful decode disarmed us inside runDetectionPass(), or
        // the window elapsed, stop the inner loop here.
        if (cfg_.armed_only && !armed_) break;
    }
}

void ToneBurstAckMonitor::runDetectionPass() {
    if (armed_) ++passes_while_armed_;
    // GAPLESS armed sweep: advance the high-water mark to the buffer end AFTER
    // this pass's windows are computed (each bin's window below is derived from
    // the PRE-pass hw). A tone straddling the end whose tail hasn't arrived yet
    // fails decode this pass; it is re-covered next pass because its start sits
    // above (hw_new − needed) — see the Config comment's induction argument.
    const uint64_t hw_after_this_pass =
        buffer_start_stream_offset_ + buffer_.size();
    // For each candidate symbol duration, try to detect+decode in the
    // current buffer. Take the FIRST CRC-passing decode (the §15.5
    // staircase implicitly suggests trying short first, then longer if
    // not found — but the dominant channel duration is set by negotiated
    // SNR, so a single duration usually wins).
    for (uint32_t symbol_ms : cfg_.symbol_durations_ms) {
        const uint32_t spp = (kSampleRate * symbol_ms) / 1000u;
        const size_t needed = static_cast<size_t>(kTotalSymbols) * spp;
        if (buffer_.size() < needed) {
            // 2026-07-04 (R3/4 ACK-miss forensics): this skip was SILENT while the
            // production buffer could never hold the 100 ms rung — every slow-bin
            // ACK was structurally undecodable and left no sender-side trace.
            // WARN once per pass-capacity mismatch (capacity, not fill level:
            // buffer_ at capacity still < needed means the rung can NEVER decode).
            if (buffer_.size() >= cfg_.buffer_capacity_samples &&
                !capacity_skip_warned_) {
                capacity_skip_warned_ = true;
                LOG_MODEM(WARN,
                          "ToneBurstAckMonitor: %u ms rung UNDECODABLE — needs %zu "
                          "samples, buffer capacity %zu (a peer ACK at this rung "
                          "will be silently missed)",
                          symbol_ms, needed, cfg_.buffer_capacity_samples);
            }
            continue;
        }
        // TAIL-WINDOW sweep (2026-07-04, rig F2 regression): sweeping the WHOLE
        // buffer per duration per pass scales the per-pass CPU with buffer size —
        // the forensic-arc capacity growth (120 k -> 172.8 k, needed so the 100 ms
        // rung is decodable AT ALL) pushed the Pi5's ARM decode thread ~19 s behind
        // live and the sender went ACK-deaf (1/4 detected). A pass only ever needs
        // to find a burst whose TAIL arrived since the previous pass: one full
        // burst length + one cadence interval + one symbol of alignment guard,
        // anchored at the buffer's newest end. This is also strictly cheaper than
        // the pre-arc code (which swept 120 k even for the 19.6 k fast rung).
        const size_t cadence_guard = armed_ ? cfg_.detect_interval_samples_armed
                                            : cfg_.detect_interval_samples;
        size_t window = cfg_.tail_window_sweep
            ? std::min(buffer_.size(), needed + cadence_guard + spp)
            : buffer_.size();
        // GAPLESS armed sweep (BUG-POSTTX-ACK-MISS — see Config comment): extend
        // this bin's window to also cover everything since the scan high-water
        // mark plus one burst of context, so a single large append (post-TX
        // capture-resume backlog) cannot leave audio that no pass ever scans.
        // Steady state: hw trails the end by ~one cadence → window ≈ tail
        // behavior; a backlog chunk pays a one-off proportional sweep.
        if (cfg_.gapless_armed_sweep && armed_) {
            const uint64_t end_stream =
                buffer_start_stream_offset_ + buffer_.size();
            const uint64_t hw =
                std::max(scan_high_water_stream_, buffer_start_stream_offset_);
            const uint64_t unscanned = (end_stream > hw) ? (end_stream - hw) : 0;
            const size_t gapless_window = static_cast<size_t>(std::min<uint64_t>(
                buffer_.size(), unscanned + needed + spp));
            window = std::min(buffer_.size(), std::max(window, gapless_window));
        }
        const size_t tail_base = buffer_.size() - window;
        const auto r = detector_.detectAndDecode(
            buffer_.data() + tail_base, window, symbol_ms,
            cfg_.sweep_step_samples, acceptance_predicate_,
            idealCostasPeak(symbol_ms) * cfg_.min_peak_fraction);
        if (r.semantic_rejections > 0) {
            semantic_candidates_rejected_ += r.semantic_rejections;
            if (r.strongest_rejected_payload.has_value()) {
                const uint64_t rejected_stream_offset =
                    buffer_start_stream_offset_ + static_cast<uint64_t>(tail_base) +
                    static_cast<uint64_t>(r.strongest_rejected_offset_samples);
                // A rejected burst remains buffered and may be revisited on the next
                // cadence pass. Log it once per stream location, while keeping the
                // cumulative counter exact for forensic summaries/tests.
                if (rejected_stream_offset !=
                    last_semantic_rejection_stream_offset_) {
                    last_semantic_rejection_stream_offset_ =
                        rejected_stream_offset;
                    const auto& p = *r.strongest_rejected_payload;
                    const float ideal = idealCostasPeak(symbol_ms);
                    const float normalized_peak =
                        ideal > 0.0f
                            ? r.strongest_rejected_correlation_peak / ideal
                            : 0.0f;
                    LOG_MODEM(
                        WARN,
                        "ToneBurstAckMonitor: CRC-valid candidate rejected before "
                        "commit group_seq=%u type=%s mask=0x%04X epoch=%u "
                        "peak=%.1f normalized_peak=%.3f min_conf=%.3f "
                        "hamming_corrected=%d symbol_ms=%u stream_offset=%llu "
                        "rejected_in_search=%zu — monitor remains armed",
                        static_cast<unsigned>(p.group_seq),
                        p.type == AckType::Nack ? "NACK" : "ACK",
                        static_cast<unsigned>(p.frame_mask),
                        static_cast<unsigned>(p.move_epoch),
                        r.strongest_rejected_correlation_peak, normalized_peak,
                        r.strongest_rejected_min_confidence,
                        r.strongest_rejected_hamming_corrected_blocks,
                        static_cast<unsigned>(symbol_ms),
                        static_cast<unsigned long long>(rejected_stream_offset),
                        r.semantic_rejections);
                }
            }
        }
        {
            static const bool kTrace = [] {
                const char* e = std::getenv("ULTRA_TONE_ACK_TRACE");
                return e && e[0] == '1';
            }();
            const float ideal_t = idealCostasPeak(symbol_ms);
            if (kTrace && ideal_t > 0.0f && r.costas_correlation_peak > 0.25f * ideal_t) {
                LOG_MODEM(INFO,
                          "TONE_TRACE pass=%llu ms=%u win=%zu tail_base=%zu buf=%zu "
                          "start_stream=%llu peak_norm=%.3f off=%zu crc=%d payload=%d "
                          "hyps=%d cfo=%.2f fed=%llu",
                          (unsigned long long)passes_while_armed_, symbol_ms, window,
                          tail_base, buffer_.size(),
                          (unsigned long long)buffer_start_stream_offset_,
                          r.costas_correlation_peak / ideal_t, r.detected_offset_samples,
                          r.decode.stats.crc_ok ? 1 : 0, r.decode.payload.has_value() ? 1 : 0,
                          r.frequency_hypotheses, r.detected_frequency_offset_hz,
                          (unsigned long long)total_samples_fed_);
            }
        }
        if (!r.decode.payload.has_value()) continue;
        if (!r.decode.stats.crc_ok) continue;
        // Peak-magnitude gate: rejects rare cases where Costas+Hamming+CRC
        // all happen to pass on a spurious low-energy match (e.g. trying
        // 12 ms duration on a 25 ms burst that aliases into a noise-floor
        // peak). Real bursts have peaks at ~30%+ of ideal even at SNR=-10.
        const float min_peak =
            idealCostasPeak(symbol_ms) * cfg_.min_peak_fraction;
        if (r.costas_correlation_peak < min_peak) continue;

        const uint64_t detected_stream_offset =
            buffer_start_stream_offset_ + static_cast<uint64_t>(tail_base) +
            static_cast<uint64_t>(r.detected_offset_samples);

        // Duplicate-detection suppression: drop if the previous detection
        // landed within suppress_window_samples of this one.
        if (last_detection_stream_offset_ != static_cast<uint64_t>(-1)) {
            const uint64_t diff = (detected_stream_offset > last_detection_stream_offset_)
                ? detected_stream_offset - last_detection_stream_offset_
                : last_detection_stream_offset_ - detected_stream_offset;
            if (diff < cfg_.suppress_window_samples) {
                ++suppressed_attempts_;
                return;
            }
            // STREAM-MONOTONICITY guard (BUG-TONEACK-FABRICATION, F116 2026-07-05):
            // detections must move FORWARD in stream time. A detection at/behind the
            // previous one is a re-scan of already-consumed audio — the F116 phantom
            // was the peer's old 12 ms ACK re-decoded 60k samples later at the 50 ms
            // rung (duration aliasing) after the under-consume bug below left it in
            // the buffer. Real consecutive acks always advance the stream.
            if (detected_stream_offset <= last_detection_stream_offset_) {
                ++suppressed_attempts_;
                LOG_MODEM(WARN,
                          "ToneBurstAckMonitor: REGRESSED detection at stream %llu "
                          "(last %llu, symbol_ms=%d) — stale-audio re-decode dropped",
                          static_cast<unsigned long long>(detected_stream_offset),
                          static_cast<unsigned long long>(last_detection_stream_offset_),
                          symbol_ms);
                return;
            }
        }

        last_detection_stream_offset_ = detected_stream_offset;
        ++detections_emitted_;

        // §15 step 4d-late: disarm on successful decode. The protocol
        // layer's burst transport advances and will arm() again before
        // the next expected ACK.
        armed_ = false;

        if (callback_) {
            ToneBurstAckDetection d;
            d.payload = *r.decode.payload;
            d.detected_stream_offset = detected_stream_offset;
            d.correlation_peak = r.costas_correlation_peak;
            d.min_symbol_confidence = r.decode.min_confidence;
            d.hamming_corrected_blocks = r.decode.stats.hamming_corrected_blocks;
            d.symbol_ms_used = symbol_ms;
            d.frequency_offset_hz = r.detected_frequency_offset_hz;
            d.frequency_prior_hz = detector_.frequencyPriorHz();
            callback_(d);
        }

        // Drop everything in the buffer up to past the decoded burst so
        // we don't re-scan the same audio on the next cadence tick. Leave
        // ~50 ms of trailing samples as guard.
        // r.detected_offset_samples is WINDOW-relative (the detector saw
        // buffer_ + tail_base) — the consume point must add tail_base back
        // (F116: omitting it under-consumed whenever tail_base > 0, leaving the
        // decoded burst re-scannable at OTHER symbol_ms rungs on later ticks —
        // the duration-aliased phantom-ack surface).
        const size_t consume_until = tail_base + r.detected_offset_samples + needed;
        const size_t guard = static_cast<size_t>(kSampleRate) * 50u / 1000u;
        const size_t drop = std::min(
            buffer_.size(), (consume_until > guard) ? (consume_until - guard) : 0);
        if (drop > 0) {
            buffer_.erase(buffer_.begin(), buffer_.begin() + drop);
            buffer_start_stream_offset_ += drop;
            last_detect_pos_in_buffer_ =
                (last_detect_pos_in_buffer_ > drop) ? last_detect_pos_in_buffer_ - drop : 0;
        }
        scan_high_water_stream_ =
            std::max(scan_high_water_stream_, hw_after_this_pass);
        return;
    }
    // GAPLESS armed sweep: everything up to the buffer end has now been inside
    // a scan window with full per-bin context (windows were derived from the
    // PRE-pass hw above).
    scan_high_water_stream_ =
        std::max(scan_high_water_stream_, hw_after_this_pass);
}

}  // namespace tone_burst_ack
}  // namespace waveform
}  // namespace ultra
