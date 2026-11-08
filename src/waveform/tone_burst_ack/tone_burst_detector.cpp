// tone_burst_detector.cpp — Goertzel-based 4-FSK demodulator + Costas sweep.

#include "tone_burst_detector.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <numeric>

namespace ultra {
namespace waveform {
namespace tone_burst_ack {

namespace {

constexpr float kTwoPi = 6.283185307179586f;

// Goertzel single-frequency power for `freq_hz` over a window of N samples.
// Returns the squared magnitude (proportional to power, no sqrt for speed).
float goertzelPowerAt(SampleSpan samples, size_t n_samples, float freq_hz) {
    const float coeff = 2.0f * std::cos(kTwoPi * freq_hz /
                                         static_cast<float>(kSampleRate));
    float q0 = 0.0f;
    float q1 = 0.0f;
    float q2 = 0.0f;
    for (size_t i = 0; i < n_samples; ++i) {
        q0 = coeff * q1 - q2 + samples[i];
        q2 = q1;
        q1 = q0;
    }
    // Squared magnitude of the resulting complex tone amplitude.
    return q1 * q1 + q2 * q2 - coeff * q1 * q2;
}

using Cd = std::complex<double>;
constexpr double kTwoPiD = 6.283185307179586476925286766559;
constexpr int kMaxBands = 2;
constexpr int kMaxBatch = kCostasSymbols * kMaxBands;  // 4 Costas tones x 2 subbands

// Tone k on subband `band` (0 = primary, 1 = diversity), shifted by the carrier offset.
float toneHz(uint32_t k, int band, float offset_hz) {
    return (band == 0 ? kToneFreqHz[k] : kToneFreqHzDiversity[k]) + offset_hz;
}

double omegaOf(float hz) { return kTwoPiD * static_cast<double>(hz) / kSampleRate; }

// GRID PREFIX SUMS for nf frequencies in one pass over x:
//     out[f][k] = sum_{m = begin}^{begin + k g - 1} x[m] e^{-j omega_f m},   k = 0..count_grid,
// with an ABSOLUTE phase reference m, so for any window [a, a + N) on the grid the DFT
// magnitude |sum_m x[m] e^{-j omega m}| is |out[(a + N - begin) / g] - out[(a - begin) / g]| —
// exactly goertzelPowerAt(x + a, N, f) up to rounding. Each frequency runs the Goertzel
// recursion s[m] = x[m] + 2 cos(w) s[m-1] - s[m-2] in DOUBLE (prefix sums grow with the span;
// float would lose the window difference), and at every grid point extracts
// y = s[m] - e^{-jw} s[m-1] = sum_l x[l] e^{jw(m-l)}, i.e. S(m) = e^{-jwm} y. The nf
// recursions are independent, so one pass keeps the loop throughput-bound, not latency-bound.
void gridPrefixSums(const float* x, size_t begin, size_t count_grid, size_t g,
                    const double* omega, int nf, std::vector<Cd>* out) {
    double coeff[kMaxBatch], s1[kMaxBatch] = {}, s2[kMaxBatch] = {};
    Cd e_mjw[kMaxBatch], rot_g[kMaxBatch], ph[kMaxBatch];
    for (int f = 0; f < nf; ++f) {
        coeff[f] = 2.0 * std::cos(omega[f]);
        e_mjw[f] = std::polar(1.0, -omega[f]);
        rot_g[f] = std::polar(1.0, -omega[f] * static_cast<double>(g));
        ph[f] = std::polar(1.0, -omega[f] * static_cast<double>(begin + g - 1));
        out[f].assign(count_grid + 1, Cd(0.0, 0.0));
    }
    size_t m = begin;
    for (size_t k = 1; k <= count_grid; ++k) {
        for (size_t i = 0; i < g; ++i, ++m) {
            const double xm = static_cast<double>(x[m]);
            for (int f = 0; f < nf; ++f) {
                const double s0 = xm + coeff[f] * s1[f] - s2[f];
                s2[f] = s1[f];
                s1[f] = s0;
            }
        }
        for (int f = 0; f < nf; ++f) {
            out[f][k] = ph[f] * (Cd(s1[f], 0.0) - e_mjw[f] * s2[f]);
            ph[f] *= rot_g[f];
            if ((k & 1023u) == 0) ph[f] /= std::abs(ph[f]);
        }
    }
}

// Costas statistic sum_i sqrt(sum_b |X_{i,b}|^2) from per-(symbol, band) window DFTs.
float costasStatistic(const Cd* X, int bands) {
    double sum = 0.0;
    for (uint32_t i = 0; i < kCostasSymbols; ++i) {
        double p = 0.0;
        for (int b = 0; b < bands; ++b) p += std::norm(X[i * bands + b]);
        sum += std::sqrt(p);
    }
    return static_cast<float>(sum);
}

// FINE timing sweep at one carrier-offset hypothesis: every offset o in [lo, hi] at 1-sample
// resolution, the four Costas-window DFTs SLID one sample at a time
//     X(o+1) = X(o) - x[o + i N] e^{-jw(o + i N)} + x[o + (i+1) N] e^{-jw(o + (i+1) N)},
// O(1) per offset instead of a fresh O(N) Goertzel. Updates (best_corr, best_offset) when an
// offset beats best_corr (strictly), as the per-offset sweep did.
void fineSweep(SampleSpan x, size_t lo, size_t hi, uint32_t spp, int bands, float offset_hz,
               float& best_corr, size_t& best_offset) {
    Cd X[kMaxBatch], ph_out[kMaxBatch], ph_in[kMaxBatch], rot[kMaxBatch];
    for (uint32_t i = 0; i < kCostasSymbols; ++i) {
        const size_t a = lo + static_cast<size_t>(i) * spp;
        double om[kMaxBands];
        for (int b = 0; b < bands; ++b) om[b] = omegaOf(toneHz(kCostasPattern[i], b, offset_hz));
        std::vector<Cd> tmp[kMaxBands];
        gridPrefixSums(x, a, 1, spp, om, bands, tmp);
        for (int b = 0; b < bands; ++b) {
            const int w = static_cast<int>(i) * bands + b;
            X[w] = tmp[b][1];
            ph_out[w] = std::polar(1.0, -om[b] * static_cast<double>(a));
            ph_in[w] = std::polar(1.0, -om[b] * static_cast<double>(a + spp));
            rot[w] = std::polar(1.0, -om[b]);
        }
    }
    for (size_t o = lo; o <= hi; ++o) {
        if (o > lo) {
            for (uint32_t i = 0; i < kCostasSymbols; ++i) {
                const size_t out_idx = o - 1 + static_cast<size_t>(i) * spp;
                const double x_out = static_cast<double>(x[out_idx]);
                const double x_in = static_cast<double>(x[out_idx + spp]);
                for (int b = 0; b < bands; ++b) {
                    const int w = static_cast<int>(i) * bands + b;
                    X[w] += x_in * ph_in[w] - x_out * ph_out[w];
                    ph_out[w] *= rot[w];
                    ph_in[w] *= rot[w];
                }
            }
        }
        const float c = costasStatistic(X, bands);
        if (c > best_corr) {
            best_corr = c;
            best_offset = o;
        }
    }
}

}  // namespace

int ToneBurstDetector::frequencyHypothesesPerSide(float halfwidth_hz, uint32_t symbol_ms) {
    const float step = frequencyHypothesisStepHz(symbol_ms);
    if (!(halfwidth_hz > 0.0f) || !(step > 0.0f)) return 0;
    const double j = std::ceil((static_cast<double>(halfwidth_hz) - 0.5 * step) / step);
    return static_cast<int>(std::max(0.0, j));
}

// ----- static helpers -----

std::vector<float> ToneBurstDetector::toneMagnitudes(SampleSpan window_samples,
                                                      size_t window_samples_count,
                                                      bool diversity, float offset_hz) {
    std::vector<float> mags;
    mags.reserve(kNumTones);
    for (uint32_t k = 0; k < kNumTones; ++k) {
        // Square-law (power) combining of the two subbands: the optimal non-coherent combiner
        // for equal-power branches.
        float power = goertzelPowerAt(window_samples, window_samples_count,
                                      toneHz(k, 0, offset_hz));
        if (diversity) {
            power += goertzelPowerAt(window_samples, window_samples_count,
                                     toneHz(k, 1, offset_hz));
        }
        // Use sqrt(power) = magnitude. Cheaper to compare on power, but the
        // tests want a "confidence ratio" so we want amplitude here.
        mags.push_back(std::sqrt(std::max(power, 0.0f)));
    }
    return mags;
}

uint8_t ToneBurstDetector::demapDibit(const std::vector<float>& magnitudes,
                                       float& confidence) {
    uint8_t best_idx = 0;
    float best = -1.0f;
    float second = -1.0f;
    for (uint8_t i = 0; i < magnitudes.size() && i < kNumTones; ++i) {
        const float m = magnitudes[i];
        if (m > best) {
            second = best;
            best = m;
            best_idx = i;
        } else if (m > second) {
            second = m;
        }
    }
    if (second <= 0.0f) {
        confidence = (best > 0.0f) ? 1e6f : 0.0f;
    } else {
        confidence = best / second;
    }
    return best_idx;
}

float ToneBurstDetector::costasCorrelationAtOffset(SampleSpan samples,
                                                    size_t num_samples,
                                                    size_t start_offset,
                                                    uint32_t samples_per_symbol,
                                                    bool diversity, float offset_hz) {
    // Sum of expected-tone Goertzel magnitudes across the 4 Costas symbols.
    // High when the actual tones at those positions match the expected
    // Costas pattern; near-noise-floor otherwise.
    float sum = 0.0f;
    for (uint32_t s = 0; s < kCostasSymbols; ++s) {
        const uint8_t expected_tone = kCostasPattern[s];
        const size_t window_start = start_offset + s * samples_per_symbol;
        if (window_start + samples_per_symbol > num_samples) return 0.0f;
        float power = goertzelPowerAt(samples + window_start, samples_per_symbol,
                                      toneHz(expected_tone, 0, offset_hz));
        if (diversity) {
            power += goertzelPowerAt(samples + window_start, samples_per_symbol,
                                     toneHz(expected_tone, 1, offset_hz));
        }
        sum += std::sqrt(std::max(power, 0.0f));
    }
    return sum;
}

// ----- decode aligned -----

DecodeAlignedResult ToneBurstDetector::decodeAlignedAt(SampleSpan samples,
                                                        size_t num_samples,
                                                        uint32_t symbol_ms,
                                                        float offset_hz) {
    DecodeAlignedResult result;
    const uint32_t samples_per_symbol = (kSampleRate * symbol_ms) / 1000u;
    const size_t needed = static_cast<size_t>(kTotalSymbols) * samples_per_symbol;
    if (num_samples < needed) {
        return result;
    }

    // Skip Costas; demap the kPayloadSymbols (30) payload symbols.
    std::vector<uint8_t> dibits;
    dibits.reserve(kPayloadSymbols);
    result.per_symbol_confidence.reserve(kPayloadSymbols);
    float min_conf = 1e30f;

    for (uint32_t s = 0; s < kPayloadSymbols; ++s) {
        const size_t window_start =
            (static_cast<size_t>(kCostasSymbols) + s) * samples_per_symbol;
        const auto mags =
            toneMagnitudes(samples + window_start, samples_per_symbol, diversity_, offset_hz);
        float conf = 0.0f;
        const uint8_t dibit = demapDibit(mags, conf);
        dibits.push_back(dibit);
        result.per_symbol_confidence.push_back(conf);
        if (conf < min_conf) min_conf = conf;
    }
    result.min_confidence = min_conf;
    result.payload = decodePayloadDibits(dibits, result.stats);
    return result;
}

// ----- detect + decode (sweep for Costas) -----

DetectAndDecodeResult ToneBurstDetector::detectAndDecode(SampleSpan samples,
                                                          size_t num_samples,
                                                          uint32_t symbol_ms,
                                                          uint32_t sweep_step_samples,
                                                          const ToneBurstAckAcceptancePredicate&
                                                              acceptance_predicate,
                                                          float min_candidate_peak) {
    DetectAndDecodeResult result;
    const uint32_t samples_per_symbol = (kSampleRate * symbol_ms) / 1000u;
    const size_t total_needed =
        static_cast<size_t>(kTotalSymbols) * samples_per_symbol;
    if (num_samples < total_needed) return result;

    // Coarse sweep: collect ALL correlation values above a noise-floor
    // threshold. We can't just take the GLOBAL peak — a Costas pattern can
    // alias against any 4-tone subsequence in the payload (the payload is
    // random-ish 4-FSK dibits; some 4-tuple of dibits will inevitably look
    // like the Costas pattern {0, 1, 3, 2}). So we collect candidates and
    // verify each by CRC.
    const size_t max_offset = num_samples - total_needed;
    if (sweep_step_samples == 0) sweep_step_samples = 1;
    const size_t step = sweep_step_samples;
    const int bands = diversity_ ? 2 : 1;

    // Carrier-offset hypotheses around the prior (see setFrequencyPrior).
    const int per_side = frequencyHypothesesPerSide(freq_halfwidth_hz_, symbol_ms);
    const float hyp_step = frequencyHypothesisStepHz(symbol_ms);
    std::vector<float> hyps;
    hyps.reserve(static_cast<size_t>(2 * per_side + 1));
    for (int j = -per_side; j <= per_side; ++j) {
        hyps.push_back(freq_prior_hz_ + static_cast<float>(j) * hyp_step);
    }
    result.frequency_hypotheses = static_cast<int>(hyps.size());

    // COARSE sweep: offsets 0, step, ..., <= max_offset, each scored by the Costas statistic
    // maximized over the offset hypotheses. All window edges (t + i N) lie on the grid
    // g = gcd(step, N), so one grid-prefix-sum pass per frequency serves every offset.
    struct Candidate {
        size_t offset;
        float corr;
        int hyp;
    };
    const size_t n_offsets = max_offset / step + 1;
    const size_t g = std::gcd(step, static_cast<size_t>(samples_per_symbol));
    const size_t span = (n_offsets - 1) * step + kCostasSymbols * samples_per_symbol;
    const size_t step_grid = step / g;
    const size_t sym_grid = samples_per_symbol / g;
    std::vector<Candidate> candidates(n_offsets);
    std::vector<Cd> prefix[kMaxBatch];
    for (size_t h = 0; h < hyps.size(); ++h) {
        double om[kMaxBatch];
        for (uint32_t i = 0; i < kCostasSymbols; ++i) {
            for (int b = 0; b < bands; ++b) {
                om[i * bands + b] = omegaOf(toneHz(kCostasPattern[i], b, hyps[h]));
            }
        }
        gridPrefixSums(samples, 0, span / g, g, om, static_cast<int>(kCostasSymbols) * bands,
                       prefix);
        for (size_t k = 0; k < n_offsets; ++k) {
            const size_t t_grid = k * step_grid;
            Cd X[kMaxBatch];
            for (uint32_t i = 0; i < kCostasSymbols; ++i) {
                for (int b = 0; b < bands; ++b) {
                    const auto& P = prefix[i * bands + b];
                    X[i * bands + b] = P[t_grid + (i + 1) * sym_grid] - P[t_grid + i * sym_grid];
                }
            }
            const float c = costasStatistic(X, bands);
            if (h == 0 || c > candidates[k].corr) {
                candidates[k] = Candidate{k * step, c, static_cast<int>(h)};
            }
        }
    }
    // Which timing candidates are worth a CRC check. NOT "3x the window average" (the rule
    // until 2026-11-08): when the ACK fills most of the window — the monitor's 12 ms tail
    // window is only 27 % longer than the burst — the average is the ACK's own energy, 3x it
    // exceeds every alignment but the global maximum, and a payload that contains the Costas
    // sequence (27 four-symbol windows x 1/256: ~10 % of payloads) lost the ACK whenever its
    // alias outscored the preamble (BUG-TONE-ACK-CLEAN-MISS: group 42 carries 0 1 3 2 at
    // symbols 5-8; decoded 5 symbols late, CRC fail, every pass). Candidates are now tried
    // strongest first while above the caller's absolute floor, bounded by the distinct-
    // candidate budget below; on noise the monitor's floor (5 % of the ideal peak, noise sits
    // near 1 %) admits none, so the CRC/false-alarm exposure per pass does not grow.
    const float candidate_threshold = std::max(min_candidate_peak, 1e-6f);

    // Timing hypotheses = the LOCAL MAXIMA of the coarse Costas statistic, one per correlation
    // lobe. The lobe is +-1 symbol wide; a payload alias of the Costas sequence is >= 4 symbols
    // from the preamble (symbols 1-3 ARE the Costas), so lobes never merge. Ranking every coarse
    // offset instead let the slope points of the strongest lobe — each refining to a distinct
    // offset 32 samples from its neighbour — spend the distinct-candidate budget before a
    // slightly weaker preamble lobe was reached (the alias regression test: 1 of 4 lost).
    {
        std::vector<Candidate> maxima;
        maxima.reserve(64);
        for (size_t k = 0; k < candidates.size(); ++k) {
            const float c = candidates[k].corr;
            const bool ge_left = (k == 0) || c >= candidates[k - 1].corr;
            const bool gt_right = (k + 1 == candidates.size()) || c > candidates[k + 1].corr;
            if (ge_left && gt_right) maxima.push_back(candidates[k]);
        }
        candidates.swap(maxima);
    }
    // Strongest first (stable: equal scores keep ascending-offset order).
    std::stable_sort(candidates.begin(), candidates.end(),
                     [](const Candidate& a, const Candidate& b) { return a.corr > b.corr; });

    // Try the top-K distinct timing candidates. K=8 covers the ordinary case where
    // the global peak is a payload alias and the real burst is somewhere lower in the
    // sorted list. If a candidate passes the wire CRC but is rejected by the protocol
    // predicate, widen the bounded search: otherwise one strong, impossible CRC
    // collision can hide a weaker valid ACK later in the same receive window.
    constexpr size_t kNormalCandidateBudget = 8;
    constexpr size_t kAfterSemanticRejectBudget = 128;
    size_t candidate_budget = kNormalCandidateBudget;
    size_t distinct_candidates_tried = 0;
    std::vector<size_t> refined_offsets_tried;
    refined_offsets_tried.reserve(kAfterSemanticRejectBudget);
    float global_peak = 0.0f;
    size_t global_offset = 0;
    float global_hyp_hz = hyps[static_cast<size_t>(per_side)];
    DecodeAlignedResult best_decode;  // empty by default
    size_t best_decode_offset = 0;
    float best_decode_hyp_hz = global_hyp_hz;

    for (size_t k = 0; k < candidates.size(); ++k) {
        size_t offset = candidates[k].offset;
        const float corr_coarse = candidates[k].corr;
        const float hyp_hz = hyps[static_cast<size_t>(candidates[k].hyp)];
        if (!(corr_coarse > candidate_threshold)) break;
        if (corr_coarse > global_peak) {
            global_peak = corr_coarse;
            global_offset = offset;
            global_hyp_hz = hyp_hz;
        }

        // Fine sweep around this candidate (at its winning offset hypothesis).
        float best_corr = corr_coarse;
        size_t best_offset = offset;
        if (step > 1) {
            const size_t lo = (offset >= step) ? offset - step : 0;
            const size_t hi = std::min(offset + step, max_offset);
            fineSweep(samples, lo, hi, samples_per_symbol, bands, hyp_hz, best_corr,
                      best_offset);
        }

        // Adjacent coarse sweep points commonly refine onto the same Costas lobe.
        // Do not let those aliases consume the bounded candidate budget (especially
        // after a semantic rejection, where they would repeatedly present the same
        // impossible payload and starve a later valid burst).
        // Keep this guard deliberately narrow. A false CRC can be a wrong timing
        // hypothesis *inside the real ACK itself*; suppressing the whole burst (or
        // even a large fraction of a symbol) would also suppress the correct timing
        // candidate we are trying to recover.
        const size_t duplicate_guard =
            std::max<size_t>(1u, static_cast<size_t>(sweep_step_samples) / 2u);
        const bool already_tried = std::any_of(
            refined_offsets_tried.begin(), refined_offsets_tried.end(),
            [best_offset, duplicate_guard](size_t prior) {
                const size_t delta = (best_offset > prior)
                    ? best_offset - prior : prior - best_offset;
                return delta <= duplicate_guard;
            });
        if (already_tried) continue;
        if (distinct_candidates_tried >= candidate_budget) break;
        refined_offsets_tried.push_back(best_offset);
        ++distinct_candidates_tried;

        // Try to decode at the refined offset. First CRC + protocol-plausibility
        // pass wins. A CRC-valid but impossible payload is treated like a failed
        // timing hypothesis and the search continues.
        const size_t remaining = num_samples - best_offset;
        auto decode = decodeAlignedAt(samples + best_offset, remaining, symbol_ms, hyp_hz);
        if (decode.stats.crc_ok) {
            if (acceptance_predicate && decode.payload.has_value() &&
                !acceptance_predicate(*decode.payload)) {
                ++result.semantic_rejections;
                if (!result.strongest_rejected_payload.has_value() ||
                    best_corr > result.strongest_rejected_correlation_peak) {
                    result.strongest_rejected_payload = *decode.payload;
                    result.strongest_rejected_offset_samples = best_offset;
                    result.strongest_rejected_correlation_peak = best_corr;
                    result.strongest_rejected_min_confidence = decode.min_confidence;
                    result.strongest_rejected_hamming_corrected_blocks =
                        decode.stats.hamming_corrected_blocks;
                }
                candidate_budget = kAfterSemanticRejectBudget;
                continue;
            }
            best_decode = std::move(decode);
            best_decode_offset = best_offset;
            best_decode_hyp_hz = hyp_hz;
            // Report the accepted candidate's own peak. A stronger rejected
            // candidate is diagnostic evidence, not signal energy belonging to this
            // ACK and must not inflate the monitor's peak gate/readout.
            global_peak = best_corr;
            global_offset = best_offset;
            break;
        }
    }

    result.detected_offset_samples = best_decode.stats.crc_ok ? best_decode_offset : global_offset;
    result.detected_frequency_offset_hz =
        best_decode.stats.crc_ok ? best_decode_hyp_hz : global_hyp_hz;
    result.costas_correlation_peak = global_peak;
    result.decode = std::move(best_decode);
    result.sync_acquired = result.decode.stats.crc_ok;
    return result;
}

}  // namespace tone_burst_ack
}  // namespace waveform
}  // namespace ultra
