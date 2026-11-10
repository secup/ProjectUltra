#include "frame_v2.hpp"
#include "ultra/fast_profile.hpp"
#include "ultra/fec.hpp"  // LDPC encoder/decoder + ChannelInterleaver
#include "../fec/frame_interleaver.hpp"  // Frame-level interleaving
#include "../fec/ldpc_codec.hpp"  // For getRecommendedIterations
#include "../fec/turbo_codec.hpp"  // One turbo codeword per OFDM data frame
#include "ultra/logging.hpp"  // LOG_MODEM
#include "ultra/timing_profiler.hpp"
#include <cstring>
#include <algorithm>
#include <array>
#include <bit>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <memory>
#include <random>

namespace ultra {
namespace protocol {

namespace {

constexpr float kFixedFrameDefaultMinSumFactor = 0.9375f;
constexpr std::array<float, 4> kFixedFrameRetryFactors = {
    0.875f, 0.75f, 0.625f, 0.5f};
constexpr std::array<float, 5> kFixedFramePerturbSigmas = {
    0.3f, 0.7f, 1.0f, 1.5f, 2.0f};
constexpr std::array<float, 5> kFixedFramePerturbFactors = {
    0.75f, 0.625f, 0.875f, 0.75f, 0.625f};

int codeRateCacheIndex(CodeRate rate) {
    switch (rate) {
        case CodeRate::R1_4: return 0;
        case CodeRate::R1_3: return 1;
        case CodeRate::R1_2: return 2;
        case CodeRate::R2_3: return 3;
        case CodeRate::R3_4: return 4;
        case CodeRate::R5_6: return 5;
        case CodeRate::R7_8: return 6;
        default: return 2;
    }
}

LDPCEncoder& fixedFrameEncoderForRate(CodeRate rate, int lifting_z) {
    struct EncoderCacheEntry {
        CodeRate rate = CodeRate::AUTO;
        int lifting_z = 0;
        std::unique_ptr<LDPCEncoder> encoder;
    };

    // Expanding the QC-LDPC matrix is substantially more expensive at Z=81.
    // A long-code burst previously paid that setup cost once per logical frame,
    // delaying five frames by roughly half a second on the live Pi path.  Keep
    // one encoder per rate/lifting geometry and reuse it on this worker thread.
    lifting_z = (lifting_z == 81) ? 81 : 27;
    constexpr size_t kLiftingVariants = 2;
    thread_local std::array<EncoderCacheEntry, 7 * kLiftingVariants> cache;
    const size_t cache_index =
        static_cast<size_t>(codeRateCacheIndex(rate)) * kLiftingVariants +
        static_cast<size_t>(lifting_z == 81 ? 1 : 0);
    EncoderCacheEntry& entry = cache[cache_index];
    if (!entry.encoder || entry.rate != rate || entry.lifting_z != lifting_z) {
        entry.encoder = std::make_unique<LDPCEncoder>(rate, lifting_z);
        entry.rate = rate;
        entry.lifting_z = lifting_z;
    }

    LDPCEncoder& encoder = *entry.encoder;
    encoder.setRate(rate);
    return encoder;
}

LDPCDecoder& fixedFrameDecoderForRate(CodeRate rate, int lifting_z) {
    struct DecoderCacheEntry {
        CodeRate rate = CodeRate::AUTO;
        int lifting_z = 0;
        std::unique_ptr<LDPCDecoder> decoder;
    };

    // Matrix expansion is expensive (tens of milliseconds on the Pi).  The
    // legacy Z=27 path has always reused a per-thread decoder, but the first
    // Z=81 implementation rebuilt its 3x-larger graph for every logical frame.
    // On the live IONOS path that pushed the receive worker behind real time and
    // caused sync load-shedding.  Cache both supported lifting geometries with
    // the rate as part of the identity; retry code may mutate only runtime
    // knobs, which are reset below on every checkout.
    lifting_z = (lifting_z == 81) ? 81 : 27;
    constexpr size_t kLiftingVariants = 2;
    thread_local std::array<DecoderCacheEntry, 7 * kLiftingVariants> cache;
    const size_t cache_index =
        static_cast<size_t>(codeRateCacheIndex(rate)) * kLiftingVariants +
        static_cast<size_t>(lifting_z == 81 ? 1 : 0);
    DecoderCacheEntry& entry = cache[cache_index];
    if (!entry.decoder || entry.rate != rate || entry.lifting_z != lifting_z) {
        entry.decoder = std::make_unique<LDPCDecoder>(rate, lifting_z);
        entry.rate = rate;
        entry.lifting_z = lifting_z;
    }

    LDPCDecoder& decoder = *entry.decoder;
    decoder.setRate(rate);
    decoder.setMaxIterations(fec::LDPCCodec::getRecommendedIterations(rate));
    decoder.setMinSumFactor(kFixedFrameDefaultMinSumFactor);
    return decoder;
}

// Diagnostic fresh-shadow decoding must not mutate the production decoder's
// min-sum state or last-result telemetry. Keep a physically separate cache;
// it is instantiated only when the default-OFF shadow experiment is enabled.
LDPCDecoder& shadowFixedFrameDecoderForRate(CodeRate rate, int lifting_z) {
    struct DecoderCacheEntry {
        CodeRate rate = CodeRate::AUTO;
        int lifting_z = 0;
        std::unique_ptr<LDPCDecoder> decoder;
    };

    lifting_z = (lifting_z == 81) ? 81 : 27;
    constexpr size_t kLiftingVariants = 2;
    thread_local std::array<DecoderCacheEntry, 7 * kLiftingVariants> cache;
    const size_t cache_index =
        static_cast<size_t>(codeRateCacheIndex(rate)) * kLiftingVariants +
        static_cast<size_t>(lifting_z == 81 ? 1 : 0);
    DecoderCacheEntry& entry = cache[cache_index];
    if (!entry.decoder || entry.rate != rate || entry.lifting_z != lifting_z) {
        entry.decoder = std::make_unique<LDPCDecoder>(rate, lifting_z);
        entry.rate = rate;
        entry.lifting_z = lifting_z;
    }

    LDPCDecoder& decoder = *entry.decoder;
    decoder.setRate(rate);
    decoder.setMaxIterations(fec::LDPCCodec::getRecommendedIterations(rate));
    decoder.setMinSumFactor(kFixedFrameDefaultMinSumFactor);
    return decoder;
}

bool harqShadowFreshEnabled() {
    // Deliberately not cached: diagnostic tests and one-process harnesses run
    // OFF/ON arms sequentially. A static first-read would make arm order alter
    // behavior and invalidate the comparison.
    const char* value = std::getenv("ULTRA_HARQ_SHADOW_FRESH");
    return value && value[0] != '\0' &&
           !(value[0] == '0' && value[1] == '\0');
}

bool harqDebugLogEnabled() {
    static const bool enabled = [] {
        const char* value = std::getenv("ULTRA_HARQ_DEBUG_LOG");
        return value && value[0] != '\0' && !(value[0] == '0' && value[1] == '\0');
    }();
    return enabled;
}

int harqDebugFilter(const char* name) {
    const char* value = std::getenv(name);
    if (!value || value[0] == '\0') {
        return -1;
    }
    char* end = nullptr;
    const long parsed = std::strtol(value, &end, 0);
    return (end && *end == '\0') ? static_cast<int>(parsed) : -1;
}

bool harqDebugKeySelected(const fec::SoftCombineBuffer::Key& key) {
    if (!harqDebugLogEnabled()) {
        return false;
    }
    const int seq_filter = harqDebugFilter("ULTRA_HARQ_DEBUG_SEQ");
    if (seq_filter >= 0 && static_cast<int>(key.seq) != seq_filter) {
        return false;
    }
    const int cw_filter = harqDebugFilter("ULTRA_HARQ_DEBUG_CW");
    if (cw_filter >= 0 && static_cast<int>(key.cw_index) != cw_filter) {
        return false;
    }
    return true;
}

float meanAbsLlr(const std::vector<float>& llrs) {
    if (llrs.empty()) {
        return 0.0f;
    }
    double sum = 0.0;
    for (float llr : llrs) {
        sum += std::abs(llr);
    }
    return static_cast<float>(sum / static_cast<double>(llrs.size()));
}

uint32_t fixedFramePerturbationSeed(const std::vector<float>& llrs) {
    uint32_t data_hash = 0;
    for (size_t j = 0; j < std::min(llrs.size(), size_t(16)); ++j) {
        const uint32_t bits = std::bit_cast<uint32_t>(llrs[j]);
        data_hash ^=
            bits + 0x9e3779b9u + (data_hash << 6) + (data_hash >> 2);
    }
    return data_hash;
}

struct FullScheduleDecodeResult {
    std::vector<uint8_t> decoded;
    bool success = false;
    int iterations = 0;
    int unsatisfied_checks = -1;
    bool used_perturbation = false;
    float winning_factor = kFixedFrameDefaultMinSumFactor;
    float winning_sigma = 0.0f;
    int attempts = 1;
};

// Decode one observation with the exact production search budget: primary,
// four deterministic factor retries, then five deterministic perturbations.
// Fresh rescue and diagnostic shadow use this common implementation so their
// verdict cannot drift from what the uncombined production input would get.
FullScheduleDecodeResult decodeWithFullFixedFrameSchedule(
    LDPCDecoder& decoder, const std::vector<float>& llrs, bool profile_timing) {
    FullScheduleDecodeResult result;
    auto decode_once = [&](const std::vector<float>& input) {
        if (profile_timing) {
            ultra::timing::ScopedTimer timer(
                ultra::timing::globalDecoderProfile().ldpc_cw_total);
            return decoder.decodeSoft(input);
        }
        return decoder.decodeSoft(input);
    };

    decoder.setMinSumFactor(kFixedFrameDefaultMinSumFactor);
    result.decoded = decode_once(llrs);
    result.success = decoder.lastDecodeSuccess();
    result.iterations = decoder.lastIterations();
    result.unsatisfied_checks = decoder.lastUnsatisfiedChecks();

    for (float factor : kFixedFrameRetryFactors) {
        if (result.success) {
            break;
        }
        decoder.setMinSumFactor(factor);
        result.decoded = decode_once(llrs);
        ++result.attempts;
        result.success = decoder.lastDecodeSuccess();
        result.iterations = decoder.lastIterations();
        result.unsatisfied_checks = decoder.lastUnsatisfiedChecks();
        if (result.success) {
            result.winning_factor = factor;
        }
    }

    if (!result.success) {
        const uint32_t data_hash = fixedFramePerturbationSeed(llrs);
        for (size_t retry = 0; retry < kFixedFramePerturbSigmas.size(); ++retry) {
            decoder.setMinSumFactor(kFixedFramePerturbFactors[retry]);
            std::mt19937 rng(data_hash + static_cast<uint32_t>(retry) * 997u +
                             static_cast<uint32_t>(retry) * 31u);
            std::normal_distribution<float> noise(
                0.0f, kFixedFramePerturbSigmas[retry]);
            auto perturbed = llrs;
            for (float& llr : perturbed) {
                llr += noise(rng);
            }
            result.decoded = decode_once(perturbed);
            ++result.attempts;
            result.success = decoder.lastDecodeSuccess();
            result.iterations = decoder.lastIterations();
            result.unsatisfied_checks = decoder.lastUnsatisfiedChecks();
            if (result.success) {
                result.used_perturbation = true;
                result.winning_factor = kFixedFramePerturbFactors[retry];
                result.winning_sigma = kFixedFramePerturbSigmas[retry];
                break;
            }
        }
    }

    decoder.setMinSumFactor(kFixedFrameDefaultMinSumFactor);
    return result;
}

bool fixedFramePayloadIsValid(const Bytes& frame_data) {
    if (frame_data.empty()) {
        return false;
    }
    const auto header = v2::parseHeader(frame_data);
    if (!header.valid) {
        return false;
    }
    return header.is_control
               ? v2::ControlFrame::deserialize(frame_data).has_value()
               : v2::DataFrame::deserialize(frame_data).has_value();
}

struct LlrAbsSummary {
    float mean = 0.0f;
    float min = 0.0f;
    float p10 = 0.0f;
    float p50 = 0.0f;
    float p90 = 0.0f;
};

LlrAbsSummary summarizeAbsLlrs(const std::vector<float>& llrs) {
    LlrAbsSummary out;
    if (llrs.empty()) {
        return out;
    }

    std::vector<float> abs_values;
    abs_values.reserve(llrs.size());
    double sum = 0.0;
    float min_value = std::numeric_limits<float>::max();
    for (float llr : llrs) {
        const float a = std::abs(llr);
        abs_values.push_back(a);
        sum += static_cast<double>(a);
        min_value = std::min(min_value, a);
    }

    std::sort(abs_values.begin(), abs_values.end());
    auto percentile = [&](float q) -> float {
        if (abs_values.empty()) {
            return 0.0f;
        }
        const float pos = q * static_cast<float>(abs_values.size() - 1);
        const size_t idx = static_cast<size_t>(std::round(pos));
        return abs_values[std::min(idx, abs_values.size() - 1)];
    };

    out.mean = static_cast<float>(sum / static_cast<double>(llrs.size()));
    out.min = min_value;
    out.p10 = percentile(0.10f);
    out.p50 = percentile(0.50f);
    out.p90 = percentile(0.90f);
    return out;
}

} // namespace

// ============================================================================
// Shared Protocol Types Implementation
// ============================================================================

const char* waveformModeToString(WaveformMode mode) {
    switch (mode) {
        case WaveformMode::OTFS_EQ:    return "OTFS-EQ";
        case WaveformMode::OTFS_RAW:   return "OTFS-RAW";
        case WaveformMode::MFSK:       return "MFSK";
        case WaveformMode::MC_DPSK:    return "MC-DPSK";
        case WaveformMode::OFDM_CHIRP: return "OFDM-CHIRP";
        case WaveformMode::OFDM_NARROW: return "OFDM-NARROW";
        case WaveformMode::AUTO:       return "AUTO";
        default:                       return "UNKNOWN";
    }
}

Bytes ChannelReport::encode() const {
    Bytes data(5);
    // SNR: 0-50 dB mapped to 0-250 (0.2 dB resolution)
    data[0] = static_cast<uint8_t>(std::min(250.0f, std::max(0.0f, snr_db * 5.0f)));
    // Delay spread: 0-25 ms mapped to 0-250 (0.1 ms resolution)
    data[1] = static_cast<uint8_t>(std::min(250.0f, std::max(0.0f, delay_spread_ms * 10.0f)));
    // Doppler: 0-25 Hz mapped to 0-250 (0.1 Hz resolution)
    data[2] = static_cast<uint8_t>(std::min(250.0f, std::max(0.0f, doppler_spread_hz * 10.0f)));
    // Recommended mode
    data[3] = static_cast<uint8_t>(recommended_mode);
    // Capabilities
    data[4] = capabilities;
    return data;
}

ChannelReport ChannelReport::decode(const Bytes& data) {
    ChannelReport report;
    if (data.size() >= 5) {
        report.snr_db = static_cast<float>(data[0]) / 5.0f;
        report.delay_spread_ms = static_cast<float>(data[1]) / 10.0f;
        report.doppler_spread_hz = static_cast<float>(data[2]) / 10.0f;
        report.recommended_mode = static_cast<WaveformMode>(data[3]);
        report.capabilities = data[4];
    }
    return report;
}

const char* ChannelReport::getConditionName() const {
    if (snr_db >= 35.0f && delay_spread_ms < 1.0f && doppler_spread_hz < 1.0f) {
        return "Excellent";
    } else if (snr_db >= 28.0f && delay_spread_ms < 2.0f && doppler_spread_hz < 2.0f) {
        return "Good";
    } else if (snr_db >= 20.0f) {
        return "Moderate";
    } else if (snr_db >= 13.0f) {
        return "Poor";
    } else {
        return "Flutter";
    }
}

namespace v2 {

// ============================================================================
// Callsign hashing (DJB2, 24-bit)
// ============================================================================
uint32_t hashCallsign(const std::string& callsign) {
    uint32_t hash = 5381;
    for (char c : callsign) {
        hash = ((hash << 5) + hash) ^ static_cast<uint8_t>(std::toupper(c));
    }
    return hash & 0xFFFFFF;  // 24 bits
}

// ============================================================================
// Frame type to string
// ============================================================================
const char* frameTypeToString(FrameType type) {
    switch (type) {
        case FrameType::PING:        return "PING";
        case FrameType::PONG:        return "PONG";
        case FrameType::PROBE:       return "PROBE";
        case FrameType::PROBE_ACK:   return "PROBE_ACK";
        case FrameType::CONNECT:     return "CONNECT";
        case FrameType::CONNECT_ACK: return "CONNECT_ACK";
        case FrameType::CONNECT_NAK: return "CONNECT_NAK";
        case FrameType::DISCONNECT:  return "DISCONNECT";
        case FrameType::KEEPALIVE:   return "KEEPALIVE";
        case FrameType::MODE_CHANGE: return "MODE_CHANGE";
        case FrameType::ACK:         return "ACK";
        case FrameType::NACK:        return "NACK";
        case FrameType::TURNOVER:    return "TURNOVER";
        case FrameType::TURN_REQUEST: return "TURN_REQUEST";
        case FrameType::FILE_CANCEL: return "FILE_CANCEL";
        case FrameType::BEACON:      return "BEACON";
        case FrameType::DATA:        return "DATA";
        case FrameType::DATA_START:  return "DATA_START";
        case FrameType::DATA_CONT:   return "DATA_CONT";
        case FrameType::DATA_END:    return "DATA_END";
        case FrameType::DATA_REPAIR: return "DATA_REPAIR";
        default:                     return "UNKNOWN";
    }
}

// ============================================================================
// CRC-16 CCITT (same as v1 for compatibility)
// ============================================================================
uint16_t ControlFrame::calculateCRC(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

// ============================================================================
// PHYMaskHeader implementation
// ============================================================================

Bytes PHYMaskHeader::serialize() const {
    Bytes out(SIZE, 0);

    out[0] = MAGIC0;
    out[1] = MAGIC1;
    out[2] = packVersionScheme(version, scheme);
    out[3] = flags;
    out[4] = payload_profile;
    out[5] = interleaver_id;
    out[6] = mask_count;
    out[7] = reserved;

    for (size_t i = 0; i < sizeof(active_mask); ++i) {
        out[8 + i] = static_cast<uint8_t>((active_mask >> (8 * i)) & 0xFF);
    }

    const uint16_t crc = ControlFrame::calculateCRC(out.data(), 16);
    const uint16_t inverted_crc = static_cast<uint16_t>(crc ^ 0xFFFF);
    out[16] = static_cast<uint8_t>((crc >> 8) & 0xFF);
    out[17] = static_cast<uint8_t>(crc & 0xFF);
    out[18] = static_cast<uint8_t>((inverted_crc >> 8) & 0xFF);
    out[19] = static_cast<uint8_t>(inverted_crc & 0xFF);

    return out;
}

bool PHYMaskHeader::validateFields() const {
    if (version != VERSION_V1 || scheme != SCHEME_BITMAP_INTERLEAVER_V1) {
        return false;
    }
    if (flags != 0 || reserved != 0) {
        return false;
    }
    if (interleaver_id != INTERLEAVER_CARRIER_LDPC_V1) {
        return false;
    }
    if (mask_count < MIN_MASK_COUNT || mask_count > MAX_MASK_COUNT) {
        return false;
    }
    if ((active_mask & ~ACTIVE_CARRIER_MASK) != 0) {
        return false;
    }
    return std::popcount(active_mask) == DATA_CARRIER_COUNT - mask_count;
}

bool PHYMaskHeader::validate(ByteSpan data) {
    return deserialize(data).has_value();
}

std::optional<PHYMaskHeader> PHYMaskHeader::deserialize(ByteSpan data) {
    if (data.size() < SIZE) {
        return std::nullopt;
    }

    if (data[0] != MAGIC0 || data[1] != MAGIC1) {
        return std::nullopt;
    }

    const uint16_t received_crc = (static_cast<uint16_t>(data[16]) << 8) | data[17];
    const uint16_t calculated_crc = ControlFrame::calculateCRC(data.data(), 16);
    if (received_crc != calculated_crc) {
        return std::nullopt;
    }

    const uint16_t received_inverted_crc = (static_cast<uint16_t>(data[18]) << 8) | data[19];
    if (received_inverted_crc != static_cast<uint16_t>(received_crc ^ 0xFFFF)) {
        return std::nullopt;
    }

    PHYMaskHeader h;
    h.version = static_cast<uint8_t>((data[2] >> 4) & 0x0F);
    h.scheme = static_cast<uint8_t>(data[2] & 0x0F);
    h.flags = data[3];
    h.payload_profile = data[4];
    h.interleaver_id = data[5];
    h.mask_count = data[6];
    h.reserved = data[7];
    h.active_mask = 0;
    for (size_t i = 0; i < sizeof(h.active_mask); ++i) {
        h.active_mask |= static_cast<uint64_t>(data[8 + i]) << (8 * i);
    }
    h.crc16 = received_crc;
    h.inverted_crc16 = received_inverted_crc;

    if (!h.validateFields()) {
        return std::nullopt;
    }
    return h;
}

// ============================================================================
// ControlFrame implementation
// ============================================================================

ControlFrame ControlFrame::makeProbe(const std::string& src, const std::string& dst) {
    ControlFrame f;
    f.type = FrameType::PROBE;
    f.flags = Flags::VERSION_V2;
    f.seq = 0;
    f.src_hash = hashCallsign(src);
    f.dst_hash = hashCallsign(dst);
    std::memset(f.payload, 0, PAYLOAD_SIZE);
    return f;
}

ControlFrame ControlFrame::makeProbeAck(const std::string& src, const std::string& dst,
                                         uint8_t snr_db, uint8_t recommended_rate) {
    ControlFrame f;
    f.type = FrameType::PROBE_ACK;
    f.flags = Flags::VERSION_V2;
    f.seq = 0;
    f.src_hash = hashCallsign(src);
    f.dst_hash = hashCallsign(dst);
    std::memset(f.payload, 0, PAYLOAD_SIZE);
    f.payload[0] = snr_db;
    f.payload[1] = recommended_rate;
    return f;
}

ControlFrame ControlFrame::makeAck(const std::string& src, const std::string& dst, uint16_t seq) {
    ControlFrame f;
    f.type = FrameType::ACK;
    f.flags = Flags::VERSION_V2;
    f.seq = seq;
    f.src_hash = hashCallsign(src);
    f.dst_hash = hashCallsign(dst);
    std::memset(f.payload, 0, PAYLOAD_SIZE);
    return f;
}

ControlFrame ControlFrame::makeNack(const std::string& src, const std::string& dst,
                                     uint16_t seq, uint32_t cw_bitmap) {
    ControlFrame f;
    f.type = FrameType::NACK;
    f.flags = Flags::VERSION_V2;
    f.seq = seq;
    f.src_hash = hashCallsign(src);
    f.dst_hash = hashCallsign(dst);

    NackPayload np;
    np.frame_seq = seq;
    np.cw_bitmap = cw_bitmap;
    np.encode(f.payload);

    return f;
}

ControlFrame ControlFrame::makeTurnover(const std::string& src, const std::string& dst) {
    ControlFrame f;
    f.type = FrameType::TURNOVER;
    f.flags = Flags::VERSION_V2;
    f.seq = 0;
    f.src_hash = hashCallsign(src);
    f.dst_hash = hashCallsign(dst);
    std::memset(f.payload, 0, PAYLOAD_SIZE);
    return f;
}

ControlFrame ControlFrame::makeTurnRequest(const std::string& src, const std::string& dst) {
    ControlFrame f;
    f.type = FrameType::TURN_REQUEST;
    f.flags = Flags::VERSION_V2;
    f.seq = 0;
    f.src_hash = hashCallsign(src);
    f.dst_hash = hashCallsign(dst);
    std::memset(f.payload, 0, PAYLOAD_SIZE);
    return f;
}

ControlFrame ControlFrame::makeFileCancel(const std::string& src, const std::string& dst) {
    ControlFrame f;
    f.type = FrameType::FILE_CANCEL;
    f.flags = Flags::VERSION_V2;
    f.seq = 0;
    f.src_hash = hashCallsign(src);
    f.dst_hash = hashCallsign(dst);
    std::memset(f.payload, 0, PAYLOAD_SIZE);
    return f;
}

ControlFrame ControlFrame::makeBurstHeader(const std::string& src, const std::string& dst,
                                           uint16_t seq, uint8_t group_size,
                                           uint8_t cw_per_frame, Modulation mod, CodeRate rate,
                                           uint8_t interleave_flags,
                                           uint8_t lifting_z) {
    ControlFrame f;
    f.type = FrameType::BURST_HEADER;
    f.flags = Flags::VERSION_V2;
    f.seq = seq;
    f.src_hash = hashCallsign(src);
    f.dst_hash = hashCallsign(dst);
    f.payload[0] = group_size;
    f.payload[1] = cw_per_frame;
    f.payload[2] = static_cast<uint8_t>(mod);
    f.payload[3] = static_cast<uint8_t>(rate);
    f.payload[4] = interleave_flags;
    // 2026-05-28 LDPC lifting Z for the data group. Wire-encoded as the literal
    // Z value (27 or 81); 0 = legacy "unspecified" (RX falls back to Z=27).
    f.payload[5] = (lifting_z == 81) ? 81 : (lifting_z == 27 ? 27 : 0);
    return f;
}

ControlFrame ControlFrame::makeGroupAck(const std::string& src, const std::string& dst,
                                        uint16_t group_seq, uint8_t quality_q) {
    ControlFrame f;
    f.type = FrameType::GROUP_ACK;
    f.flags = Flags::VERSION_V2;
    f.seq = group_seq;
    f.src_hash = hashCallsign(src);
    f.dst_hash = hashCallsign(dst);
    f.payload[0] = static_cast<uint8_t>(group_seq & 0xFF);
    f.payload[1] = static_cast<uint8_t>((group_seq >> 8) & 0xFF);
    f.payload[2] = quality_q;  // §14.36 decode-headroom feedback (0xFF = none)
    return f;
}

ControlFrame ControlFrame::makeGroupNack(const std::string& src, const std::string& dst,
                                         uint16_t group_seq) {
    ControlFrame f;
    f.type = FrameType::GROUP_NACK;
    f.flags = Flags::VERSION_V2;
    f.seq = group_seq;
    f.src_hash = hashCallsign(src);
    f.dst_hash = hashCallsign(dst);
    f.payload[0] = static_cast<uint8_t>(group_seq & 0xFF);
    f.payload[1] = static_cast<uint8_t>((group_seq >> 8) & 0xFF);
    return f;
}

ControlFrame ControlFrame::makeBeacon(const std::string& src) {
    ControlFrame f;
    f.type = FrameType::BEACON;
    f.flags = Flags::VERSION_V2;
    f.seq = 0;
    f.src_hash = hashCallsign(src);
    f.dst_hash = 0xFFFFFF;  // Broadcast
    std::memset(f.payload, 0, PAYLOAD_SIZE);
    return f;
}

ControlFrame ControlFrame::makeKeepalive(const std::string& src, const std::string& dst) {
    ControlFrame f;
    f.type = FrameType::KEEPALIVE;
    f.flags = Flags::VERSION_V2;
    f.seq = 0;
    f.src_hash = hashCallsign(src);
    f.dst_hash = hashCallsign(dst);
    std::memset(f.payload, 0, PAYLOAD_SIZE);
    return f;
}

ControlFrame ControlFrame::makeDisconnect(const std::string& src, const std::string& dst) {
    ControlFrame f;
    f.type = FrameType::DISCONNECT;
    f.flags = Flags::VERSION_V2;
    f.seq = DISCONNECT_SEQ;
    f.src_hash = hashCallsign(src);
    f.dst_hash = hashCallsign(dst);
    std::memset(f.payload, 0, PAYLOAD_SIZE);
    return f;
}

ControlFrame ControlFrame::makeModeChange(const std::string& src, const std::string& dst,
                                           uint16_t seq, Modulation new_mod, CodeRate new_rate,
                                           float snr_db, float fading_index, uint8_t reason,
                                           uint8_t cw_count,
                                           LadderRungId rung_id) {
    ControlFrame f;
    f.type = FrameType::MODE_CHANGE;
    f.flags = Flags::VERSION_V2;
    f.seq = seq;
    f.src_hash = hashCallsign(src);
    f.dst_hash = hashCallsign(dst);
    f.payload[0] = static_cast<uint8_t>(new_mod);
    f.payload[1] = static_cast<uint8_t>(new_rate);
    f.payload[2] = encodeSNR(snr_db);
    f.payload[3] = reason;
    f.payload[4] = encodeFadingIndex(fading_index);
    f.payload[5] = packCWCountAndRung(cw_count, rung_id);
    return f;
}

ControlFrame ControlFrame::makeModeChangeByHash(const std::string& src, uint32_t dst_hash,
                                                 uint16_t seq, Modulation new_mod, CodeRate new_rate,
                                                 float snr_db, float fading_index, uint8_t reason,
                                                 uint8_t cw_count,
                                                 LadderRungId rung_id) {
    ControlFrame f;
    f.type = FrameType::MODE_CHANGE;
    f.flags = Flags::VERSION_V2;
    f.seq = seq;
    f.src_hash = hashCallsign(src);
    f.dst_hash = dst_hash & 0xFFFFFF;
    f.payload[0] = static_cast<uint8_t>(new_mod);
    f.payload[1] = static_cast<uint8_t>(new_rate);
    f.payload[2] = encodeSNR(snr_db);
    f.payload[3] = reason;
    f.payload[4] = encodeFadingIndex(fading_index);
    f.payload[5] = packCWCountAndRung(cw_count, rung_id);
    return f;
}

ControlFrame ControlFrame::makeConnect(const std::string& src, const std::string& dst,
                                        uint8_t mode_capabilities, uint8_t preferred_mode) {
    ControlFrame f;
    f.type = FrameType::CONNECT;
    f.flags = Flags::VERSION_V2;
    f.seq = 0;
    f.src_hash = hashCallsign(src);
    f.dst_hash = hashCallsign(dst);
    std::memset(f.payload, 0, PAYLOAD_SIZE);
    f.payload[0] = mode_capabilities;  // Our supported modes
    f.payload[1] = preferred_mode;     // Our preferred mode
    return f;
}

ControlFrame ControlFrame::makeConnectAck(const std::string& src, const std::string& dst,
                                           uint8_t negotiated_mode) {
    ControlFrame f;
    f.type = FrameType::CONNECT_ACK;
    f.flags = Flags::VERSION_V2;
    f.seq = 0;
    f.src_hash = hashCallsign(src);
    f.dst_hash = hashCallsign(dst);
    std::memset(f.payload, 0, PAYLOAD_SIZE);
    f.payload[0] = negotiated_mode;
    return f;
}

ControlFrame ControlFrame::makeConnectNak(const std::string& src, const std::string& dst) {
    ControlFrame f;
    f.type = FrameType::CONNECT_NAK;
    f.flags = Flags::VERSION_V2;
    f.seq = 0;
    f.src_hash = hashCallsign(src);
    f.dst_hash = hashCallsign(dst);
    std::memset(f.payload, 0, PAYLOAD_SIZE);
    return f;
}

// Hash-based factory methods (for responding when callsign is unknown)
ControlFrame ControlFrame::makeProbeAckByHash(const std::string& src, uint32_t dst_hash,
                                               uint8_t snr_db, uint8_t recommended_rate) {
    ControlFrame f;
    f.type = FrameType::PROBE_ACK;
    f.flags = Flags::VERSION_V2;
    f.seq = 0;
    f.src_hash = hashCallsign(src);
    f.dst_hash = dst_hash & 0xFFFFFF;  // Ensure 24-bit
    std::memset(f.payload, 0, PAYLOAD_SIZE);
    f.payload[0] = snr_db;
    f.payload[1] = recommended_rate;
    return f;
}

ControlFrame ControlFrame::makeConnectAckByHash(const std::string& src, uint32_t dst_hash,
                                                 uint8_t negotiated_mode) {
    ControlFrame f;
    f.type = FrameType::CONNECT_ACK;
    f.flags = Flags::VERSION_V2;
    f.seq = 0;
    f.src_hash = hashCallsign(src);
    f.dst_hash = dst_hash & 0xFFFFFF;
    std::memset(f.payload, 0, PAYLOAD_SIZE);
    f.payload[0] = negotiated_mode;
    return f;
}

ControlFrame ControlFrame::makeConnectNakByHash(const std::string& src, uint32_t dst_hash) {
    ControlFrame f;
    f.type = FrameType::CONNECT_NAK;
    f.flags = Flags::VERSION_V2;
    f.seq = 0;
    f.src_hash = hashCallsign(src);
    f.dst_hash = dst_hash & 0xFFFFFF;
    std::memset(f.payload, 0, PAYLOAD_SIZE);
    return f;
}

ControlFrame ControlFrame::makeAckByHash(const std::string& src, uint32_t dst_hash, uint16_t seq) {
    ControlFrame f;
    f.type = FrameType::ACK;
    f.flags = Flags::VERSION_V2;
    f.seq = seq;
    f.src_hash = hashCallsign(src);
    f.dst_hash = dst_hash & 0xFFFFFF;
    std::memset(f.payload, 0, PAYLOAD_SIZE);
    return f;
}

ControlFrame ControlFrame::makeNackByHash(const std::string& src, uint32_t dst_hash,
                                           uint16_t seq, uint32_t cw_bitmap) {
    ControlFrame f;
    f.type = FrameType::NACK;
    f.flags = Flags::VERSION_V2;
    f.seq = seq;
    f.src_hash = hashCallsign(src);
    f.dst_hash = dst_hash & 0xFFFFFF;

    NackPayload np;
    np.frame_seq = seq;
    np.cw_bitmap = cw_bitmap;
    np.encode(f.payload);

    return f;
}

Bytes ControlFrame::serialize() const {
    Bytes out(SIZE);

    // Magic (2 bytes, big-endian)
    out[0] = (MAGIC_V2 >> 8) & 0xFF;
    out[1] = MAGIC_V2 & 0xFF;

    // Type (1 byte)
    out[2] = static_cast<uint8_t>(type);

    // Flags (1 byte)
    out[3] = flags;

    // Sequence (2 bytes, big-endian)
    out[4] = (seq >> 8) & 0xFF;
    out[5] = seq & 0xFF;

    // Source hash (3 bytes, big-endian)
    out[6] = (src_hash >> 16) & 0xFF;
    out[7] = (src_hash >> 8) & 0xFF;
    out[8] = src_hash & 0xFF;

    // Destination hash (3 bytes, big-endian)
    out[9] = (dst_hash >> 16) & 0xFF;
    out[10] = (dst_hash >> 8) & 0xFF;
    out[11] = dst_hash & 0xFF;

    // Payload (6 bytes)
    std::memcpy(out.data() + 12, payload, PAYLOAD_SIZE);

    // CRC16 (2 bytes, big-endian) - over bytes 0-17
    uint16_t crc = calculateCRC(out.data(), SIZE - 2);
    out[18] = (crc >> 8) & 0xFF;
    out[19] = crc & 0xFF;

    return out;
}

std::optional<ControlFrame> ControlFrame::deserialize(ByteSpan data) {
    if (data.size() < SIZE) {
        return std::nullopt;
    }

    // Check magic
    uint16_t magic = (static_cast<uint16_t>(data[0]) << 8) | data[1];
    if (magic != MAGIC_V2) {
        return std::nullopt;
    }

    // Verify CRC
    uint16_t received_crc = (static_cast<uint16_t>(data[18]) << 8) | data[19];
    uint16_t calculated_crc = calculateCRC(data.data(), SIZE - 2);
    if (received_crc != calculated_crc) {
        return std::nullopt;
    }

    ControlFrame f;
    f.type = static_cast<FrameType>(data[2]);
    f.flags = data[3];
    f.seq = (static_cast<uint16_t>(data[4]) << 8) | data[5];
    f.src_hash = (static_cast<uint32_t>(data[6]) << 16) |
                 (static_cast<uint32_t>(data[7]) << 8) |
                 data[8];
    f.dst_hash = (static_cast<uint32_t>(data[9]) << 16) |
                 (static_cast<uint32_t>(data[10]) << 8) |
                 data[11];
    std::memcpy(f.payload, data.data() + 12, PAYLOAD_SIZE);

    return f;
}

// ============================================================================
// DataFrame implementation
// ============================================================================

uint8_t DataFrame::calculateCodewords(size_t payload_size) {
    return calculateCodewords(payload_size, CodeRate::R1_4);
}

uint8_t DataFrame::calculateCodewords(size_t payload_size, CodeRate rate) {
    // Total frame size = header (17) + payload + frame_CRC (2)
    size_t total = HEADER_SIZE + payload_size + CRC_SIZE;

    size_t bytes_per_cw = getBytesPerCodeword(rate);
    if (total <= bytes_per_cw) {
        return 1;
    }

    size_t data_payload_size = bytes_per_cw - DATA_CW_HEADER_SIZE;
    size_t continuation_bytes = total - bytes_per_cw;
    size_t continuation_cw = (continuation_bytes + data_payload_size - 1) / data_payload_size;

    return static_cast<uint8_t>(1 + continuation_cw);
}

DataFrame DataFrame::makeData(const std::string& src, const std::string& dst,
                               uint16_t seq, const Bytes& data) {
    DataFrame f;
    f.type = FrameType::DATA;
    f.flags = Flags::VERSION_V2;
    f.seq = seq;
    f.src_hash = hashCallsign(src);
    f.dst_hash = hashCallsign(dst);
    f.payload = data;
    f.payload_len = static_cast<uint16_t>(data.size());
    f.total_cw = calculateCodewords(data.size());
    return f;
}

DataFrame DataFrame::makeData(const std::string& src, const std::string& dst,
                               uint16_t seq, const std::string& text) {
    Bytes data(text.begin(), text.end());
    return makeData(src, dst, seq, data);
}

DataFrame DataFrame::makeData(const std::string& src, const std::string& dst,
                               uint16_t seq, const Bytes& data, CodeRate cw1_rate) {
    DataFrame f;
    f.type = FrameType::DATA;
    f.flags = Flags::VERSION_V2;
    f.seq = seq;
    f.src_hash = hashCallsign(src);
    f.dst_hash = hashCallsign(dst);
    f.payload = data;
    f.payload_len = static_cast<uint16_t>(data.size());
    f.total_cw = calculateCodewords(data.size(), cw1_rate);
    return f;
}

DataFrame DataFrame::makeData(const std::string& src, const std::string& dst,
                               uint16_t seq, const std::string& text, CodeRate cw1_rate) {
    Bytes data(text.begin(), text.end());
    return makeData(src, dst, seq, data, cw1_rate);
}

Bytes DataFrame::serialize() const {
    // Total size = header + payload + CRC
    size_t total_size = HEADER_SIZE + payload.size() + CRC_SIZE;
    Bytes out(total_size);

    // Magic (2 bytes, big-endian)
    out[0] = (MAGIC_V2 >> 8) & 0xFF;
    out[1] = MAGIC_V2 & 0xFF;

    // Type (1 byte)
    out[2] = static_cast<uint8_t>(type);

    // Flags (1 byte)
    out[3] = flags;

    // Sequence (2 bytes, big-endian)
    out[4] = (seq >> 8) & 0xFF;
    out[5] = seq & 0xFF;

    // Source hash (3 bytes, big-endian)
    out[6] = (src_hash >> 16) & 0xFF;
    out[7] = (src_hash >> 8) & 0xFF;
    out[8] = src_hash & 0xFF;

    // Destination hash (3 bytes, big-endian)
    out[9] = (dst_hash >> 16) & 0xFF;
    out[10] = (dst_hash >> 8) & 0xFF;
    out[11] = dst_hash & 0xFF;

    // Total codewords (1 byte)
    out[12] = total_cw;

    // Payload length (2 bytes, big-endian)
    out[13] = (payload_len >> 8) & 0xFF;
    out[14] = payload_len & 0xFF;

    // Header CRC (2 bytes) - CRC of bytes 0-14
    uint16_t hcrc = ControlFrame::calculateCRC(out.data(), 15);
    out[15] = (hcrc >> 8) & 0xFF;
    out[16] = hcrc & 0xFF;

    // Payload
    if (!payload.empty()) {
        std::memcpy(out.data() + HEADER_SIZE, payload.data(), payload.size());
    }

    // Frame CRC (2 bytes) - CRC of entire frame except last 2 bytes
    uint16_t fcrc = ControlFrame::calculateCRC(out.data(), total_size - 2);
    out[total_size - 2] = (fcrc >> 8) & 0xFF;
    out[total_size - 1] = fcrc & 0xFF;

    return out;
}

std::optional<DataFrame> DataFrame::deserialize(ByteSpan data) {
    if (data.size() < HEADER_SIZE + CRC_SIZE) {
        return std::nullopt;
    }

    // Check magic
    uint16_t magic = (static_cast<uint16_t>(data[0]) << 8) | data[1];
    if (magic != MAGIC_V2) {
        return std::nullopt;
    }

    // Verify header CRC
    uint16_t received_hcrc = (static_cast<uint16_t>(data[15]) << 8) | data[16];
    uint16_t calculated_hcrc = ControlFrame::calculateCRC(data.data(), 15);
    if (received_hcrc != calculated_hcrc) {
        return std::nullopt;
    }

    // Parse header
    DataFrame f;
    f.type = static_cast<FrameType>(data[2]);
    f.flags = data[3];
    f.seq = (static_cast<uint16_t>(data[4]) << 8) | data[5];
    f.src_hash = (static_cast<uint32_t>(data[6]) << 16) |
                 (static_cast<uint32_t>(data[7]) << 8) |
                 data[8];
    f.dst_hash = (static_cast<uint32_t>(data[9]) << 16) |
                 (static_cast<uint32_t>(data[10]) << 8) |
                 data[11];
    f.total_cw = data[12];
    f.payload_len = (static_cast<uint16_t>(data[13]) << 8) | data[14];

    // Check we have enough data
    size_t expected_size = HEADER_SIZE + f.payload_len + CRC_SIZE;
    if (data.size() < expected_size) {
        return std::nullopt;
    }

    // Verify frame CRC
    uint16_t received_fcrc = (static_cast<uint16_t>(data[expected_size - 2]) << 8) |
                              data[expected_size - 1];
    uint16_t calculated_fcrc = ControlFrame::calculateCRC(data.data(), expected_size - 2);
    if (received_fcrc != calculated_fcrc) {
        return std::nullopt;
    }

    // Extract payload
    if (f.payload_len > 0) {
        f.payload.assign(data.begin() + HEADER_SIZE, data.begin() + HEADER_SIZE + f.payload_len);
    }

    return f;
}

std::string DataFrame::payloadAsText() const {
    return std::string(payload.begin(), payload.end());
}

// ============================================================================
// ConnectFrame implementation (ham-compliant with full callsigns)
// ============================================================================

ConnectFrame ConnectFrame::makeConnect(const std::string& src, const std::string& dst,
                                        uint8_t mode_caps, uint8_t forced_waveform,
                                        uint8_t forced_modulation, uint8_t forced_code_rate,
                                        uint8_t forced_cw_count) {
    ConnectFrame f;
    f.type = FrameType::CONNECT;
    f.flags = Flags::VERSION_V2;
    f.seq = 0;
    f.src_hash = hashCallsign(src);
    f.dst_hash = hashCallsign(dst);

    // Copy callsigns (null-terminated, max 9 chars)
    std::strncpy(f.src_callsign, src.c_str(), MAX_CALLSIGN_LEN - 1);
    f.src_callsign[MAX_CALLSIGN_LEN - 1] = '\0';

    f.mode_capabilities = mode_caps;
    f.negotiated_mode = forced_waveform;        // 0xFF = AUTO, else forced
    f.initial_modulation = forced_modulation;   // 0xFF = AUTO, else forced
    f.initial_code_rate = forced_code_rate;     // 0xFF = AUTO, else forced
    f.measured_snr = 0;                         // Not used in CONNECT
    f.data_frame_cw_count = forced_cw_count;    // 0=AUTO, else 1..8 forced
    f.ladder_rung_id = LadderRungId::UNKNOWN;
    f.phy_mask_v1_capability = ultra::protocol::hasPhyMaskV1Capability(f.mode_capabilities);
    return f;
}

ConnectFrame ConnectFrame::makeConnectAck(const std::string& src, const std::string& dst,
                                           uint8_t neg_mode, Modulation init_mod, CodeRate init_rate,
                                           float snr_db, float fading_index,
                                           uint8_t cw_count,
                                           LadderRungId rung_id) {
    ConnectFrame f;
    f.type = FrameType::CONNECT_ACK;
    f.flags = Flags::VERSION_V2;
    f.seq = 0;
    f.src_hash = hashCallsign(src);
    f.dst_hash = hashCallsign(dst);

    std::strncpy(f.src_callsign, src.c_str(), MAX_CALLSIGN_LEN - 1);
    f.src_callsign[MAX_CALLSIGN_LEN - 1] = '\0';

    // CONNECT_ACK reuses this byte to carry responder fading index.
    f.mode_capabilities = encodeFadingIndex(fading_index);
    f.negotiated_mode = neg_mode;

    // Initial data mode - eliminates separate MODE_CHANGE after connect
    f.initial_modulation = static_cast<uint8_t>(init_mod);
    f.initial_code_rate = static_cast<uint8_t>(init_rate);
    f.measured_snr = encodeSNR(snr_db);
    f.data_frame_cw_count = cw_count;  // Final negotiated CW count (1..8)
    f.ladder_rung_id = rung_id;
    f.phy_mask_v1_capability = false;
    return f;
}

ConnectFrame ConnectFrame::makeConnectNak(const std::string& src, const std::string& dst) {
    ConnectFrame f;
    f.type = FrameType::CONNECT_NAK;
    f.flags = Flags::VERSION_V2;
    f.seq = 0;
    f.src_hash = hashCallsign(src);
    f.dst_hash = hashCallsign(dst);

    std::strncpy(f.src_callsign, src.c_str(), MAX_CALLSIGN_LEN - 1);
    f.src_callsign[MAX_CALLSIGN_LEN - 1] = '\0';

    f.mode_capabilities = 0;
    f.negotiated_mode = 0;
    f.ladder_rung_id = LadderRungId::UNKNOWN;
    f.phy_mask_v1_capability = false;
    return f;
}

ConnectFrame ConnectFrame::makeDisconnect(const std::string& src, const std::string& dst) {
    ConnectFrame f;
    f.type = FrameType::DISCONNECT;
    f.flags = Flags::VERSION_V2;
    f.seq = DISCONNECT_SEQ;  // Unique seq to distinguish from data ACKs
    f.src_hash = hashCallsign(src);
    f.dst_hash = hashCallsign(dst);

    std::strncpy(f.src_callsign, src.c_str(), MAX_CALLSIGN_LEN - 1);
    f.src_callsign[MAX_CALLSIGN_LEN - 1] = '\0';

    f.mode_capabilities = 0;
    f.negotiated_mode = 0;
    f.phy_mask_v1_capability = false;
    return f;
}

ConnectFrame ConnectFrame::makeConnectAckByHash(const std::string& src, uint32_t dst_hash,
                                                 uint8_t neg_mode, Modulation init_mod, CodeRate init_rate,
                                                 float snr_db, float fading_index,
                                                 uint8_t cw_count,
                                                 LadderRungId rung_id) {
    ConnectFrame f;
    f.type = FrameType::CONNECT_ACK;
    f.flags = Flags::VERSION_V2;
    f.seq = 0;
    f.src_hash = hashCallsign(src);
    f.dst_hash = dst_hash & 0xFFFFFF;

    std::strncpy(f.src_callsign, src.c_str(), MAX_CALLSIGN_LEN - 1);
    f.src_callsign[MAX_CALLSIGN_LEN - 1] = '\0';

    f.mode_capabilities = encodeFadingIndex(fading_index);
    f.negotiated_mode = neg_mode;

    // Initial data mode - eliminates separate MODE_CHANGE after connect
    f.initial_modulation = static_cast<uint8_t>(init_mod);
    f.initial_code_rate = static_cast<uint8_t>(init_rate);
    f.measured_snr = encodeSNR(snr_db);
    f.data_frame_cw_count = cw_count;  // Final negotiated CW count (1..8)
    f.ladder_rung_id = rung_id;
    f.phy_mask_v1_capability = false;
    return f;
}

ConnectFrame ConnectFrame::makeConnectNakByHash(const std::string& src, uint32_t dst_hash) {
    ConnectFrame f;
    f.type = FrameType::CONNECT_NAK;
    f.flags = Flags::VERSION_V2;
    f.seq = 0;
    f.src_hash = hashCallsign(src);
    f.dst_hash = dst_hash & 0xFFFFFF;

    std::strncpy(f.src_callsign, src.c_str(), MAX_CALLSIGN_LEN - 1);
    f.src_callsign[MAX_CALLSIGN_LEN - 1] = '\0';

    f.mode_capabilities = 0;
    f.negotiated_mode = 0;
    f.phy_mask_v1_capability = false;
    return f;
}

Bytes ConnectFrame::serialize() const {
    // DATA frame format: header (17B) + payload (16B) + CRC (2B) = 35 bytes, sent as a fixed
    // kConnectFrameCodewords R1/4 frame with frame-level interleaving.
    Bytes out;
    out.reserve(DataFrame::HEADER_SIZE + PAYLOAD_SIZE + DataFrame::CRC_SIZE);

    // Magic (2 bytes)
    out.push_back((MAGIC_V2 >> 8) & 0xFF);
    out.push_back(MAGIC_V2 & 0xFF);

    // Type, flags, seq (4 bytes)
    out.push_back(static_cast<uint8_t>(type));
    out.push_back(flags);
    out.push_back((seq >> 8) & 0xFF);
    out.push_back(seq & 0xFF);

    // Hashes (6 bytes)
    out.push_back((src_hash >> 16) & 0xFF);
    out.push_back((src_hash >> 8) & 0xFF);
    out.push_back(src_hash & 0xFF);
    out.push_back((dst_hash >> 16) & 0xFF);
    out.push_back((dst_hash >> 8) & 0xFF);
    out.push_back(dst_hash & 0xFF);

    // Total codewords (1 byte): the fixed handshake frame's codeword count
    out.push_back(static_cast<uint8_t>(kConnectFrameCodewords));

    // Payload length (2 bytes)
    out.push_back((PAYLOAD_SIZE >> 8) & 0xFF);
    out.push_back(PAYLOAD_SIZE & 0xFF);

    // Header CRC (2 bytes)
    uint16_t hcrc = ControlFrame::calculateCRC(out.data(), out.size());
    out.push_back((hcrc >> 8) & 0xFF);
    out.push_back(hcrc & 0xFF);

    // Payload: src_callsign (10B) + caps (1B) + wfmode (1B) + mod (1B) + rate (1B)
    //          + snr (1B) + cw_count (1B) = 16B. The destination is dst_hash only.
    for (int i = 0; i < MAX_CALLSIGN_LEN; i++) {
        out.push_back(static_cast<uint8_t>(src_callsign[i]));
    }
    out.push_back(mode_capabilities);
    out.push_back(negotiated_mode);
    out.push_back(initial_modulation);
    out.push_back(initial_code_rate);
    out.push_back(measured_snr);
    const uint8_t cw_count_wire = packCWCountAndRung(
        data_frame_cw_count,
        type == FrameType::CONNECT_ACK ? ladder_rung_id : LadderRungId::UNKNOWN,
        type == FrameType::CONNECT_ACK && phy_mask_v1_capability);
    out.push_back(cw_count_wire);

    // Frame CRC (2 bytes)
    uint16_t fcrc = ControlFrame::calculateCRC(out.data(), out.size());
    out.push_back((fcrc >> 8) & 0xFF);
    out.push_back(fcrc & 0xFF);

    return out;
}

std::optional<ConnectFrame> ConnectFrame::deserialize(ByteSpan data) {
    constexpr size_t MIN_SIZE = DataFrame::HEADER_SIZE + PAYLOAD_SIZE + DataFrame::CRC_SIZE;
    if (data.size() < MIN_SIZE) {
        return std::nullopt;
    }

    // Verify magic
    uint16_t magic = (static_cast<uint16_t>(data[0]) << 8) | data[1];
    if (magic != MAGIC_V2) {
        return std::nullopt;
    }

    // Check type is CONNECT, CONNECT_ACK, CONNECT_NAK, or DISCONNECT
    FrameType ftype = static_cast<FrameType>(data[2]);
    if (!isConnectFrame(ftype)) {
        return std::nullopt;
    }

    // Verify header CRC
    uint16_t stored_hcrc = (static_cast<uint16_t>(data[15]) << 8) | data[16];
    uint16_t calc_hcrc = ControlFrame::calculateCRC(data.data(), 15);
    if (stored_hcrc != calc_hcrc) {
        return std::nullopt;
    }

    // Verify frame CRC
    size_t fcrc_offset = DataFrame::HEADER_SIZE + PAYLOAD_SIZE;
    uint16_t stored_fcrc = (static_cast<uint16_t>(data[fcrc_offset]) << 8) | data[fcrc_offset + 1];
    uint16_t calc_fcrc = ControlFrame::calculateCRC(data.data(), fcrc_offset);
    if (stored_fcrc != calc_fcrc) {
        return std::nullopt;
    }

    ConnectFrame f;
    f.type = ftype;
    f.flags = data[3];
    f.seq = (static_cast<uint16_t>(data[4]) << 8) | data[5];
    f.src_hash = (static_cast<uint32_t>(data[6]) << 16) |
                 (static_cast<uint32_t>(data[7]) << 8) |
                 data[8];
    f.dst_hash = (static_cast<uint32_t>(data[9]) << 16) |
                 (static_cast<uint32_t>(data[10]) << 8) |
                 data[11];

    // Parse payload (starts at offset 17)
    size_t payload_offset = DataFrame::HEADER_SIZE;
    for (int i = 0; i < MAX_CALLSIGN_LEN; i++) {
        f.src_callsign[i] = static_cast<char>(data[payload_offset + i]);
    }
    f.src_callsign[MAX_CALLSIGN_LEN - 1] = '\0';  // Ensure null-terminated

    size_t field_offset = payload_offset + MAX_CALLSIGN_LEN;
    f.mode_capabilities = data[field_offset];
    f.negotiated_mode = data[field_offset + 1];
    f.initial_modulation = data[field_offset + 2];
    f.initial_code_rate = data[field_offset + 3];
    f.measured_snr = data[field_offset + 4];
    const uint8_t cw_count_wire = data[field_offset + 5];
    if (f.type == FrameType::CONNECT_ACK) {
        f.phy_mask_v1_capability = (cw_count_wire & ModeCapabilities::PHY_MASK_V1) != 0;
        f.data_frame_cw_count = unpackCWCount(cw_count_wire);
        f.ladder_rung_id = unpackLadderRungId(cw_count_wire);
    } else {
        f.data_frame_cw_count = unpackCWCount(cw_count_wire);
        f.ladder_rung_id = LadderRungId::UNKNOWN;
        f.phy_mask_v1_capability = ultra::protocol::hasPhyMaskV1Capability(f.mode_capabilities);
    }

    return f;
}

std::string ConnectFrame::getSrcCallsign() const {
    return std::string(src_callsign);
}


bool hasPhyMaskV1Capability(const ConnectFrame& frame) {
    if (frame.type == FrameType::CONNECT_ACK) {
        return frame.phy_mask_v1_capability;
    }
    return ultra::protocol::hasPhyMaskV1Capability(frame.mode_capabilities);
}

void setPhyMaskV1Capability(ConnectFrame& frame) {
    if (frame.type == FrameType::CONNECT_ACK) {
        frame.phy_mask_v1_capability = true;
        return;
    }
    frame.mode_capabilities = ultra::protocol::setPhyMaskV1Capability(frame.mode_capabilities);
    frame.phy_mask_v1_capability = true;
}

// ============================================================================
// NackPayload implementation
// ============================================================================

void NackPayload::encode(uint8_t* out) const {
    // Frame sequence (2 bytes)
    out[0] = (frame_seq >> 8) & 0xFF;
    out[1] = frame_seq & 0xFF;

    // Codeword bitmap (4 bytes)
    out[2] = (cw_bitmap >> 24) & 0xFF;
    out[3] = (cw_bitmap >> 16) & 0xFF;
    out[4] = (cw_bitmap >> 8) & 0xFF;
    out[5] = cw_bitmap & 0xFF;
}

NackPayload NackPayload::decode(const uint8_t* in) {
    NackPayload np;
    np.frame_seq = (static_cast<uint16_t>(in[0]) << 8) | in[1];
    np.cw_bitmap = (static_cast<uint32_t>(in[2]) << 24) |
                   (static_cast<uint32_t>(in[3]) << 16) |
                   (static_cast<uint32_t>(in[4]) << 8) |
                   in[5];
    return np;
}

int NackPayload::countFailed() const {
    int count = 0;
    uint32_t b = cw_bitmap;
    while (b) {
        count += b & 1;
        b >>= 1;
    }
    return count;
}

// ============================================================================
// DataRepairFrame implementation
// ============================================================================

namespace {

uint8_t countBits16(uint16_t value) {
    uint8_t count = 0;
    while (value != 0) {
        count += static_cast<uint8_t>(value & 1u);
        value >>= 1;
    }
    return count;
}

} // namespace

DataRepairFrame DataRepairFrame::make(const std::string& src, const std::string& dst,
                                      uint16_t target_seq, uint8_t original_total_cw,
                                      uint32_t repair_bitmap, CodeRate rate,
                                      const std::vector<Bytes>& repair_codewords) {
    DataRepairFrame frame;
    frame.flags = Flags::VERSION_V2;
    frame.target_seq = target_seq;
    frame.src_hash = hashCallsign(src);
    frame.dst_hash = hashCallsign(dst);
    frame.original_total_cw = original_total_cw;
    frame.repair_bitmap = static_cast<uint16_t>(repair_bitmap & 0xFFFFu);
    frame.repair_count = static_cast<uint8_t>(repair_codewords.size());
    frame.rate = rate;
    frame.repair_codewords = repair_codewords;
    return frame;
}

bool DataRepairFrame::valid() const {
    if (original_total_cw == 0 || original_total_cw > MAX_REPAIR_CW) {
        return false;
    }
    const uint16_t expected_mask = original_total_cw >= 16
        ? 0xFFFFu
        : static_cast<uint16_t>((1u << original_total_cw) - 1u);
    if (repair_bitmap == 0 || (repair_bitmap & ~expected_mask) != 0) {
        return false;
    }
    if (repair_count == 0 || repair_count != countBits16(repair_bitmap)) {
        return false;
    }
    if (repair_codewords.size() != repair_count) {
        return false;
    }
    const size_t bytes_per_cw = getBytesPerCodeword(rate);
    if (bytes_per_cw < HEADER_BYTES) {
        return false;
    }
    for (const auto& cw : repair_codewords) {
        if (cw.empty() || cw.size() > bytes_per_cw) {
            return false;
        }
    }
    return true;
}

std::vector<uint8_t> DataRepairFrame::repairIndices() const {
    std::vector<uint8_t> indices;
    for (uint8_t i = 0; i < original_total_cw && i < MAX_REPAIR_CW; ++i) {
        if ((repair_bitmap & (1u << i)) != 0) {
            indices.push_back(i);
        }
    }
    return indices;
}

Bytes DataRepairFrame::headerCodeword() const {
    const size_t bytes_per_cw = std::max(getBytesPerCodeword(rate), HEADER_BYTES);
    Bytes out(bytes_per_cw, 0);

    out[0] = (MAGIC_V2 >> 8) & 0xFF;
    out[1] = MAGIC_V2 & 0xFF;
    out[2] = static_cast<uint8_t>(FrameType::DATA_REPAIR);
    out[3] = flags;
    out[4] = (target_seq >> 8) & 0xFF;
    out[5] = target_seq & 0xFF;
    out[6] = (src_hash >> 16) & 0xFF;
    out[7] = (src_hash >> 8) & 0xFF;
    out[8] = src_hash & 0xFF;
    out[9] = (dst_hash >> 16) & 0xFF;
    out[10] = (dst_hash >> 8) & 0xFF;
    out[11] = dst_hash & 0xFF;
    out[12] = original_total_cw;
    out[13] = (repair_bitmap >> 8) & 0xFF;
    out[14] = repair_bitmap & 0xFF;
    out[15] = repair_count;
    out[16] = static_cast<uint8_t>(rate);
    out[17] = 0;  // reserved

    const uint16_t crc = ControlFrame::calculateCRC(out.data(), 18);
    out[18] = (crc >> 8) & 0xFF;
    out[19] = crc & 0xFF;
    return out;
}

std::vector<Bytes> DataRepairFrame::infoCodewords() const {
    std::vector<Bytes> codewords;
    if (!valid()) {
        return codewords;
    }
    const size_t bytes_per_cw = getBytesPerCodeword(rate);
    codewords.push_back(headerCodeword());
    for (auto cw : repair_codewords) {
        cw.resize(bytes_per_cw, 0);
        codewords.push_back(std::move(cw));
    }
    return codewords;
}

Bytes DataRepairFrame::serialize() const {
    Bytes out;
    auto codewords = infoCodewords();
    const size_t bytes_per_cw = getBytesPerCodeword(rate);
    out.reserve(codewords.size() * bytes_per_cw);
    for (const auto& cw : codewords) {
        out.insert(out.end(), cw.begin(), cw.end());
    }
    return out;
}

std::optional<DataRepairFrame> DataRepairFrame::parseHeader(ByteSpan first_codeword) {
    if (first_codeword.size() < HEADER_BYTES) {
        return std::nullopt;
    }
    const uint16_t magic = (static_cast<uint16_t>(first_codeword[0]) << 8) | first_codeword[1];
    if (magic != MAGIC_V2 || first_codeword[2] != static_cast<uint8_t>(FrameType::DATA_REPAIR)) {
        return std::nullopt;
    }
    const uint16_t received_crc =
        (static_cast<uint16_t>(first_codeword[18]) << 8) | first_codeword[19];
    const uint16_t calculated_crc = ControlFrame::calculateCRC(first_codeword.data(), 18);
    if (received_crc != calculated_crc) {
        return std::nullopt;
    }

    DataRepairFrame frame;
    frame.flags = first_codeword[3];
    frame.target_seq = (static_cast<uint16_t>(first_codeword[4]) << 8) | first_codeword[5];
    frame.src_hash = (static_cast<uint32_t>(first_codeword[6]) << 16) |
                     (static_cast<uint32_t>(first_codeword[7]) << 8) |
                     first_codeword[8];
    frame.dst_hash = (static_cast<uint32_t>(first_codeword[9]) << 16) |
                     (static_cast<uint32_t>(first_codeword[10]) << 8) |
                     first_codeword[11];
    frame.original_total_cw = first_codeword[12];
    frame.repair_bitmap = (static_cast<uint16_t>(first_codeword[13]) << 8) |
                          first_codeword[14];
    frame.repair_count = first_codeword[15];
    frame.rate = static_cast<CodeRate>(first_codeword[16]);
    return frame;
}

std::optional<DataRepairFrame> DataRepairFrame::deserialize(ByteSpan data) {
    auto frame = parseHeader(data);
    if (!frame) {
        return std::nullopt;
    }
    const size_t bytes_per_cw = getBytesPerCodeword(frame->rate);
    if (bytes_per_cw < HEADER_BYTES) {
        return std::nullopt;
    }
    const size_t expected_size =
        static_cast<size_t>(frame->repair_count + 1) * bytes_per_cw;
    if (data.size() < expected_size) {
        return std::nullopt;
    }

    frame->repair_codewords.clear();
    frame->repair_codewords.reserve(frame->repair_count);
    for (uint8_t i = 0; i < frame->repair_count; ++i) {
        const size_t offset = static_cast<size_t>(i + 1) * bytes_per_cw;
        frame->repair_codewords.emplace_back(data.begin() + offset,
                                             data.begin() + offset + bytes_per_cw);
    }
    if (!frame->valid()) {
        return std::nullopt;
    }
    return frame;
}

// ============================================================================
// Codeword helpers
// ============================================================================

std::vector<Bytes> splitIntoCodewords(const Bytes& frame_data) {
    return splitIntoCodewords(frame_data, CodeRate::R1_4);
}

std::vector<Bytes> splitIntoCodewords(const Bytes& frame_data, CodeRate rate) {
    std::vector<Bytes> codewords;
    const size_t bytes_per_cw = getBytesPerCodeword(rate);
    const size_t data_payload_size = bytes_per_cw - DATA_CW_HEADER_SIZE;

    // CW0: First 20 bytes of frame data (contains header with 0x554C magic)
    // No modification needed - the magic already identifies it
    {
        Bytes cw0(bytes_per_cw, 0);  // Zero-pad if needed
        size_t cw0_data = std::min(bytes_per_cw, frame_data.size());
        std::memcpy(cw0.data(), frame_data.data(), cw0_data);
        codewords.push_back(std::move(cw0));
    }

    // CW1+: Add marker + index header before payload data
    size_t offset = bytes_per_cw;  // Start after CW0's data
    uint8_t cw_index = 1;

    while (offset < frame_data.size()) {
        Bytes cw(bytes_per_cw, 0);  // Zero-pad if needed

        // Add marker and index
        cw[0] = DATA_CW_MARKER;
        cw[1] = cw_index;

        // Copy payload data (up to 18 bytes)
        size_t remaining = frame_data.size() - offset;
        size_t chunk_size = std::min(data_payload_size, remaining);
        std::memcpy(cw.data() + DATA_CW_HEADER_SIZE, frame_data.data() + offset, chunk_size);

        codewords.push_back(std::move(cw));
        offset += data_payload_size;  // Each CW1+ consumes payload bytes after marker
        cw_index++;
    }

    return codewords;
}

Bytes reassembleCodewords(const std::vector<Bytes>& codewords, size_t expected_size) {
    Bytes result;
    result.reserve(expected_size);

    for (size_t i = 0; i < codewords.size(); i++) {
        size_t remaining = expected_size - result.size();
        if (remaining == 0) break;

        if (i == 0) {
            // CW0: All 20 bytes are frame data (header + payload start)
            size_t to_copy = std::min(remaining, codewords[i].size());
            result.insert(result.end(), codewords[i].begin(), codewords[i].begin() + to_copy);
        } else {
            // CW1+: Skip marker (0xD5) and index, copy payload portion
            // Verify marker byte (optional - for robustness)
            if (codewords[i].size() >= DATA_CW_HEADER_SIZE && codewords[i][0] == DATA_CW_MARKER) {
                size_t payload_size = codewords[i].size() - DATA_CW_HEADER_SIZE;
                size_t to_copy = std::min(remaining, payload_size);
                result.insert(result.end(),
                              codewords[i].begin() + DATA_CW_HEADER_SIZE,
                              codewords[i].begin() + DATA_CW_HEADER_SIZE + to_copy);
            } else {
                // Fallback: old format without marker (backward compatibility during transition)
                size_t to_copy = std::min(remaining, codewords[i].size());
                result.insert(result.end(), codewords[i].begin(), codewords[i].begin() + to_copy);
            }
        }
    }

    return result;
}

Bytes reassembleFixedCodewords(const std::vector<Bytes>& codewords, size_t expected_size) {
    Bytes result;
    result.reserve(expected_size);

    for (const auto& cw : codewords) {
        size_t remaining = expected_size - result.size();
        if (remaining == 0) break;

        size_t to_copy = std::min(remaining, cw.size());
        result.insert(result.end(), cw.begin(), cw.begin() + to_copy);
    }

    return result;
}

uint32_t CodewordStatus::getNackBitmap() const {
    uint32_t bitmap = 0;
    for (size_t i = 0; i < decoded.size() && i < 32; i++) {
        if (!decoded[i]) {
            bitmap |= (1u << i);
        }
    }
    return bitmap;
}

bool CodewordStatus::allSuccess() const {
    for (bool d : decoded) {
        if (!d) return false;
    }
    return true;
}

int CodewordStatus::countFailures() const {
    int count = 0;
    for (bool d : decoded) {
        if (!d) count++;
    }
    return count;
}

uint8_t CodewordStatus::getExpectedCodewords() const {
    if (decoded.empty() || !decoded[0] || data.empty() || data[0].size() < 20) {
        return 0;
    }

    // Parse header from first codeword
    auto info = parseHeader(data[0]);
    if (!info.valid) {
        return 0;
    }

    return info.total_cw;
}

Bytes CodewordStatus::reassemble() const {
    if (decoded.empty() || !decoded[0] || data.empty()) {
        LOG_MODEM(WARN, "reassemble: early return (decoded=%zu, data=%zu)",
                  decoded.size(), data.size());
        return {};
    }

    // Parse header to know total size
    auto info = parseHeader(data[0]);
    if (!info.valid) {
        // Log all 20 bytes to diagnose LDPC false positives vs bit errors
        if (data[0].size() >= 17) {
            LOG_MODEM(WARN, "reassemble: header invalid, CW0[0..16]: "
                      "%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X (size=%zu)",
                      data[0][0], data[0][1], data[0][2], data[0][3],
                      data[0][4], data[0][5], data[0][6], data[0][7],
                      data[0][8], data[0][9], data[0][10], data[0][11],
                      data[0][12], data[0][13], data[0][14], data[0][15],
                      data[0][16], data[0].size());
        }
        return {};
    }

    // Calculate expected frame size
    size_t expected_size;
    if (info.is_control) {
        expected_size = ControlFrame::SIZE;  // 20 bytes
    } else {
        expected_size = DataFrame::HEADER_SIZE + info.payload_len + DataFrame::CRC_SIZE;
    }

    // Fixed OFDM frames pack raw info bytes contiguously across all fixed CWs.
    // Do not run them through the marker-aware variable-CW reassembler: a real
    // payload byte of 0xD5 at a CW boundary would be mistaken for a marker.
    if (!info.is_control && fixed_frame &&
        info.total_cw >= kMinFixedFrameCodewords &&
        info.total_cw <= kMaxFixedFrameCodewords) {
        return reassembleFixedCodewords(data, expected_size);
    }

    return reassembleCodewords(data, expected_size);
}

bool CodewordStatus::mergeCodeword(size_t index, const Bytes& cw_data) {
    if (index >= decoded.size()) {
        return false;  // Index out of range
    }

    if (decoded[index]) {
        return false;  // Already decoded, no need to merge
    }

    // Merge the retransmitted codeword
    decoded[index] = true;
    data[index] = cw_data;
    return true;
}

void CodewordStatus::initForFrame(uint8_t total_cw) {
    // Must clear first! resize() preserves existing elements when shrinking,
    // which would leave stale 'true' values from previous frame
    fixed_frame = false;
    decoded.clear();
    decoded.resize(total_cw, false);
    data.clear();
    data.resize(total_cw);
    iterations.assign(total_cw, 0);
    unsatisfied_checks.assign(total_cw, -1);
    llr_abs_mean.assign(total_cw, 0.0f);
    llr_abs_min.assign(total_cw, 0.0f);
    llr_abs_p10.assign(total_cw, 0.0f);
    llr_abs_p50.assign(total_cw, 0.0f);
    llr_abs_p90.assign(total_cw, 0.0f);
    used_perturbation.assign(total_cw, 0);
    harq_attempts.assign(total_cw, 0);
}

// ============================================================================
// LDPC Integration Implementation
// ============================================================================

std::vector<Bytes> encodeFrameWithLDPC(const Bytes& frame_data) {
    // Default to R1/4 for control frames
    return encodeFrameWithLDPC(frame_data, CodeRate::R1_4);
}

std::vector<Bytes> encodeFrameWithLDPC(const Bytes& frame_data, CodeRate rate) {
    std::vector<Bytes> chunks = splitIntoCodewords(frame_data, rate);

    // Create LDPC encoder with specified rate
    LDPCEncoder encoder(rate);

    std::vector<Bytes> encoded_codewords;
    encoded_codewords.reserve(chunks.size());

    for (const auto& chunk : chunks) {
        auto encoded = encoder.encode(chunk);
        encoded_codewords.push_back(std::move(encoded));
    }

    return encoded_codewords;
}

std::vector<Bytes> encodeInfoCodewordsWithLDPC(const std::vector<Bytes>& info_codewords,
                                               CodeRate rate) {
    const size_t bytes_per_cw = getBytesPerCodeword(rate);
    LDPCEncoder encoder(rate);

    std::vector<Bytes> encoded_codewords;
    encoded_codewords.reserve(info_codewords.size());
    for (auto cw : info_codewords) {
        cw.resize(bytes_per_cw, 0);
        auto encoded = encoder.encode(cw);
        encoded_codewords.push_back(std::move(encoded));
    }
    return encoded_codewords;
}

std::pair<bool, Bytes> decodeSingleCodeword(const std::vector<float>& soft_bits) {
    // Default to R1/4 for control frames
    return decodeSingleCodeword(soft_bits, CodeRate::R1_4);
}

std::pair<bool, Bytes> decodeSingleCodeword(const std::vector<float>& soft_bits, CodeRate rate) {
    if (soft_bits.size() < LDPC_CODEWORD_BITS) {
        return {false, {}};
    }

    // Get bytes per codeword for this rate
    size_t bytes_per_cw = getBytesPerCodeword(rate);

    // Create LDPC decoder with specified rate and recommended iterations
    LDPCDecoder decoder(rate);
    decoder.setMaxIterations(fec::LDPCCodec::getRecommendedIterations(rate));

    // Decode - returns k/8 bytes
    auto decoded = decoder.decodeSoft(soft_bits);
    bool success = decoder.lastDecodeSuccess();

    if (!success || decoded.size() < bytes_per_cw) {
        return {false, {}};
    }

    // Return exactly bytes_per_cw bytes (the info portion)
    Bytes result(decoded.begin(), decoded.begin() + bytes_per_cw);
    unwhitenControlCodeword(result);  // INV-FRAME-WHITEN-001: no-op unless whitened
    return {true, result};
}

bool controlFrameWhiteningEnabled() {
    static const bool on = [] {
        const char* e = std::getenv("ULTRA_CONTROL_WHITEN");
        return !(e != nullptr && e[0] == '0' && e[1] == '\0');
    }();
    return on;
}

CodewordStatus decodeCodewordsWithLDPC(const std::vector<std::vector<float>>& soft_bits) {
    CodewordStatus status;
    status.decoded.resize(soft_bits.size(), false);
    status.data.resize(soft_bits.size());
    status.iterations.assign(soft_bits.size(), 0);
    status.unsatisfied_checks.assign(soft_bits.size(), -1);
    status.llr_abs_mean.assign(soft_bits.size(), 0.0f);
    status.llr_abs_min.assign(soft_bits.size(), 0.0f);
    status.llr_abs_p10.assign(soft_bits.size(), 0.0f);
    status.llr_abs_p50.assign(soft_bits.size(), 0.0f);
    status.llr_abs_p90.assign(soft_bits.size(), 0.0f);
    status.used_perturbation.assign(soft_bits.size(), 0);
    status.harq_attempts.assign(soft_bits.size(), 1);

    for (size_t i = 0; i < soft_bits.size(); i++) {
        const auto llr_summary = summarizeAbsLlrs(soft_bits[i]);
        auto [success, data] = decodeSingleCodeword(soft_bits[i]);
        status.decoded[i] = success;
        status.llr_abs_mean[i] = llr_summary.mean;
        status.llr_abs_min[i] = llr_summary.min;
        status.llr_abs_p10[i] = llr_summary.p10;
        status.llr_abs_p50[i] = llr_summary.p50;
        status.llr_abs_p90[i] = llr_summary.p90;
        if (success) {
            status.data[i] = std::move(data);
        }
    }

    return status;
}

HeaderInfo parseHeader(const Bytes& first_codeword_data) {
    HeaderInfo info;

    if (first_codeword_data.size() < BYTES_PER_CODEWORD) {
        return info;  // invalid
    }

    // Check magic
    uint16_t magic = (static_cast<uint16_t>(first_codeword_data[0]) << 8) |
                      first_codeword_data[1];
    if (magic != MAGIC_V2) {
        return info;  // invalid magic
    }

    // Parse type
    info.type = static_cast<FrameType>(first_codeword_data[2]);
    info.is_control = isControlFrame(info.type);

    // Parse sequence
    info.seq = (static_cast<uint16_t>(first_codeword_data[4]) << 8) |
                first_codeword_data[5];

    // Parse hashes
    info.src_hash = (static_cast<uint32_t>(first_codeword_data[6]) << 16) |
                    (static_cast<uint32_t>(first_codeword_data[7]) << 8) |
                    first_codeword_data[8];
    info.dst_hash = (static_cast<uint32_t>(first_codeword_data[9]) << 16) |
                    (static_cast<uint32_t>(first_codeword_data[10]) << 8) |
                    first_codeword_data[11];

    if (info.is_control) {
        // Control frame: always 1 codeword, verify CRC
        uint16_t received_crc = (static_cast<uint16_t>(first_codeword_data[18]) << 8) |
                                 first_codeword_data[19];
        uint16_t calculated_crc = ControlFrame::calculateCRC(first_codeword_data.data(), 18);
        if (received_crc != calculated_crc) {
            return info;  // CRC failed
        }
        info.total_cw = 1;
        info.payload_len = 0;
    } else if (info.type == FrameType::DATA_REPAIR) {
        auto repair = DataRepairFrame::parseHeader(first_codeword_data);
        if (!repair) {
            return info;
        }
        info.total_cw = static_cast<uint8_t>(repair->repair_count + 1);
        info.payload_len = 0;
    } else {
        // Data frame: read TOTAL_CW and LEN, verify header CRC
        info.total_cw = first_codeword_data[12];
        info.payload_len = (static_cast<uint16_t>(first_codeword_data[13]) << 8) |
                            first_codeword_data[14];

        uint16_t received_hcrc = (static_cast<uint16_t>(first_codeword_data[15]) << 8) |
                                  first_codeword_data[16];
        uint16_t calculated_hcrc = ControlFrame::calculateCRC(first_codeword_data.data(), 15);
        if (received_hcrc != calculated_hcrc) {
            LOG_MODEM(WARN, "parseHeader: data frame header CRC mismatch: rx=%04X calc=%04X (total_cw=%d, payload_len=%d)",
                      received_hcrc, calculated_hcrc, info.total_cw, info.payload_len);
            return info;  // Header CRC failed
        }
    }

    info.valid = true;
    return info;
}

CodewordInfo identifyCodeword(const Bytes& cw_data) {
    CodewordInfo info;

    if (cw_data.size() < 2) {
        return info;  // Too short to identify
    }

    // Check for header magic (0x554C = "UL")
    uint16_t first_two = (static_cast<uint16_t>(cw_data[0]) << 8) | cw_data[1];
    if (first_two == MAGIC_V2) {
        info.type = CodewordType::HEADER;
        info.index = 0;
        return info;
    }

    // Check for data codeword marker (0xD5)
    if (cw_data[0] == DATA_CW_MARKER) {
        info.type = CodewordType::DATA;
        info.index = cw_data[1];
        return info;
    }

    // Unknown codeword type
    return info;
}

// ============================================================================
// Fixed-Codeword Frame Implementation
// ============================================================================

namespace {
// Info-block counts of the 24-column 802.11n base matrix (k = info_blocks * Z).
// Matches LDPCEncoder/Decoder getCodeParams. Used to size the file-class long
// codeword (Z=81) without touching the rate-only getBytesPerCodeword() used by
// the default (Z=27) path and its many capacity-math callers.
inline int infoBlocksForRate(CodeRate r) {
    switch (r) {
        case CodeRate::R1_4: return 6;
        case CodeRate::R1_2: return 12;
        case CodeRate::R2_3: return 16;
        case CodeRate::R3_4: return 18;
        case CodeRate::R5_6: return 20;
        default:             return 12;
    }
}
// Info bytes per codeword at lifting size Z. Z=27 must equal getBytesPerCodeword().
inline size_t infoBytesPerCodewordZ(CodeRate r, int lifting_z) {
    return static_cast<size_t>(infoBlocksForRate(r)) * static_cast<size_t>(lifting_z) / 8;
}
constexpr int kLdpcBlockCols = 24;  // n = kLdpcBlockCols * Z (648 @ Z27, 1944 @ Z81)
}  // namespace

Bytes padFixedFrameInfo(const Bytes& frame_data, size_t total_info_bytes) {
    Bytes padded = frame_data;
    if (padded.size() < total_info_bytes) {
        // WHITEN THE PAD with a deterministic PRBS, NOT zeros. A zero pad makes a mostly-
        // empty frame (e.g. the ~90%-padding FILE_START) LDPC-encode to long runs of the
        // SAME 16QAM constellation point; a systematic channel-estimate error then hits all
        // of them the same way -> correlated symbol errors that LDPC (built for RANDOM
        // errors) cannot fix. Measured 2026-05-30: FILE_START failed with biased LLRs
        // (llr_avg≈4-6, high unsat) while random-data frames in the SAME group decoded clean
        // -- purely a payload-entropy effect. Whitening the pad gives every frame diverse
        // symbols. RX-transparent: the receiver reads payload_len bytes and ignores the pad,
        // so no de-whitening and no compat bit are needed. (The turbo data frame whitens
        // the WHOLE info block on top of this; see fec/turbo_codec.hpp.)
        const size_t pad_start = padded.size();
        padded.resize(total_info_bytes);
        uint32_t r = 0x9E3779B9u;  // fixed nonzero seed -> deterministic xorshift32 PRBS
        for (size_t i = pad_start; i < total_info_bytes; ++i) {
            r ^= r << 13; r ^= r >> 17; r ^= r << 5;
            padded[i] = static_cast<uint8_t>(r);
        }
    } else if (padded.size() > total_info_bytes) {
        padded.resize(total_info_bytes);  // Truncate (caller should have chunked)
    }
    return padded;
}

DataFrameFec ofdmDataFrameFec() {
    static const DataFrameFec fec = [] {
        if (const char* e = std::getenv("ULTRA_DATA_FEC")) {
            const std::string s(e);
            if (s == "turbo" || s == "TURBO") return DataFrameFec::TURBO;
            if (s == "ldpc" || s == "LDPC") return DataFrameFec::LDPC;
            LOG_MODEM(WARN, "ULTRA_DATA_FEC='%s' not recognised (ldpc|turbo); using default", e);
        }
        // Unset (or unrecognised): the ULTRA_FAST profile selects turbo.
        return fastProfileEnabled() ? DataFrameFec::TURBO : DataFrameFec::LDPC;
    }();
    return fec;
}

namespace {
size_t fixedFrameInfoBytesPerCodeword(CodeRate rate, int lifting_z) {
    return (lifting_z == 27) ? getBytesPerCodeword(rate) : infoBytesPerCodewordZ(rate, lifting_z);
}
}  // namespace

Bytes encodeTurboFixedFrame(const Bytes& frame_data, CodeRate rate, int cw_count, int lifting_z) {
    cw_count = sanitizeFixedFrameCodewords(cw_count);
    lifting_z = (lifting_z == 81) ? 81 : 27;
    const size_t total_info_bytes =
        static_cast<size_t>(cw_count) * fixedFrameInfoBytesPerCodeword(rate, lifting_z);
    const int air_bits = cw_count * kLdpcBlockCols * lifting_z;
    return fec::turbo::encode(padFixedFrameInfo(frame_data, total_info_bytes), air_bits);
}

CodewordStatus decodeTurboFixedFrame(const std::vector<float>& air_soft, CodeRate rate,
                                     int cw_count, int lifting_z, Bytes* hard_info_bytes) {
    cw_count = sanitizeFixedFrameCodewords(cw_count);
    lifting_z = (lifting_z == 81) ? 81 : 27;
    const size_t block = static_cast<size_t>(kLdpcBlockCols * lifting_z);
    const size_t air_bits = static_cast<size_t>(cw_count) * block;
    const size_t n = static_cast<size_t>(cw_count);

    CodewordStatus status;
    status.fixed_frame = true;
    status.decoded.assign(n, false);
    status.data.assign(n, Bytes{});
    status.iterations.assign(n, 0);
    status.unsatisfied_checks.assign(n, 1);
    status.llr_abs_mean.assign(n, 0.0f);
    status.llr_abs_min.assign(n, 0.0f);
    status.llr_abs_p10.assign(n, 0.0f);
    status.llr_abs_p50.assign(n, 0.0f);
    status.llr_abs_p90.assign(n, 0.0f);
    status.used_perturbation.assign(n, 0);
    status.harq_attempts.assign(n, 1);
    if (air_soft.size() < air_bits) {
        return status;
    }
    // Per-air-block pre-FEC |LLR| summaries keep the LDPC-era diagnostics meaningful.
    for (size_t i = 0; i < n; ++i) {
        const std::vector<float> blk(air_soft.begin() + static_cast<std::ptrdiff_t>(i * block),
                                     air_soft.begin() + static_cast<std::ptrdiff_t>((i + 1) * block));
        const auto s = summarizeAbsLlrs(blk);
        status.llr_abs_mean[i] = s.mean;
        status.llr_abs_min[i] = s.min;
        status.llr_abs_p10[i] = s.p10;
        status.llr_abs_p50[i] = s.p50;
        status.llr_abs_p90[i] = s.p90;
    }

    const size_t total_info_bytes = n * fixedFrameInfoBytesPerCodeword(rate, lifting_z);
    const auto r = fec::turbo::decode(std::span<const float>(air_soft.data(), air_bits),
                                      total_info_bytes);
    std::fill(status.iterations.begin(), status.iterations.end(), r.iterations);
    if (hard_info_bytes) {
        *hard_info_bytes = r.info_bytes;  // de-whitened hard decisions (valid or not)
    }
    if (r.crc_ok) {
        status.data[0] = r.info_bytes;
        std::fill(status.decoded.begin(), status.decoded.end(), true);
        // Defense in depth: CRC-32C passed, the frame's own header/frame CRCs must too.
        if (!fixedFramePayloadIsValid(status.reassemble())) {
            LOG_MODEM(WARN, "turbo: CRC-32C ok but frame header/CRC invalid (cw=%d) — rejected", cw_count);
            std::fill(status.decoded.begin(), status.decoded.end(), false);
            status.data[0].clear();
        } else {
            std::fill(status.unsatisfied_checks.begin(), status.unsatisfied_checks.end(), 0);
        }
    }
    LOG_MODEM(DEBUG, "turbo: cw=%d K=%d N=%zu iters=%d crc=%d stop=%d |llr|=%.2f clip=%.3f",
              cw_count, fec::turbo::infoBitsForBytes(total_info_bytes), air_bits, r.iterations,
              r.crc_ok ? 1 : 0, static_cast<int>(r.stop), r.mean_abs_llr, r.clip_fraction);
    return status;
}

Bytes encodeFixedFrame(const Bytes& frame_data, CodeRate rate, int cw_count,
                       bool use_channel_interleave, size_t bits_per_symbol,
                       int lifting_z) {
    using namespace fec;

    cw_count = sanitizeFixedFrameCodewords(cw_count);
    lifting_z = (lifting_z == 81) ? 81 : 27;
    const int codeword_bits = kLdpcBlockCols * lifting_z;  // 648 or 1944
    size_t bytes_per_cw = (lifting_z == 27) ? getBytesPerCodeword(rate)
                                            : infoBytesPerCodewordZ(rate, lifting_z);
    size_t total_info_bytes = static_cast<size_t>(cw_count) * bytes_per_cw;

    // Pad frame data to exactly N CWs worth of info bytes.
    Bytes padded = padFixedFrameInfo(frame_data, total_info_bytes);

    // Split into fixed-size info chunks and LDPC encode each
    LDPCEncoder& encoder = fixedFrameEncoderForRate(rate, lifting_z);
    std::vector<std::vector<uint8_t>> coded_codewords;
    coded_codewords.reserve(static_cast<size_t>(cw_count));

    // Create channel interleaver if enabled
    std::unique_ptr<ChannelInterleaver> interleaver;
    if (use_channel_interleave) {
        interleaver = std::make_unique<ChannelInterleaver>(bits_per_symbol, codeword_bits);
    }

    for (int cw = 0; cw < cw_count; ++cw) {
        // Extract info bytes for this CW
        Bytes info_chunk(padded.begin() + cw * bytes_per_cw,
                         padded.begin() + (cw + 1) * bytes_per_cw);

        // LDPC encode → 81 bytes (648 bits)
        auto coded = encoder.encode(info_chunk);

        // Apply channel interleaving if enabled
        if (use_channel_interleave && interleaver) {
            coded = interleaver->interleave(coded);
        }

        coded_codewords.push_back(std::move(coded));
    }

    // Apply frame-level interleaving — spreads bits ACROSS the frame's codewords.
    // 2026-05-28: when cw_count==1 (long-LDPC z=81 path), there is no other
    // codeword to spread bits across, so the frame-level interleaver step is
    // omitted entirely (the result is just the lone codeword's coded bytes).
    // BurstInterleaver (cross-frame) and ChannelInterleaver (within-CW across
    // OFDM symbols) still apply — they protect different impairments.
    if (cw_count == 1) {
        return coded_codewords.empty() ? Bytes{} : coded_codewords[0];
    }
    return FrameInterleaver::interleave(coded_codewords, cw_count, codeword_bits);
}

Bytes encodeFixedFrame(const Bytes& frame_data, CodeRate rate, bool use_channel_interleave, size_t bits_per_symbol) {
    return encodeFixedFrame(frame_data, rate, kDefaultFixedFrameCodewords,
                            use_channel_interleave, bits_per_symbol);
}

Bytes encodeFixedFrame(const Bytes& frame_data, CodeRate rate, int cw_count) {
    return encodeFixedFrame(frame_data, rate, cw_count, false);
}

// Default: no channel interleaving (backward compatible)
Bytes encodeFixedFrame(const Bytes& frame_data, CodeRate rate) {
    return encodeFixedFrame(frame_data, rate, false);
}

CodewordStatus decodeFixedFrame(const std::vector<float>& interleaved_soft, CodeRate rate, int cw_count,
                                bool use_channel_deinterleave, size_t bits_per_symbol,
                                fec::SoftCombineBuffer* harq_buffer,
                                const fec::SoftCombineBuffer::Key* harq_key,
                                int lifting_z,
                                bool harq_key_provisional) {
    using namespace fec;
    ultra::timing::ScopedTimer _profile_(
        ultra::timing::globalDecoderProfile().decode_fixed_frame_total);

    cw_count = sanitizeFixedFrameCodewords(cw_count);
    const int codeword_bits = kLdpcBlockCols * lifting_z;  // 648 @ Z27, 1944 @ Z81
    const size_t total_frame_bits =
        static_cast<size_t>(FrameInterleaver::totalFrameBits(cw_count, codeword_bits));

    CodewordStatus status;
    status.fixed_frame = true;
    status.decoded.resize(static_cast<size_t>(cw_count), false);
    status.data.resize(static_cast<size_t>(cw_count));
    status.iterations.assign(static_cast<size_t>(cw_count), 0);
    status.unsatisfied_checks.assign(static_cast<size_t>(cw_count), -1);
    status.llr_abs_mean.assign(static_cast<size_t>(cw_count), 0.0f);
    status.llr_abs_min.assign(static_cast<size_t>(cw_count), 0.0f);
    status.llr_abs_p10.assign(static_cast<size_t>(cw_count), 0.0f);
    status.llr_abs_p50.assign(static_cast<size_t>(cw_count), 0.0f);
    status.llr_abs_p90.assign(static_cast<size_t>(cw_count), 0.0f);
    status.used_perturbation.assign(static_cast<size_t>(cw_count), 0);
    status.harq_attempts.assign(static_cast<size_t>(cw_count), 0);

    // Check we have enough soft bits
    if (interleaved_soft.size() < total_frame_bits) {
        return status;  // All failed - not enough data
    }

    const bool harq_has_key =
        harq_buffer && harq_key && harq_key->sender_hash != 0;
    const bool harq_active = harq_has_key && harq_buffer->enabled();

    // Deinterleave to restore original CW order (frame-level). 2026-05-28:
    // symmetric with the encoder — when cw_count==1 the frame-level interleaver
    // was skipped, so the incoming soft bits ARE the lone codeword's soft bits.
    std::vector<std::vector<float>> cw_soft_bits;
    if (cw_count == 1) {
        cw_soft_bits.emplace_back(interleaved_soft.begin(),
                                  interleaved_soft.begin() + std::min(
                                      interleaved_soft.size(),
                                      static_cast<size_t>(codeword_bits)));
    } else {
        cw_soft_bits = FrameInterleaver::deinterleave(interleaved_soft, cw_count, codeword_bits);
    }

    // Create channel interleaver for deinterleaving if enabled
    std::unique_ptr<ChannelInterleaver> interleaver;
    if (use_channel_deinterleave) {
        interleaver = std::make_unique<ChannelInterleaver>(bits_per_symbol, codeword_bits);
    }

    // Decode each codeword
    // Use min-sum factor 0.9375 (closer to BP) as default — empirically best
    // for DQPSK differential LLRs on fading channels. Both supported lifting
    // geometries use the same rate/Z-keyed thread-local cache; rebuilding the
    // Z=81 matrix per frame is too expensive for the live audio deadline.
    LDPCDecoder& decoder = fixedFrameDecoderForRate(rate, lifting_z);
    size_t bytes_per_cw = (lifting_z == 27) ? getBytesPerCodeword(rate)
                                            : infoBytesPerCodewordZ(rate, lifting_z);

    std::vector<std::vector<float>> decoder_soft_bits(static_cast<size_t>(cw_count));
    std::vector<int> harq_attempts(static_cast<size_t>(cw_count), 1);
    // Preserve the actual uncombined observation for every true combine hit.
    // It is decoded eagerly only for a failed sum or the default-OFF shadow;
    // on a frame-CRC failure it is decoded lazily to enforce the all-fresh
    // production fallback without taxing successful normal traffic.
    std::vector<std::vector<float>> harq_fresh_observations(
        static_cast<size_t>(cw_count));
    std::vector<std::vector<float>> harq_combined_observations(
        static_cast<size_t>(cw_count));
    std::vector<bool> harq_fresh_evaluated(static_cast<size_t>(cw_count), false);
    std::vector<bool> harq_fresh_decoded(static_cast<size_t>(cw_count), false);
    std::vector<Bytes> harq_fresh_data(static_cast<size_t>(cw_count));
    std::vector<int> harq_fresh_iterations(static_cast<size_t>(cw_count), 0);
    std::vector<int> harq_fresh_unsatisfied(static_cast<size_t>(cw_count), -1);
    std::vector<uint8_t> harq_fresh_used_perturbation(
        static_cast<size_t>(cw_count), 0);
    std::vector<bool> harq_selected_fresh(static_cast<size_t>(cw_count), false);

    auto keyForCodeword = [&](int cw) {
        fec::SoftCombineBuffer::Key cw_key = *harq_key;
        cw_key.cw_index = static_cast<uint8_t>(std::clamp(cw, 0, 255));
        return cw_key;
    };

    auto finalize_harq = [&](const CodewordStatus& final_status) {
        if (!harq_has_key) {
            return;
        }
        // Provisional-key finalize guard (2026-07-01, design-review MANDATORY):
        // if the key was position-predicted and the frame's header CW decoded,
        // verify the prediction against the full header-visible identity. On mismatch, touch
        // NOTHING under this key — a drop would destroy the real seq's
        // legitimate accumulation, and a retain would poison it.
        if (harq_key_provisional && !final_status.decoded.empty() &&
            final_status.decoded[0] && !final_status.data.empty() &&
            !final_status.data[0].empty()) {
            const auto& cw0 = final_status.data[0];
            const auto hdr = parseHeader(cw0);
            if (isOFDMBurstPadHeader(hdr)) {
                LOG_MODEM(INFO,
                          "HARQ provisional finalize guard: decoded physical "
                          "ULPAD seq=%u dst=0x%06X — skipping drop/retain",
                          hdr.seq, hdr.dst_hash);
                return;
            }
            const bool actual_tail =
                cw0.size() > 3 &&
                (cw0[3] & Flags::PHYSICAL_BURST_END) != 0;
            const bool identity_matches =
                hdr.valid && !hdr.is_control &&
                fec::SoftCombineBuffer::provisionalHeaderIdentityMatchesKey(
                    *harq_key, hdr.src_hash, hdr.dst_hash, hdr.seq,
                    hdr.total_cw, actual_tail);
            if (!identity_matches) {
                if (hdr.valid) {
                    ultra::timing::globalDecoderProfile()
                        .harq_prediction_mismatch.fetch_add(
                            1, std::memory_order_relaxed);
                }
                LOG_MODEM(INFO,
                          "HARQ provisional finalize guard: predicted "
                          "src=0x%06X dst=0x%06X seq=%u cw=%u tail=%u but decoded "
                          "src=0x%06X dst=0x%06X seq=%u cw=%u tail=%u valid=%d control=%d "
                          "— skipping drop/retain",
                          harq_key->sender_hash, harq_key->dst_hash,
                          harq_key->seq, harq_key->cw_count,
                          harq_key->physical_burst_end, hdr.src_hash,
                          hdr.dst_hash, hdr.seq, hdr.total_cw,
                          actual_tail ? 1 : 0, hdr.valid ? 1 : 0,
                          hdr.is_control ? 1 : 0);
                return;
            }
        }
        for (int cw = 0; cw < cw_count; ++cw) {
            const auto cw_key = keyForCodeword(cw);
            if (cw < static_cast<int>(final_status.decoded.size()) &&
                final_status.decoded[static_cast<size_t>(cw)]) {
                if (harqDebugKeySelected(cw_key)) {
                    LOG_MODEM(WARN,
                              "HARQ_DEBUG finalize action=drop seq=%u cw=%d/%u frame_all_success=%d decoded=1",
                              cw_key.seq, cw, cw_key.cw_count, final_status.allSuccess() ? 1 : 0);
                }
                harq_buffer->drop(cw_key);
            } else if (harq_active &&
                       cw < static_cast<int>(decoder_soft_bits.size()) &&
                       !decoder_soft_bits[static_cast<size_t>(cw)].empty()) {
                if (harqDebugKeySelected(cw_key)) {
                    LOG_MODEM(WARN,
                              "HARQ_DEBUG finalize action=retain seq=%u cw=%d/%u frame_all_success=%d decoded=0 mean_abs=%.3f",
                              cw_key.seq, cw, cw_key.cw_count,
                              final_status.allSuccess() ? 1 : 0,
                              meanAbsLlr(decoder_soft_bits[static_cast<size_t>(cw)]));
                }
                harq_buffer->retain(cw_key, decoder_soft_bits[static_cast<size_t>(cw)],
                                    harq_key_provisional);
            }
        }
    };

    for (int cw = 0; cw < cw_count; ++cw) {
        auto cw_bits = cw_soft_bits[cw];

        // Apply channel deinterleaving if enabled
        if (use_channel_deinterleave && interleaver) {
            cw_bits = interleaver->deinterleave(cw_bits);
        }

        // Fresh-only copy of THIS attempt's LLRs, kept whenever combining
        // replaces cw_bits — the combined sum can be POISONED (a stored copy
        // with confidently-wrong LLRs actively fights a good fresh copy; the
        // 2026-07-01 rig poison-loop), so a combined-and-failed CW gets one
        // standalone pass on the fresh bits below. This makes combining
        // harm-free by construction regardless of stored-copy quality.
        std::vector<float> fresh_cw_bits;
        if (harq_active) {
            std::vector<float> combined_cw_bits;
            const auto cw_key = keyForCodeword(cw);
            const int attempts = harq_buffer->combine(cw_key, cw_bits, combined_cw_bits);
            harq_attempts[static_cast<size_t>(cw)] = attempts;
            // combine() intentionally returns the incoming vector on a miss.
            // Only attempts>1 proves that stored state actually replaced it;
            // treating a miss as a combine needlessly ran the fresh fallback
            // on every ordinary first attempt.
            if (attempts > 1 && !combined_cw_bits.empty()) {
                fresh_cw_bits = cw_bits;  // preserve the un-combined copy
                harq_fresh_observations[static_cast<size_t>(cw)] = cw_bits;
                harq_combined_observations[static_cast<size_t>(cw)] =
                    combined_cw_bits;
                cw_bits = std::move(combined_cw_bits);
            }
            if (attempts > 1) {
                LOG_MODEM(INFO,
                          "HARQ: combining attempt %d for seq=%u cw=%d/%u (sender_hash=0x%06X)",
                          attempts, harq_key->seq, cw, harq_key->cw_count,
                          harq_key->sender_hash);
            }
        }
        decoder_soft_bits[static_cast<size_t>(cw)] = cw_bits;

        // Debug: check LLR statistics for this CW
        float llr_sum = 0.0f;
        for (float llr : cw_bits) {
            llr_sum += llr;
        }
        float llr_avg = cw_bits.empty() ? 0.0f : llr_sum / cw_bits.size();
        auto llr_summary = summarizeAbsLlrs(cw_bits);

        auto sum_result = decodeWithFullFixedFrameSchedule(
            decoder, cw_bits, /*profile_timing=*/true);
        std::vector<uint8_t> decoded = std::move(sum_result.decoded);
        bool success = sum_result.success && decoded.size() >= bytes_per_cw;
        int iterations = sum_result.iterations;
        int final_unsatisfied = sum_result.unsatisfied_checks;
        bool used_perturbation = sum_result.used_perturbation;
        const bool combined_sum_success = !fresh_cw_bits.empty() && success;

        if (success && sum_result.attempts > 1) {
            LOG_MODEM(INFO,
                      "CW[%d]: RETRY OK (factor=%.4f sigma=%.1f attempts=%d "
                      "iters=%d)",
                      cw, sum_result.winning_factor,
                      sum_result.winning_sigma, sum_result.attempts, iterations);
        }

        // Phase F — exact FRESH-ONLY rescue. The previous three-pass subset
        // could still make combining harmful: an ordinary fresh observation
        // receives primary + four factor + five perturbation attempts, while a
        // fresh copy behind a failed sum did not. Use the identical schedule so
        // enabling HARQ cannot suppress a decode the normal path would find.
        if (!success && !fresh_cw_bits.empty()) {
            auto fresh_result = decodeWithFullFixedFrameSchedule(
                decoder, fresh_cw_bits, /*profile_timing=*/true);
            iterations = fresh_result.iterations;
            final_unsatisfied = fresh_result.unsatisfied_checks;
            const bool fresh_has_bytes =
                fresh_result.success && fresh_result.decoded.size() >= bytes_per_cw;
            harq_fresh_evaluated[static_cast<size_t>(cw)] = true;
            harq_fresh_decoded[static_cast<size_t>(cw)] = fresh_has_bytes;
            harq_fresh_iterations[static_cast<size_t>(cw)] = fresh_result.iterations;
            harq_fresh_unsatisfied[static_cast<size_t>(cw)] =
                fresh_result.unsatisfied_checks;
            harq_fresh_used_perturbation[static_cast<size_t>(cw)] =
                fresh_result.used_perturbation ? 1 : 0;
            if (fresh_has_bytes) {
                harq_fresh_data[static_cast<size_t>(cw)].assign(
                    fresh_result.decoded.begin(),
                    fresh_result.decoded.begin() + bytes_per_cw);
                decoded = std::move(fresh_result.decoded);
                success = true;
                used_perturbation = fresh_result.used_perturbation;
                decoder_soft_bits[static_cast<size_t>(cw)] = fresh_cw_bits;
                harq_selected_fresh[static_cast<size_t>(cw)] = true;
                llr_sum = 0.0f;
                for (float llr : fresh_cw_bits) {
                    llr_sum += llr;
                }
                llr_avg = fresh_cw_bits.empty()
                              ? 0.0f
                              : llr_sum / fresh_cw_bits.size();
                llr_summary = summarizeAbsLlrs(fresh_cw_bits);
                LOG_MODEM(INFO,
                          "CW[%d]: FRESH-ONLY CANDIDATE (sum syndrome failed; "
                          "fresh syndrome passed factor=%.4f sigma=%.1f "
                          "iters=%d provisional=%d) — awaiting frame CRC",
                          cw, fresh_result.winning_factor,
                          fresh_result.winning_sigma, iterations,
                          harq_key_provisional ? 1 : 0);
            } else {
                // If BOTH the sum and fresh copy failed under a PROVISIONAL
                // (unverified) key, retain FRESH instead of the sum. Resetting
                // a suspect accumulator to the latest copy caps poison
                // persistence at one round. Header-verified keys keep the sum
                // because their Chase identity is authoritative.
                if (harq_key_provisional) {
                    decoder_soft_bits[static_cast<size_t>(cw)] = fresh_cw_bits;
                }
            }
        }

        // Default-OFF causal shadow. A successful combined sum alone does not
        // prove HARQ helped because the fresh observation may also have passed.
        // This only captures the candidate here; classification is deferred to
        // full header+frame-CRC validation below.
        if (combined_sum_success && harqShadowFreshEnabled()) {
            FullScheduleDecodeResult shadow;
            {
                ultra::timing::ScopedTimer timer(
                    ultra::timing::globalDecoderProfile()
                        .harq_shadow_fresh_decode);
                LDPCDecoder& shadow_decoder =
                    shadowFixedFrameDecoderForRate(rate, lifting_z);
                shadow = decodeWithFullFixedFrameSchedule(
                    shadow_decoder, fresh_cw_bits, /*profile_timing=*/false);
            }
            const bool fresh_has_bytes =
                shadow.success && shadow.decoded.size() >= bytes_per_cw;
            harq_fresh_evaluated[static_cast<size_t>(cw)] = true;
            harq_fresh_decoded[static_cast<size_t>(cw)] = fresh_has_bytes;
            harq_fresh_iterations[static_cast<size_t>(cw)] = shadow.iterations;
            harq_fresh_unsatisfied[static_cast<size_t>(cw)] =
                shadow.unsatisfied_checks;
            harq_fresh_used_perturbation[static_cast<size_t>(cw)] =
                shadow.used_perturbation ? 1 : 0;
            if (fresh_has_bytes) {
                harq_fresh_data[static_cast<size_t>(cw)].assign(
                    shadow.decoded.begin(), shadow.decoded.begin() + bytes_per_cw);
            }
            const auto cw_key = keyForCodeword(cw);
            LOG_MODEM(INFO,
                      "HARQ_SHADOW seq=%u cw=%d/%u attempts=%d sum_pass=1 "
                      "fresh_syndrome=%d provisional=%d fresh_iters=%d "
                      "class=pending_frame_crc",
                      cw_key.seq, cw, cw_key.cw_count,
                      harq_attempts[static_cast<size_t>(cw)],
                      fresh_has_bytes ? 1 : 0,
                      harq_key_provisional ? 1 : 0,
                      shadow.iterations);
        }

        const int final_iterations = iterations;

        LOG_MODEM(INFO, "CW[%d]: %s (iters=%d, unsat=%d, llr_avg=%.2f, |llr|=mean %.2f p10 %.2f p50 %.2f p90 %.2f)",
                  cw, success ? "OK" : "FAIL", final_iterations, final_unsatisfied,
                  llr_avg, llr_summary.mean, llr_summary.p10,
                  llr_summary.p50, llr_summary.p90);
        if (harq_active) {
            const auto cw_key = keyForCodeword(cw);
            if (harq_attempts[static_cast<size_t>(cw)] > 1 && harqDebugKeySelected(cw_key)) {
                const char* winner = combined_sum_success
                                         ? "sum"
                                         : (harq_selected_fresh[static_cast<size_t>(cw)]
                                                ? "fresh"
                                                : "none");
                LOG_MODEM(WARN,
                          "HARQ_DEBUG decode_after_combine seq=%u cw=%d/%u "
                          "attempts=%d sum_success=%d final_success=%d "
                          "winner=%s iters=%d selected_mean_abs=%.3f "
                          "perturbation=%d provisional=%d",
                          cw_key.seq, cw, cw_key.cw_count,
                          harq_attempts[static_cast<size_t>(cw)],
                          combined_sum_success ? 1 : 0, success ? 1 : 0,
                          winner, iterations,
                          meanAbsLlr(decoder_soft_bits[static_cast<size_t>(cw)]),
                          used_perturbation ? 1 : 0,
                          harq_key_provisional ? 1 : 0);
            }
        }

        status.decoded[cw] = success;
        status.iterations[static_cast<size_t>(cw)] = final_iterations;
        status.unsatisfied_checks[static_cast<size_t>(cw)] = final_unsatisfied;
        status.llr_abs_mean[static_cast<size_t>(cw)] = llr_summary.mean;
        status.llr_abs_min[static_cast<size_t>(cw)] = llr_summary.min;
        status.llr_abs_p10[static_cast<size_t>(cw)] = llr_summary.p10;
        status.llr_abs_p50[static_cast<size_t>(cw)] = llr_summary.p50;
        status.llr_abs_p90[static_cast<size_t>(cw)] = llr_summary.p90;
        status.used_perturbation[static_cast<size_t>(cw)] =
            used_perturbation ? static_cast<uint8_t>(1) : static_cast<uint8_t>(0);
        status.harq_attempts[static_cast<size_t>(cw)] =
            harq_attempts[static_cast<size_t>(cw)];
        if (success && decoded.size() >= bytes_per_cw) {
            // Take exactly bytes_per_cw bytes
            status.data[cw].assign(decoded.begin(), decoded.begin() + bytes_per_cw);
        }
    }

    // ========================================================================
    // FRAME-VALIDATED FALSE-POSITIVE RECOVERY + HARQ COUNTERFACTUAL
    // ========================================================================
    // LDPC syndrome success is not payload correctness. Every production and
    // all-fresh verdict below is gated by the complete header and frame CRC.
    // This helper is the historical frame-level recovery, parameterized by the
    // observation set so the HARQ counterfactual receives exactly the same
    // treatment as the normal path.
    auto recoverFrameCandidate = [&](CodewordStatus& candidate,
                                     const std::vector<std::vector<float>>& soft_bits,
                                     const char* label,
                                     LDPCDecoder& recovery_decoder,
                                     ultra::timing::PhaseStats* recovery_timing) -> bool {
        if (!candidate.allSuccess()) {
            return false;
        }
        if (fixedFramePayloadIsValid(candidate.reassemble())) {
            return true;
        }

        int candidate_perturbed_cws = 0;
        for (uint8_t used : candidate.used_perturbation) {
            candidate_perturbed_cws += used != 0 ? 1 : 0;
        }
        LOG_MODEM(WARN,
                  "LDPC false positive (%s): all CW syndromes passed but frame "
                  "CRC failed (perturbed_cws=%d)",
                  label, candidate_perturbed_cws);

        // A perturbation-produced wrong codeword is random code-space output;
        // preserve the existing integrity rule and never search around it.
        if (candidate_perturbed_cws > 0) {
            LOG_MODEM(WARN,
                      "LDPC false positive (%s): perturbation involved; "
                      "marking frame failed",
                      label);
            for (auto&& decoded_ok : candidate.decoded) {
                decoded_ok = false;
            }
            return false;
        }

        // Bit-flip salvage remains deliberately removed: a syndrome-valid
        // wrong LDPC codeword differs by at least the code distance, while the
        // old CRC-syndrome search delivered a byte-corrupt file. Re-decoding
        // an entire CW under four deterministic factors is the only fallback.
        static constexpr std::array<float, 4> recovery_factors = {
            0.75f, 0.625f, 0.5f, 0.875f};
        for (float factor : recovery_factors) {
            for (int cw = 0; cw < cw_count; ++cw) {
                const auto& candidate_bits = soft_bits[static_cast<size_t>(cw)];
                const auto original_data = candidate.data[static_cast<size_t>(cw)];
                recovery_decoder.setMinSumFactor(factor);
                std::vector<uint8_t> re_decoded;
                if (recovery_timing) {
                    // The caller times the complete counterfactual frame
                    // evaluation, including cache construction, CRC, and every
                    // recovery call. Do not double-count individual LDPC calls.
                    re_decoded = recovery_decoder.decodeSoft(candidate_bits);
                } else {
                    ultra::timing::ScopedTimer timer(
                        ultra::timing::globalDecoderProfile().ldpc_cw_total);
                    re_decoded = recovery_decoder.decodeSoft(candidate_bits);
                }
                if (!recovery_decoder.lastDecodeSuccess() ||
                    re_decoded.size() < bytes_per_cw) {
                    continue;
                }
                Bytes replacement(re_decoded.begin(),
                                  re_decoded.begin() + bytes_per_cw);
                if (replacement == original_data) {
                    continue;
                }
                candidate.data[static_cast<size_t>(cw)] = std::move(replacement);
                if (fixedFramePayloadIsValid(candidate.reassemble())) {
                    recovery_decoder.setMinSumFactor(
                        kFixedFrameDefaultMinSumFactor);
                    LOG_MODEM(INFO,
                              "CW[%d]: FALSE POSITIVE RECOVERED (%s factor=%.3f)",
                              cw, label, factor);
                    return true;
                }
                candidate.data[static_cast<size_t>(cw)] = original_data;
            }
        }
        recovery_decoder.setMinSumFactor(kFixedFrameDefaultMinSumFactor);

        LOG_MODEM(WARN,
                  "LDPC false positive (%s): recovery FAILED, marking frame "
                  "as decode failure",
                  label);
        for (auto&& decoded_ok : candidate.decoded) {
            decoded_ok = false;
        }
        return false;
    };

    const bool had_combine_hit = std::any_of(
        harq_fresh_observations.begin(), harq_fresh_observations.end(),
        [](const auto& bits) { return !bits.empty(); });
    // Avoid deep-copying every CW LLR vector on the default/no-hit path. These
    // snapshots exist only to build a real HARQ counterfactual.
    CodewordStatus pre_recovery_status;
    std::vector<std::vector<float>> pre_recovery_soft_bits;
    if (had_combine_hit) {
        pre_recovery_status = status;
        pre_recovery_soft_bits = decoder_soft_bits;
    }
    bool production_frame_valid =
        recoverFrameCandidate(status, decoder_soft_bits, "production",
                              decoder,
                              /*recovery_timing=*/nullptr);

    const bool selected_fresh_candidate =
        had_combine_hit && std::any_of(
                               harq_selected_fresh.begin(),
                               harq_selected_fresh.end(),
                               [](bool selected) { return selected; });
    const bool evaluate_all_fresh =
        had_combine_hit && (!production_frame_valid || harqShadowFreshEnabled());

    CodewordStatus all_fresh_status;
    std::vector<std::vector<float>> all_fresh_soft_bits;
    bool all_fresh_frame_valid = false;
    if (evaluate_all_fresh) {
        // Complete any counterfactual CWs not already evaluated by failed-sum
        // fallback or the eager default-OFF shadow diagnostic.
        for (int cw = 0; cw < cw_count; ++cw) {
            const size_t idx = static_cast<size_t>(cw);
            if (harq_fresh_observations[idx].empty() ||
                harq_fresh_evaluated[idx]) {
                continue;
            }
            FullScheduleDecodeResult fresh_result;
            {
                ultra::timing::ScopedTimer timer(
                    ultra::timing::globalDecoderProfile()
                        .harq_lazy_fresh_decode);
                LDPCDecoder& counterfactual_decoder =
                    shadowFixedFrameDecoderForRate(rate, lifting_z);
                fresh_result = decodeWithFullFixedFrameSchedule(
                    counterfactual_decoder, harq_fresh_observations[idx],
                    /*profile_timing=*/false);
            }
            const bool fresh_has_bytes =
                fresh_result.success && fresh_result.decoded.size() >= bytes_per_cw;
            harq_fresh_evaluated[idx] = true;
            harq_fresh_decoded[idx] = fresh_has_bytes;
            harq_fresh_iterations[idx] = fresh_result.iterations;
            harq_fresh_unsatisfied[idx] = fresh_result.unsatisfied_checks;
            harq_fresh_used_perturbation[idx] =
                fresh_result.used_perturbation ? 1 : 0;
            if (fresh_has_bytes) {
                harq_fresh_data[idx].assign(
                    fresh_result.decoded.begin(),
                    fresh_result.decoded.begin() + bytes_per_cw);
            }
        }

        all_fresh_status = pre_recovery_status;
        all_fresh_soft_bits = pre_recovery_soft_bits;
        for (int cw = 0; cw < cw_count; ++cw) {
            const size_t idx = static_cast<size_t>(cw);
            if (harq_fresh_observations[idx].empty()) {
                continue;
            }
            all_fresh_soft_bits[idx] = harq_fresh_observations[idx];
            all_fresh_status.decoded[idx] = harq_fresh_decoded[idx];
            all_fresh_status.data[idx] = harq_fresh_data[idx];
            all_fresh_status.iterations[idx] = harq_fresh_iterations[idx];
            all_fresh_status.unsatisfied_checks[idx] = harq_fresh_unsatisfied[idx];
            all_fresh_status.used_perturbation[idx] =
                harq_fresh_used_perturbation[idx];
            const auto summary =
                summarizeAbsLlrs(harq_fresh_observations[idx]);
            all_fresh_status.llr_abs_mean[idx] = summary.mean;
            all_fresh_status.llr_abs_min[idx] = summary.min;
            all_fresh_status.llr_abs_p10[idx] = summary.p10;
            all_fresh_status.llr_abs_p50[idx] = summary.p50;
            all_fresh_status.llr_abs_p90[idx] = summary.p90;
        }
        auto& frame_eval_stats = production_frame_valid
                                     ? ultra::timing::globalDecoderProfile()
                                           .harq_shadow_frame_evaluation
                                     : ultra::timing::globalDecoderProfile()
                                           .harq_lazy_frame_evaluation;
        {
            ultra::timing::ScopedTimer timer(frame_eval_stats);
            LDPCDecoder& all_fresh_recovery_decoder =
                shadowFixedFrameDecoderForRate(rate, lifting_z);
            all_fresh_frame_valid = recoverFrameCandidate(
                all_fresh_status, all_fresh_soft_bits,
                "all-fresh HARQ baseline", all_fresh_recovery_decoder,
                &frame_eval_stats);
        }
    }

    auto& harq_profile = ultra::timing::globalDecoderProfile();
    auto countProvisional = [&](std::atomic<uint64_t>& total,
                                std::atomic<uint64_t>& provisional) {
        total.fetch_add(1, std::memory_order_relaxed);
        if (harq_key_provisional) {
            provisional.fetch_add(1, std::memory_order_relaxed);
        }
    };
    auto classificationIdentityMatches = [&](const CodewordStatus& candidate) {
        if (!harq_key_provisional) {
            return true;
        }
        if (!candidate.allSuccess()) {
            return false;
        }
        const Bytes bytes = candidate.reassemble();
        if (!fixedFramePayloadIsValid(bytes)) {
            return false;
        }
        const auto header = parseHeader(bytes);
        if (isOFDMBurstPadHeader(header)) {
            return false;
        }
        const bool physical_tail =
            bytes.size() > 3 &&
            (bytes[3] & Flags::PHYSICAL_BURST_END) != 0;
        return header.valid && !header.is_control &&
               fec::SoftCombineBuffer::provisionalHeaderIdentityMatchesKey(
                   *harq_key, header.src_hash, header.dst_hash, header.seq,
                   header.total_cw, physical_tail);
    };
    const bool production_identity_matches =
        production_frame_valid && classificationIdentityMatches(status);
    const bool all_fresh_identity_matches =
        all_fresh_frame_valid &&
        classificationIdentityMatches(all_fresh_status);

    bool used_all_fresh_frame_rescue = false;
    if (had_combine_hit && selected_fresh_candidate && production_frame_valid) {
        // At least one combined sum had no valid LDPC codeword, and replacing
        // it with the exact fresh baseline produced a CRC-valid frame.
        if (production_identity_matches) {
            countProvisional(harq_profile.harq_fresh_rescue,
                             harq_profile.harq_fresh_rescue_provisional);
            LOG_MODEM(INFO,
                      "HARQ_FRAME class=fresh_rescue provisional=%d seq=%u "
                      "reason=sum_syndrome_failed",
                      harq_key_provisional ? 1 : 0, harq_key->seq);
        } else {
            LOG_MODEM(INFO,
                      "HARQ_FRAME class=identity_rejected provisional=1 seq=%u "
                      "candidate=hybrid_fresh_rescue",
                      harq_key->seq);
        }
    } else if (had_combine_hit && !production_frame_valid &&
               all_fresh_frame_valid) {
        // Covers the subtle wrong-but-syndrome-valid sum: production failed
        // complete frame validation, while the lazily evaluated all-fresh path
        // passed the same decoder and frame-recovery schedule.
        status = std::move(all_fresh_status);
        decoder_soft_bits = std::move(all_fresh_soft_bits);
        production_frame_valid = true;
        used_all_fresh_frame_rescue = true;
        if (all_fresh_identity_matches) {
            countProvisional(harq_profile.harq_fresh_rescue,
                             harq_profile.harq_fresh_rescue_provisional);
            countProvisional(
                harq_profile.harq_all_fresh_frame_rescue,
                harq_profile.harq_all_fresh_frame_rescue_provisional);
            LOG_MODEM(INFO,
                      "HARQ_FRAME class=fresh_rescue provisional=%d seq=%u "
                      "reason=combined_frame_crc_failed",
                      harq_key_provisional ? 1 : 0, harq_key->seq);
        } else {
            LOG_MODEM(INFO,
                      "HARQ_FRAME class=identity_rejected provisional=1 seq=%u "
                      "candidate=all_fresh_frame_rescue",
                      harq_key->seq);
        }
    } else if (had_combine_hit && !production_frame_valid &&
               evaluate_all_fresh && !all_fresh_frame_valid) {
        countProvisional(harq_profile.harq_double_fail,
                         harq_profile.harq_double_fail_provisional);
        // A provisional accumulator is not authoritative. On a double-fail,
        // retain the current fresh evidence rather than persisting a suspect
        // sum into the next retry. Verified keys keep their Chase sum.
        if (harq_key_provisional) {
            decoder_soft_bits = all_fresh_soft_bits;
        } else {
            // A fresh syndrome candidate may have temporarily replaced the
            // selected decode input before the complete frame CRC failed. For
            // a header-verified key the Chase identity is authoritative: retain
            // the accumulated sum, never silently downgrade it to one fresh
            // observation on a hybrid-frame double failure.
            for (int cw = 0; cw < cw_count; ++cw) {
                const size_t idx = static_cast<size_t>(cw);
                if (!harq_combined_observations[idx].empty()) {
                    decoder_soft_bits[idx] = harq_combined_observations[idx];
                }
            }
        }
        LOG_MODEM(INFO,
                  "HARQ_FRAME class=double_fail provisional=%d seq=%u",
                  harq_key_provisional ? 1 : 0, harq_key->seq);
    }

    // Only frames whose production decision used successful sums throughout
    // are eligible for the shadow both/combine-only question. Fresh-rescued
    // production frames answer the opposite direction and are counted above.
    if (had_combine_hit && harqShadowFreshEnabled() &&
        !selected_fresh_candidate && production_frame_valid &&
        production_identity_matches && evaluate_all_fresh &&
        !used_all_fresh_frame_rescue) {
        countProvisional(harq_profile.harq_shadow_eligible,
                         harq_profile.harq_shadow_eligible_provisional);
        const Bytes production_bytes = status.reassemble();
        const Bytes fresh_bytes =
            all_fresh_frame_valid ? all_fresh_status.reassemble() : Bytes{};
        if (!all_fresh_frame_valid) {
            countProvisional(harq_profile.harq_shadow_combine_only,
                             harq_profile.harq_shadow_combine_only_provisional);
            LOG_MODEM(INFO,
                      "HARQ_FRAME class=combine_only provisional=%d seq=%u",
                      harq_key_provisional ? 1 : 0, harq_key->seq);
        } else if (all_fresh_identity_matches &&
                   fresh_bytes == production_bytes) {
            countProvisional(harq_profile.harq_shadow_both_pass,
                             harq_profile.harq_shadow_both_pass_provisional);
            LOG_MODEM(INFO,
                      "HARQ_FRAME class=both_pass provisional=%d seq=%u",
                      harq_key_provisional ? 1 : 0, harq_key->seq);
        } else {
            // Two distinct CRC-valid frames are not causal evidence; preserve
            // production but flag the ambiguity rather than calling it a win.
            countProvisional(harq_profile.harq_shadow_divergent,
                             harq_profile.harq_shadow_divergent_provisional);
            LOG_MODEM(ERROR,
                      "HARQ_FRAME class=divergent_valid provisional=%d seq=%u",
                      harq_key_provisional ? 1 : 0, harq_key->seq);
        }
    }

    finalize_harq(status);
    return status;
}

CodewordStatus decodeFixedFrame(const std::vector<float>& interleaved_soft, CodeRate rate,
                                bool use_channel_deinterleave, size_t bits_per_symbol) {
    return decodeFixedFrame(interleaved_soft, rate, kDefaultFixedFrameCodewords,
                            use_channel_deinterleave, bits_per_symbol);
}

CodewordStatus decodeFixedFrame(const std::vector<float>& interleaved_soft, CodeRate rate, int cw_count) {
    return decodeFixedFrame(interleaved_soft, rate, cw_count, false);
}

// Default: no channel deinterleaving (backward compatible)
CodewordStatus decodeFixedFrame(const std::vector<float>& interleaved_soft, CodeRate rate) {
    return decodeFixedFrame(interleaved_soft, rate, false);
}

DataFrame makeFixedDataFrame(const std::string& src, const std::string& dst,
                              uint16_t seq, const Bytes& payload, CodeRate rate,
                              int cw_count, int lifting_z) {
    cw_count = sanitizeFixedFrameCodewords(cw_count);
    // Z-aware capacity. The legacy getFixedFramePayloadCapacity returns the Z=27
    // size (~96 B at R3/4 cw=2) even when the encoder runs at z=81 (real capacity
    // ~340 B). The caller passes the active lifting_z (Connection::selectBurstLiftingZ);
    // without it a z=81 FILE_DATA chunk gets SILENTLY TRUNCATED to 96 B and the
    // receiver's assembler skips ~244 B/frame — file never assembles even though
    // PHY decode succeeds on every group.
    const size_t capacity = (lifting_z == 81)
        ? getFixedFramePayloadCapacityZ(rate, cw_count, 81)
        : getFixedFramePayloadCapacity(rate, cw_count);

    // Truncate or use as-is
    Bytes actual_payload = payload;
    if (actual_payload.size() > capacity) {
        actual_payload.resize(capacity);
    }

    // Create frame with explicit fixed-frame total_cw
    DataFrame frame;
    frame.type = FrameType::DATA;
    frame.flags = Flags::VERSION_V2;
    frame.seq = seq;
    frame.src_hash = hashCallsign(src);
    frame.dst_hash = hashCallsign(dst);
    frame.payload = actual_payload;
    frame.payload_len = static_cast<uint16_t>(actual_payload.size());
    frame.total_cw = static_cast<uint8_t>(cw_count);

    return frame;
}

} // namespace v2
} // namespace protocol
} // namespace ultra
