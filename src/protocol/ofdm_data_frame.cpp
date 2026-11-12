// Compact OFDM DATA frame — see ofdm_data_frame.hpp.
#include "ofdm_data_frame.hpp"

#include "fec/turbo_codec.hpp"
#include "mcdpsk_frame.hpp"

#include <algorithm>

namespace ultra {
namespace protocol {
namespace ofdm_frame {

namespace {
constexpr int kBlockCols = 24;  // a 648-bit (Z=27) or 1944-bit (Z=81) air block = 24 x Z bits

uint16_t headerCrc(const uint8_t* header6, uint32_t seed) {
    return static_cast<uint16_t>(v2::ControlFrame::calculateCRC(header6, 6) ^ (seed & 0xFFFFu));
}
}  // namespace

uint8_t packPosition(const Position& pos) {
    if (!pos.known() || pos.count > 16) return kPositionUnknown;
    return static_cast<uint8_t>((pos.index << 4) | (pos.count - 1));
}

Position unpackPosition(uint8_t byte) {
    Position p;
    p.index = byte >> 4;
    p.count = (byte & 0x0F) + 1;
    if (!p.known()) return Position{};
    return p;
}

uint32_t sessionCrcSeed(uint32_t src_hash, uint32_t dst_hash) {
    // The MC-DPSK seed for the pair, mixed with a fixed OFDM constant: the two waveforms' frames
    // never verify as each other's (they also differ in length).
    const uint32_t seed = mcdpsk_frame::sessionCrcSeed(src_hash, dst_hash) ^ 0x4F46444Du;  // "OFDM"
    return seed != 0 ? seed : 1u;
}

size_t infoBytesFor(CodeRate rate, int cw_count, int lifting_z) {
    return v2::ofdmFixedFrameInfoBytes(rate, cw_count, lifting_z);
}

std::optional<Bytes> encode(ByteSpan v2_data_frame, CodeRate rate, int cw_count, int lifting_z,
                            const Position& position) {
    const auto frame = v2::DataFrame::deserialize(v2_data_frame);
    if (!frame || frame->type != v2::FrameType::DATA) {
        return std::nullopt;
    }
    cw_count = v2::sanitizeFixedFrameCodewords(cw_count);
    lifting_z = (lifting_z == 81) ? 81 : 27;
    const size_t info = infoBytesFor(rate, cw_count, lifting_z);
    if (info <= kHeaderBytes || frame->payload.size() > info - kHeaderBytes) {
        return std::nullopt;
    }
    const uint32_t seed = sessionCrcSeed(frame->src_hash, frame->dst_hash);
    Bytes block(kHeaderBytes);
    block[0] = frame->flags;
    block[1] = static_cast<uint8_t>(frame->seq >> 8);
    block[2] = static_cast<uint8_t>(frame->seq & 0xFF);
    block[3] = packPosition(position);
    block[4] = static_cast<uint8_t>(frame->payload.size() >> 8);
    block[5] = static_cast<uint8_t>(frame->payload.size() & 0xFF);
    const uint16_t hcrc = headerCrc(block.data(), seed);
    block[6] = static_cast<uint8_t>(hcrc >> 8);
    block[7] = static_cast<uint8_t>(hcrc & 0xFF);
    block.insert(block.end(), frame->payload.begin(), frame->payload.end());
    return fec::turbo::encode(v2::padFixedFrameInfo(block, info),
                              cw_count * kBlockCols * lifting_z, seed);
}

std::optional<Header> parseHeader(const Bytes& info, CodeRate rate, int cw_count, int lifting_z,
                                  uint32_t src_hash, uint32_t dst_hash) {
    if (info.size() < kHeaderBytes) return std::nullopt;
    const uint32_t seed = sessionCrcSeed(src_hash, dst_hash);
    const uint16_t got = static_cast<uint16_t>((info[6] << 8) | info[7]);
    if (headerCrc(info.data(), seed) != got) return std::nullopt;
    Header h;
    h.flags = info[0];
    h.seq = static_cast<uint16_t>((info[1] << 8) | info[2]);
    h.position = unpackPosition(info[3]);
    h.length = static_cast<uint16_t>((info[4] << 8) | info[5]);
    h.header_crc = got;
    const size_t capacity = infoBytesFor(rate, v2::sanitizeFixedFrameCodewords(cw_count),
                                         (lifting_z == 81) ? 81 : 27);
    if (capacity <= kHeaderBytes || h.length > capacity - kHeaderBytes) return std::nullopt;
    return h;
}

DecodeResult decode(std::span<const float> air_llrs, CodeRate rate, int cw_count, int lifting_z,
                    uint32_t src_hash, uint32_t dst_hash, bool session_known) {
    DecodeResult out;
    cw_count = v2::sanitizeFixedFrameCodewords(cw_count);
    lifting_z = (lifting_z == 81) ? 81 : 27;
    const size_t info = infoBytesFor(rate, cw_count, lifting_z);
    const size_t air_bits = static_cast<size_t>(cw_count * kBlockCols * lifting_z);
    if (info <= kHeaderBytes || air_llrs.size() < air_bits) {
        return out;
    }
    fec::turbo::DecodeOptions opts;
    if (session_known) {
        opts.crc_seed = sessionCrcSeed(src_hash, dst_hash);
        opts.check_alt_crc_seed = true;  // the v2 layout: the plain CRC
        opts.alt_crc_seed = 0;
    }
    const auto r = fec::turbo::decode(air_llrs.first(air_bits), info, opts);
    out.iterations = r.iterations;
    out.hard_info = r.info_bytes;
    if (!r.crc_ok) {
        return out;
    }
    const bool compact = session_known && !r.alt_crc_seed_matched;
    if (!compact) {
        out.ok = true;
        out.frame = r.info_bytes;  // the v2 frame as sent (+ pad); the caller validates it
        return out;
    }
    const auto h = parseHeader(r.info_bytes, rate, cw_count, lifting_z, src_hash, dst_hash);
    if (!h) {
        return out;  // CRC-32C passed but the header is inconsistent: reject
    }
    v2::DataFrame frame;
    frame.type = v2::FrameType::DATA;
    frame.flags = h->flags;
    frame.seq = h->seq;
    frame.src_hash = src_hash & 0xFFFFFFu;
    frame.dst_hash = dst_hash & 0xFFFFFFu;
    frame.total_cw = static_cast<uint8_t>(cw_count);
    frame.payload_len = h->length;
    frame.payload.assign(r.info_bytes.begin() + kHeaderBytes,
                         r.info_bytes.begin() + kHeaderBytes + h->length);
    out.frame = frame.serialize();
    out.position = h->position;
    out.compact = true;
    out.ok = true;
    return out;
}

v2::CodewordStatus decodeStatus(const std::vector<float>& air_soft, CodeRate rate, int cw_count,
                                int lifting_z, uint32_t src_hash, uint32_t dst_hash,
                                bool session_known, DecodeResult* detail) {
    v2::CodewordStatus status = v2::turboFrameStatusShell(air_soft, cw_count, lifting_z);
    DecodeResult d = decode(std::span<const float>(air_soft.data(), air_soft.size()), rate,
                            cw_count, lifting_z, src_hash, dst_hash, session_known);
    std::fill(status.iterations.begin(), status.iterations.end(), d.iterations);
    if (d.ok) {
        status.data[0] = d.frame;
        std::fill(status.decoded.begin(), status.decoded.end(), true);
        // Defense in depth for the v2 layout (CRC-32C passed): its own header/frame CRCs must too.
        // A compact frame was rebuilt from a header that passed its own CRC.
        if (!d.compact && !v2::DataFrame::deserialize(status.reassemble()).has_value()) {
            std::fill(status.decoded.begin(), status.decoded.end(), false);
            status.data[0].clear();
            d.ok = false;
        } else {
            std::fill(status.unsatisfied_checks.begin(), status.unsatisfied_checks.end(), 0);
        }
    }
    if (detail) *detail = std::move(d);
    return status;
}

}  // namespace ofdm_frame
}  // namespace protocol
}  // namespace ultra
