// Compact MC-DPSK DATA frame — see mcdpsk_frame.hpp.
#include "mcdpsk_frame.hpp"

#include "fec/turbo_codec.hpp"

#include <algorithm>

namespace ultra {
namespace protocol {
namespace mcdpsk_frame {

namespace {

// Exact code-rate fraction (getCodeRateValue is a float): R1/4 of 1944 bits is 486 bits.
bool rateFraction(CodeRate rate, int& num, int& den) {
    switch (rate) {
        case CodeRate::R1_4: num = 1; den = 4; return true;
        case CodeRate::R1_3: num = 1; den = 3; return true;
        case CodeRate::R1_2: num = 1; den = 2; return true;
        case CodeRate::R2_3: num = 2; den = 3; return true;
        case CodeRate::R3_4: num = 3; den = 4; return true;
        case CodeRate::R5_6: num = 5; den = 6; return true;
        case CodeRate::R7_8: num = 7; den = 8; return true;
        default: return false;
    }
}

}  // namespace

size_t infoBytesFor(int codewords, CodeRate rate) {
    int num = 0, den = 1;
    if (codewords <= 0 || !rateFraction(rate, num, den)) {
        return 0;
    }
    const size_t info_bits = static_cast<size_t>(airBitsFor(codewords)) *
                             static_cast<size_t>(num) / static_cast<size_t>(den);
    const size_t bytes = info_bits / 8;
    return bytes > fec::turbo::kCrcBytes + kHeaderBytes ? bytes - fec::turbo::kCrcBytes : 0;
}

size_t payloadCapacity(int codewords, CodeRate rate) {
    const size_t info = infoBytesFor(codewords, rate);
    return info > kHeaderBytes ? std::min<size_t>(info - kHeaderBytes, 255) : 0;
}

uint32_t sessionCrcSeed(uint32_t src_hash, uint32_t dst_hash) {
    // SplitMix64 finalizer over the ordered pair: the two directions get unrelated seeds.
    uint64_t x = ((static_cast<uint64_t>(src_hash & 0xFFFFFFu)) << 24) |
                 static_cast<uint64_t>(dst_hash & 0xFFFFFFu);
    x += 0x9E3779B97F4A7C15ull;
    x = (x ^ (x >> 30)) * 0xBF58476D1CE4E5B9ull;
    x = (x ^ (x >> 27)) * 0x94D049BB133111EBull;
    x ^= x >> 31;
    const uint32_t seed = static_cast<uint32_t>(x ^ (x >> 32));
    return seed != 0 ? seed : 1u;
}

std::optional<Bytes> encode(ByteSpan v2_data_frame, int codewords, CodeRate rate) {
    const auto frame = v2::DataFrame::deserialize(v2_data_frame);
    if (!frame || frame->type != v2::FrameType::DATA) {
        return std::nullopt;
    }
    const size_t info = infoBytesFor(codewords, rate);
    if (info == 0 || frame->payload.size() > payloadCapacity(codewords, rate)) {
        return std::nullopt;
    }
    Bytes block(info, 0);
    block[0] = frame->flags;
    block[1] = static_cast<uint8_t>(frame->seq >> 8);
    block[2] = static_cast<uint8_t>(frame->seq & 0xFF);
    block[3] = static_cast<uint8_t>(frame->payload.size());
    std::copy(frame->payload.begin(), frame->payload.end(), block.begin() + kHeaderBytes);
    return fec::turbo::encode(block, airBitsFor(codewords),
                              sessionCrcSeed(frame->src_hash, frame->dst_hash));
}

DecodeResult decode(std::span<const float> air_llrs, int codewords, CodeRate rate,
                    uint32_t src_hash, uint32_t dst_hash) {
    DecodeResult out;
    const size_t info = infoBytesFor(codewords, rate);
    const size_t air_bits = static_cast<size_t>(airBitsFor(codewords));
    if (info == 0 || air_llrs.size() < air_bits) {
        return out;
    }
    fec::turbo::DecodeOptions opts;
    opts.crc_seed = sessionCrcSeed(src_hash, dst_hash);
    const auto r = fec::turbo::decode(air_llrs.first(air_bits), info, opts);
    out.iterations = r.iterations;
    out.extrinsic_agreement = r.extrinsic_agreement;
    if (!r.crc_ok || r.info_bytes.size() < kHeaderBytes) {
        return out;
    }
    const size_t length = r.info_bytes[3];
    if (length > info - kHeaderBytes) {
        return out;
    }
    v2::DataFrame frame;
    frame.type = v2::FrameType::DATA;
    frame.flags = r.info_bytes[0];
    frame.seq = static_cast<uint16_t>((r.info_bytes[1] << 8) | r.info_bytes[2]);
    frame.src_hash = src_hash & 0xFFFFFFu;
    frame.dst_hash = dst_hash & 0xFFFFFFu;
    frame.total_cw = static_cast<uint8_t>(codewords);
    frame.payload_len = static_cast<uint16_t>(length);
    frame.payload.assign(r.info_bytes.begin() + kHeaderBytes,
                         r.info_bytes.begin() + kHeaderBytes + static_cast<std::ptrdiff_t>(length));
    out.v2_frame = frame.serialize();
    out.ok = true;
    return out;
}

}  // namespace mcdpsk_frame
}  // namespace protocol
}  // namespace ultra
