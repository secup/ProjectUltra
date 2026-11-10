#pragma once

#include "ultra/fec.hpp"             // ChannelInterleaver
#include "fec/codec_factory.hpp"     // fec::CodecPtr (ICodec)
#include "protocol/frame_v2.hpp"     // unwhitenControlCodeword (INV-FRAME-WHITEN-001)

#include <memory>
#include <utility>

namespace ultra {
namespace gui {

// The receiver's LDPC codec with whitened control frames restored on the way out
// (INV-FRAME-WHITEN-001). LDPC decoding does not depend on the whitening, so every decode site
// decodes as before; a codeword that starts with the whitened magic is XORed back here, before any
// header peek, so no caller sees a whitened frame. Everything else is forwarded unchanged.
class ControlUnwhiteningCodec final : public fec::ICodec {
public:
    explicit ControlUnwhiteningCodec(fec::CodecPtr inner) : inner_(std::move(inner)) {}

    std::string getName() const override { return inner_->getName(); }
    void setRate(CodeRate rate) override { inner_->setRate(rate); }
    CodeRate getRate() const override { return inner_->getRate(); }
    void setMaxIterations(int iterations) override { inner_->setMaxIterations(iterations); }
    int getMaxIterations() const override { return inner_->getMaxIterations(); }
    Bytes encode(const Bytes& data) override { return inner_->encode(data); }

    std::pair<bool, Bytes> decode(const std::vector<float>& soft_bits) override {
        auto result = inner_->decode(soft_bits);
        if (result.first) protocol::v2::unwhitenControlCodeword(result.second);
        return result;
    }
    fec::DecodeResult decodeExtended(const std::vector<float>& soft_bits) override {
        auto result = inner_->decodeExtended(soft_bits);
        if (result.success) protocol::v2::unwhitenControlCodeword(result.data);
        return result;
    }

    size_t getCodewordBits() const override { return inner_->getCodewordBits(); }
    size_t getInfoBits() const override { return inner_->getInfoBits(); }
    size_t getParityBits() const override { return inner_->getParityBits(); }
    size_t getCodewordBytes() const override { return inner_->getCodewordBytes(); }
    size_t getDataBytes() const override { return inner_->getDataBytes(); }
    float getEffectiveRate() const override { return inner_->getEffectiveRate(); }

private:
    fec::CodecPtr inner_;
};

inline fec::CodecPtr withControlUnwhitening(fec::CodecPtr inner) {
    if (!inner) return inner;
    return std::make_unique<ControlUnwhiteningCodec>(std::move(inner));
}

// FrameDecoder — the "soft bits -> frame" FEC decode stage, carved out of the StreamingDecoder
// god-class [component-decomposition-plan].
//
// Concern split: demodulation ("samples -> soft bits": the CFO pre-correction + the waveform's
// baseband/FFT/equalize) is FrameDemodulator's job; FEC decode ("soft bits -> frame": deinterleave +
// LDPC, the rate/interleave strategy) is a distinct concern and lives here. FrameDecoder owns the
// decode PRIMITIVES — the rate-switchable LDPC codec + the channel interleaver — and grows to own the
// decode ORCHESTRATION (decodeFrame / decodeMCDPSKFrame), driven by a FrameDecodeContext the decoder
// passes in (so FrameDecoder stays decoupled from the decoder's config).
//
// §7 C-FDec-1 (relocation): these primitives are transitional-public — the decode methods still live
// on the decoder and drive them as frame_decoder_.codec_/interleaver_, and the mode/rate machinery
// reconfigures them in place. They privatize behind FrameDecoder methods once decodeFrame moves in.
// codec_ is always assigned through withControlUnwhitening().
class FrameDecoder {
public:
    fec::CodecPtr codec_;
    std::unique_ptr<ChannelInterleaver> interleaver_;
};

}  // namespace gui
}  // namespace ultra
