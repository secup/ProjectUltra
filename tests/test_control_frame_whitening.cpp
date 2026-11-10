// OFDM control-frame whitening (INV-FRAME-WHITEN-001, protocol/frame_v2.hpp).
//
//  1. every 20-byte control frame survives whiten -> LDPC R1/4 -> decode -> unwhiten byte-exactly
//     and deserializes; the legacy unscrambled frame still decodes unchanged (receivers accept both);
//  2. unwhitenControlCodeword leaves raw-magic header codewords and data codewords alone;
//  3. the receiver's codec wrapper (ControlUnwhiteningCodec) restores a whitened frame;
//  4. the BURST_HEADER descriptor modulated with the production OFDM control profile: unscrambled,
//     a symbol's ENVELOPE peak stands far above the frame's RMS (the systematic header bits put one
//     QPSK point on most carriers); whitened, no symbol is an outlier. The envelope is the robust
//     measure: the real-sample peak is the envelope times the carrier phase at that instant, a
//     lottery (offline 12.1 dB, but 15.3 dB in the 2026-11-10 GUI recording, behind the TX filter);
//  5. the chirp-less burst's settling fill has no symbol peaking above a typical data symbol and sits at
//     the data's level (it leads every chirp-less burst, so an outlier there set the burst's level).
#include "gui/modem/frame_decoder.hpp"
#include "protocol/frame_v2.hpp"
#include "waveform/ofdm_chirp_waveform.hpp"
#include "ultra/types.hpp"

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdio>
#include <vector>

using namespace ultra;
namespace v2 = ultra::protocol::v2;

namespace {
int g_fail = 0;
void check(bool ok, const char* what) {
    std::printf("%s: %s\n", ok ? "ok  " : "FAIL", what);
    if (!ok) ++g_fail;
}

std::vector<float> softBits(const Bytes& coded) {
    std::vector<float> s;
    s.reserve(coded.size() * 8);
    for (uint8_t byte : coded) {
        for (int b = 7; b >= 0; --b) s.push_back(((byte >> b) & 1) ? -5.0f : 5.0f);
    }
    return s;
}

Bytes encodeOne(const Bytes& info) {
    auto cws = v2::encodeFrameWithLDPC(info, CodeRate::R1_4);
    Bytes coded;
    for (auto& c : cws) coded.insert(coded.end(), c.begin(), c.end());
    return coded;
}

std::vector<v2::ControlFrame> sampleFrames() {
    std::vector<v2::ControlFrame> f;
    for (uint16_t seq : {0, 1, 7, 63, 200}) {
        f.push_back(v2::ControlFrame::makeBurstHeader("ALPHA", "BRAVO", seq, 8, 8, Modulation::QPSK,
                                                      CodeRate::R1_2, 0));
        f.push_back(v2::ControlFrame::makeBurstHeader("BRAVO", "ALPHA", seq, 4, 12, Modulation::QAM16,
                                                      CodeRate::R2_3, 1));
        f.push_back(v2::ControlFrame::makeAck("ALPHA", "BRAVO", seq));
        f.push_back(v2::ControlFrame::makeGroupAck("BRAVO", "ALPHA", seq, 200));
    }
    f.push_back(v2::ControlFrame::makeKeepalive("VE2XYZ", "W1AW"));
    f.push_back(v2::ControlFrame::makeDisconnect("VE2XYZ", "W1AW"));
    f.push_back(v2::ControlFrame::makeModeChange("ALPHA", "BRAVO", 3, Modulation::QPSK, CodeRate::R1_2,
                                                 12.5f, 0.3f, 1));
    return f;
}

// The descriptor exactly as the burst transmits it on the control profile (light preamble = the two
// LTS, which reset the mixer, then the codeword), returning each body symbol's peak over the body RMS
// (dB): the real-sample peak (what the modem's peak normalization sees) and the envelope peak (what
// a real PA's PEP sees; FFT analytic signal).
struct SymbolPeaks {
    double real_max_db = 0.0;
    double env_max_db = 0.0;
};
SymbolPeaks descriptorPeaks(const Bytes& coded) {
    ModemConfig cfg = presets::balanced();
    cfg.modulation = Modulation::QPSK;
    cfg.code_rate = CodeRate::R1_4;
    OFDMChirpWaveform wf(cfg);
    const Samples pre = wf.generateDataPreamble();
    const Samples body = wf.modulate(coded);
    const size_t sym = static_cast<size_t>(wf.getSamplesPerSymbol());
    double ss = 0.0;
    for (float v : body) ss += static_cast<double>(v) * v;
    const double rms = std::sqrt(ss / std::max<size_t>(1, body.size()));
    // Envelope of the body via an FFT analytic signal (zero-padded to a power of two).
    size_t n = 1;
    while (n < 2 * body.size()) n <<= 1;
    std::vector<std::complex<double>> x(n);
    for (size_t k = 0; k < body.size(); ++k) x[k] = body[k];
    auto fft = [](std::vector<std::complex<double>>& a, bool inverse) {
        const size_t len = a.size();
        for (size_t i = 1, j = 0; i < len; ++i) {
            size_t bit = len >> 1;
            for (; j & bit; bit >>= 1) j ^= bit;
            j ^= bit;
            if (i < j) std::swap(a[i], a[j]);
        }
        for (size_t m = 2; m <= len; m <<= 1) {
            const double ang = 2.0 * M_PI / static_cast<double>(m) * (inverse ? 1.0 : -1.0);
            const std::complex<double> wm(std::cos(ang), std::sin(ang));
            for (size_t i = 0; i < len; i += m) {
                std::complex<double> w(1.0, 0.0);
                for (size_t j = 0; j < m / 2; ++j) {
                    const auto t = w * a[i + j + m / 2];
                    a[i + j + m / 2] = a[i + j] - t;
                    a[i + j] += t;
                    w *= wm;
                }
            }
        }
        if (inverse) for (auto& v : a) v /= static_cast<double>(len);
    };
    fft(x, false);
    for (size_t k = 1; k < n / 2; ++k) x[k] *= 2.0;
    for (size_t k = n / 2 + 1; k < n; ++k) x[k] = 0.0;
    fft(x, true);
    SymbolPeaks out;
    for (size_t s = 0; s + sym <= body.size(); s += sym) {
        double pk = 0.0, env = 0.0;
        for (size_t k = s; k < s + sym; ++k) {
            pk = std::max(pk, std::abs(static_cast<double>(body[k])));
            env = std::max(env, std::abs(x[k]));
        }
        out.real_max_db = std::max(out.real_max_db, 20.0 * std::log10(pk / rms));
        // Envelope peak over the envelope RMS (sqrt(2) x the real RMS).
        out.env_max_db = std::max(out.env_max_db, 20.0 * std::log10(env / (std::sqrt(2.0) * rms)));
    }
    (void)pre;
    return out;
}
}  // namespace

int main() {
    const auto frames = sampleFrames();

    // 1. Round trip, both forms.
    {
        int whitened_ok = 0, legacy_ok = 0;
        for (const auto& cf : frames) {
            const Bytes raw = cf.serialize();
            const Bytes air = v2::whitenControlFrame(raw);
            auto [ok_w, dec_w] = v2::decodeSingleCodeword(softBits(encodeOne(air)), CodeRate::R1_4);
            if (ok_w && dec_w == raw && v2::ControlFrame::deserialize(dec_w)) ++whitened_ok;
            auto [ok_l, dec_l] = v2::decodeSingleCodeword(softBits(encodeOne(raw)), CodeRate::R1_4);
            if (ok_l && dec_l == raw && v2::ControlFrame::deserialize(dec_l)) ++legacy_ok;
        }
        char msg[160];
        std::snprintf(msg, sizeof(msg), "whitened frames decode byte-exact %d/%zu; legacy frames %d/%zu",
                      whitened_ok, frames.size(), legacy_ok, frames.size());
        check(whitened_ok == static_cast<int>(frames.size()) &&
                  legacy_ok == static_cast<int>(frames.size()),
              msg);
        const Bytes air = v2::whitenControlFrame(frames[0].serialize());
        check(!v2::isHeaderCodeword(air) && !v2::isDataCodeword(air) &&
                  air[0] == v2::kWhitenedMagic0 && air[1] == v2::kWhitenedMagic1,
              "a whitened frame reads as neither a raw header nor a data codeword");
    }

    // 2. Raw-magic header codewords and data codewords pass untouched.
    {
        Bytes header = frames[2].serialize();
        const Bytes header_copy = header;
        Bytes data_cw(20, 0x00);
        data_cw[0] = v2::DATA_CW_MARKER;
        data_cw[1] = 3;
        const Bytes data_copy = data_cw;
        const bool h = v2::unwhitenControlCodeword(header);
        const bool d = v2::unwhitenControlCodeword(data_cw);
        check(!h && !d && header == header_copy && data_cw == data_copy,
              "unwhitenControlCodeword leaves raw headers and data codewords alone");
    }

    // 3. The receiver codec wrapper.
    {
        auto codec = gui::withControlUnwhitening(
            fec::CodecFactory::create(fec::CodecType::LDPC, CodeRate::R1_4));
        const Bytes raw = frames[1].serialize();
        auto [ok, dec] = codec->decode(softBits(encodeOne(v2::whitenControlFrame(raw))));
        dec.resize(std::min(dec.size(), raw.size()));
        auto ext = codec->decodeExtended(softBits(encodeOne(v2::whitenControlFrame(raw))));
        ext.data.resize(std::min(ext.data.size(), raw.size()));
        check(ok && dec == raw && ext.success && ext.data == raw,
              "ControlUnwhiteningCodec returns the original frame from a whitened codeword");
    }

    // 4. PAPR of the descriptor symbols, legacy vs whitened, for the descriptors the GUI gate sent
    //    (ALPHA -> BRAVO; 2026-11-10 recording: QPSK R1/4 cw4 flags 0x02, then QPSK R3/4 cw8 0x06).
    {
        SymbolPeaks legacy, whitened;
        for (uint16_t seq = 0; seq < 8; ++seq) {
            for (int variant = 0; variant < 2; ++variant) {
                const auto cf = variant == 0
                    ? v2::ControlFrame::makeBurstHeader("ALPHA", "BRAVO", seq, 8, 4, Modulation::QPSK,
                                                        CodeRate::R1_4, 0x02)
                    : v2::ControlFrame::makeBurstHeader("ALPHA", "BRAVO", seq, 4, 8, Modulation::QPSK,
                                                        CodeRate::R3_4, 0x06);
                const Bytes raw = cf.serialize();
                const SymbolPeaks l = descriptorPeaks(encodeOne(raw));
                const SymbolPeaks w = descriptorPeaks(encodeOne(v2::whitenControlFrame(raw)));
                legacy.real_max_db = std::max(legacy.real_max_db, l.real_max_db);
                legacy.env_max_db = std::max(legacy.env_max_db, l.env_max_db);
                whitened.real_max_db = std::max(whitened.real_max_db, w.real_max_db);
                whitened.env_max_db = std::max(whitened.env_max_db, w.env_max_db);
            }
        }
        char msg[240];
        std::snprintf(msg, sizeof(msg),
                      "BURST_HEADER symbol peak over body RMS: legacy real %.2f / envelope %.2f dB, "
                      "whitened real %.2f / envelope %.2f dB",
                      legacy.real_max_db, legacy.env_max_db, whitened.real_max_db, whitened.env_max_db);
        // Envelope: a random 59-carrier QPSK symbol stays near 10 dB over RMS (the whitened max over
        // these 112 symbols); the unscrambled header symbol sat ~3 dB above that.
        check(legacy.env_max_db > 12.5 && whitened.env_max_db < 11.0 &&
                  legacy.env_max_db - whitened.env_max_db > 2.0,
              msg);
    }

    // 5. Settling fill: every symbol's sample peak within the bound, level equal to the data's.
    {
        double worst_db = 0.0, level_err_db = 0.0;
        for (Modulation mod : {Modulation::QPSK, Modulation::QAM8, Modulation::QAM16}) {
            ModemConfig cfg = presets::balanced();
            cfg.modulation = mod;
            cfg.code_rate = CodeRate::R1_2;
            OFDMChirpWaveform wf(cfg);
            const Samples fill = wf.generateSettlingSymbols(6);
            const size_t sym = static_cast<size_t>(wf.getSamplesPerSymbol());
            double ss = 0.0;
            for (float v : fill) ss += static_cast<double>(v) * v;
            const double fill_rms = std::sqrt(ss / std::max<size_t>(1, fill.size()));
            for (size_t s0 = 0; s0 + sym <= fill.size(); s0 += sym) {
                double pk = 0.0, sss = 0.0;
                for (size_t k = s0; k < s0 + sym; ++k) {
                    pk = std::max(pk, std::abs(static_cast<double>(fill[k])));
                    sss += static_cast<double>(fill[k]) * fill[k];
                }
                worst_db = std::max(worst_db, 20.0 * std::log10(pk / std::sqrt(sss / sym)));
            }
            // Data level reference: random payload on the same profile.
            Bytes payload(600);
            uint32_t r = 12345u;
            for (auto& b : payload) { r ^= r << 13; r ^= r >> 17; r ^= r << 5; b = static_cast<uint8_t>(r); }
            const Samples data = wf.modulate(payload);
            double ds = 0.0;
            for (float v : data) ds += static_cast<double>(v) * v;
            const double data_rms = std::sqrt(ds / std::max<size_t>(1, data.size()));
            level_err_db = std::max(level_err_db, std::abs(20.0 * std::log10(fill_rms / data_rms)));
        }
        char msg[200];
        std::snprintf(msg, sizeof(msg),
                      "settling fill: worst symbol peak %.2f dB over its RMS (bound 10), level within %.2f dB of data",
                      worst_db, level_err_db);
        check(worst_db <= 10.0 + 1e-6 && level_err_db < 1.0, msg);
    }

    if (g_fail == 0) {
        std::printf("PASS: control-frame whitening\n");
        return 0;
    }
    std::printf("%d FAILURE(S)\n", g_fail);
    return 1;
}
