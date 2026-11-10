#include "io/wav_io.hpp"

#include "ultra/dsp.hpp"

#include <algorithm>
#include <cstring>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace ultra::tools::io {
namespace {

constexpr uint16_t kFormatPcm = 1;
constexpr uint16_t kFormatFloat = 3;
constexpr uint16_t kFormatExtensible = 0xFFFE;

uint16_t readLe16(const uint8_t* p) {
    return static_cast<uint16_t>(p[0]) |
           (static_cast<uint16_t>(p[1]) << 8);
}

uint32_t readLe32(const uint8_t* p) {
    return static_cast<uint32_t>(p[0]) |
           (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16) |
           (static_cast<uint32_t>(p[3]) << 24);
}

void writeLe16(std::ostream& out, uint16_t value) {
    const uint8_t bytes[2] = {
        static_cast<uint8_t>(value & 0xFFu),
        static_cast<uint8_t>((value >> 8) & 0xFFu),
    };
    out.write(reinterpret_cast<const char*>(bytes), sizeof(bytes));
}

void writeLe32(std::ostream& out, uint32_t value) {
    const uint8_t bytes[4] = {
        static_cast<uint8_t>(value & 0xFFu),
        static_cast<uint8_t>((value >> 8) & 0xFFu),
        static_cast<uint8_t>((value >> 16) & 0xFFu),
        static_cast<uint8_t>((value >> 24) & 0xFFu),
    };
    out.write(reinterpret_cast<const char*>(bytes), sizeof(bytes));
}

bool readExact(std::istream& in, void* dst, size_t len) {
    in.read(static_cast<char*>(dst), static_cast<std::streamsize>(len));
    return static_cast<size_t>(in.gcount()) == len;
}

uint16_t extensibleSubformatTag(const std::vector<uint8_t>& fmt) {
    if (fmt.size() < 40) return 0;
    return readLe16(fmt.data() + 24);
}

std::vector<float> decodeWavPayload(const std::vector<uint8_t>& data,
                                    uint16_t format,
                                    uint16_t channels,
                                    uint16_t bits,
                                    uint16_t block_align) {
    if (channels == 0 || channels > 2) {
        throw std::runtime_error("Only mono/stereo WAV files are supported");
    }

    const bool is_pcm = (format == kFormatPcm);
    const bool is_float = (format == kFormatFloat);
    const uint16_t bytes_per_sample = static_cast<uint16_t>((bits + 7) / 8);
    if (!((is_pcm && (bits == 16 || bits == 24)) || (is_float && bits == 32))) {
        std::ostringstream oss;
        oss << "Unsupported WAV format=" << format << " bits=" << bits;
        throw std::runtime_error(oss.str());
    }

    const uint16_t expected_align = static_cast<uint16_t>(channels * bytes_per_sample);
    if (block_align == 0) block_align = expected_align;
    if (block_align < expected_align) {
        throw std::runtime_error("Invalid WAV block alignment");
    }

    const size_t frames = data.size() / block_align;
    std::vector<float> mono;
    mono.reserve(frames);

    for (size_t frame = 0; frame < frames; ++frame) {
        const uint8_t* base = data.data() + frame * block_align;
        double sum = 0.0;
        for (uint16_t ch = 0; ch < channels; ++ch) {
            const uint8_t* p = base + ch * bytes_per_sample;
            float sample = 0.0f;
            if (is_float) {
                const uint32_t raw = readLe32(p);
                std::memcpy(&sample, &raw, sizeof(sample));
            } else if (bits == 16) {
                const int16_t v = static_cast<int16_t>(readLe16(p));
                sample = static_cast<float>(v) / 32768.0f;
            } else {
                int32_t v = static_cast<int32_t>(p[0]) |
                            (static_cast<int32_t>(p[1]) << 8) |
                            (static_cast<int32_t>(p[2]) << 16);
                if (v & 0x00800000) v |= static_cast<int32_t>(0xFF000000);
                sample = static_cast<float>(v) / 8388608.0f;
            }
            sum += sample;
        }
        mono.push_back(static_cast<float>(sum / static_cast<double>(channels)));
    }

    return mono;
}

}  // namespace

LoadedWav loadWavMono48k(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        throw std::runtime_error("Failed to open WAV: " + path);
    }

    uint8_t riff_header[12] = {};
    if (!readExact(in, riff_header, sizeof(riff_header)) ||
        std::memcmp(riff_header, "RIFF", 4) != 0 ||
        std::memcmp(riff_header + 8, "WAVE", 4) != 0) {
        throw std::runtime_error("Not a RIFF/WAVE file: " + path);
    }

    std::vector<uint8_t> fmt_chunk;
    std::vector<uint8_t> data_chunk;

    while (in) {
        uint8_t chunk_header[8] = {};
        if (!readExact(in, chunk_header, sizeof(chunk_header))) break;
        const uint32_t chunk_size = readLe32(chunk_header + 4);
        std::vector<uint8_t> chunk(chunk_size);
        if (chunk_size > 0 && !readExact(in, chunk.data(), chunk_size)) {
            throw std::runtime_error("Truncated WAV chunk");
        }
        if (chunk_size & 1u) {
            in.seekg(1, std::ios::cur);
        }

        if (std::memcmp(chunk_header, "fmt ", 4) == 0) {
            fmt_chunk = std::move(chunk);
        } else if (std::memcmp(chunk_header, "data", 4) == 0) {
            data_chunk = std::move(chunk);
        }
    }

    if (fmt_chunk.size() < 16 || data_chunk.empty()) {
        throw std::runtime_error("WAV missing fmt or data chunk");
    }

    uint16_t format = readLe16(fmt_chunk.data());
    const uint16_t channels = readLe16(fmt_chunk.data() + 2);
    const uint32_t sample_rate = readLe32(fmt_chunk.data() + 4);
    uint16_t block_align = readLe16(fmt_chunk.data() + 12);
    uint16_t bits = readLe16(fmt_chunk.data() + 14);
    if (sample_rate == 0) {
        throw std::runtime_error("Invalid WAV sample rate");
    }

    if (format == kFormatExtensible) {
        const uint16_t subformat = extensibleSubformatTag(fmt_chunk);
        if (subformat == kFormatPcm || subformat == kFormatFloat) {
            format = subformat;
        }
        if (fmt_chunk.size() >= 20) {
            const uint16_t valid_bits = readLe16(fmt_chunk.data() + 18);
            if (valid_bits == 16 || valid_bits == 24 || valid_bits == 32) {
                bits = valid_bits;
            }
        }
    }

    LoadedWav wav;
    wav.path = path;
    wav.source_rate = sample_rate;
    wav.source_channels = channels;
    wav.source_bits = bits;
    wav.source_format = format;

    auto mono = decodeWavPayload(data_chunk, format, channels, bits, block_align);
    if (sample_rate == kWavTargetSampleRate) {
        wav.samples_48k = std::move(mono);
    } else {
        ultra::Resampler resampler(sample_rate, kWavTargetSampleRate);
        ultra::SampleSpan span(mono.data(), mono.size());
        wav.samples_48k = resampler.process(span);
        const ultra::Samples tail = resampler.flush();
        wav.samples_48k.insert(wav.samples_48k.end(), tail.begin(), tail.end());
    }

    return wav;
}

bool writeWavF32Mono(const std::string& path,
                     const std::vector<float>& samples,
                     uint32_t sample_rate) {
    std::ofstream out(path, std::ios::binary);
    if (!out) return false;

    const uint32_t data_bytes = static_cast<uint32_t>(samples.size() * sizeof(float));
    const uint16_t audio_format = kFormatFloat;
    const uint16_t channels = 1;
    const uint16_t bits = 32;
    const uint16_t block_align = static_cast<uint16_t>(channels * bits / 8);
    const uint32_t byte_rate = sample_rate * block_align;
    const uint32_t riff_size = 36 + data_bytes;

    out.write("RIFF", 4);
    writeLe32(out, riff_size);
    out.write("WAVE", 4);
    out.write("fmt ", 4);
    writeLe32(out, 16);
    writeLe16(out, audio_format);
    writeLe16(out, channels);
    writeLe32(out, sample_rate);
    writeLe32(out, byte_rate);
    writeLe16(out, block_align);
    writeLe16(out, bits);
    out.write("data", 4);
    writeLe32(out, data_bytes);
    if (!samples.empty()) {
        out.write(reinterpret_cast<const char*>(samples.data()),
                  static_cast<std::streamsize>(data_bytes));
    }
    return out.good();
}

bool writeWavPCM16Mono(const std::string& path,
                       const std::vector<float>& samples,
                       uint32_t sample_rate) {
    std::ofstream out(path, std::ios::binary);
    if (!out) return false;

    const uint32_t data_bytes = static_cast<uint32_t>(samples.size() * sizeof(int16_t));
    const uint16_t audio_format = kFormatPcm;
    const uint16_t channels = 1;
    const uint16_t bits = 16;
    const uint16_t block_align = static_cast<uint16_t>(channels * bits / 8);
    const uint32_t byte_rate = sample_rate * block_align;
    const uint32_t riff_size = 36 + data_bytes;

    out.write("RIFF", 4);
    writeLe32(out, riff_size);
    out.write("WAVE", 4);
    out.write("fmt ", 4);
    writeLe32(out, 16);
    writeLe16(out, audio_format);
    writeLe16(out, channels);
    writeLe32(out, sample_rate);
    writeLe32(out, byte_rate);
    writeLe16(out, block_align);
    writeLe16(out, bits);
    out.write("data", 4);
    writeLe32(out, data_bytes);

    for (float sample : samples) {
        const float clamped = std::clamp(sample, -1.0f, 1.0f);
        const int32_t scaled = static_cast<int32_t>((clamped >= 0.0f)
            ? clamped * 32767.0f + 0.5f
            : clamped * 32768.0f - 0.5f);
        const int32_t pcm = std::clamp(scaled, -32768, 32767);
        writeLe16(out, static_cast<uint16_t>(static_cast<int16_t>(pcm)));
    }
    return out.good();
}

}  // namespace ultra::tools::io
