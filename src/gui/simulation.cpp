#include "simulation.hpp"
#include <cmath>
#include <cstring>

namespace ultra {
namespace gui {

SimulationEngine::SimulationEngine() {
    encoder_ = std::make_unique<LDPCEncoder>(CodeRate::R1_2);
    decoder_ = std::make_unique<LDPCDecoder>(CodeRate::R1_2);
}

SimulationEngine::~SimulationEngine() = default;

void SimulationEngine::setChannelPreset(ChannelPreset preset) {
    preset_ = preset;
    switch (preset) {
        case ChannelPreset::AWGN_ONLY:
            snr_db_ = 25.0f;  // Clean channel - good for testing
            multipath_delay_ms_ = 0.0f;
            doppler_hz_ = 0.0f;
            break;
        case ChannelPreset::GOOD:
            snr_db_ = 25.0f;  // Excellent HF conditions
            multipath_delay_ms_ = 0.5f;
            doppler_hz_ = 0.2f;
            break;
        case ChannelPreset::MODERATE:
            snr_db_ = 20.0f;  // Typical good HF (QAM64 needs ~20dB)
            multipath_delay_ms_ = 1.0f;
            doppler_hz_ = 0.5f;
            break;
        case ChannelPreset::POOR:
            snr_db_ = 12.0f;  // Challenging HF (use QPSK/16-QAM)
            multipath_delay_ms_ = 2.0f;
            doppler_hz_ = 1.0f;
            break;
        default:
            break;
    }
}

void SimulationEngine::setSNR(float snr_db) {
    snr_db_ = snr_db;
    preset_ = ChannelPreset::CUSTOM;
}

void SimulationEngine::setMultipathDelay(float ms) {
    multipath_delay_ms_ = ms;
    preset_ = ChannelPreset::CUSTOM;
}

void SimulationEngine::setDopplerSpread(float hz) {
    doppler_hz_ = hz;
    preset_ = ChannelPreset::CUSTOM;
}

std::vector<std::complex<float>> SimulationEngine::extractSymbols(const Bytes& encoded,
                                                                   Modulation mod) {
    std::vector<std::complex<float>> symbols;
    int bits_per_symbol = static_cast<int>(mod);

    size_t bit_idx = 0;
    while (bit_idx + bits_per_symbol <= encoded.size() * 8) {
        uint32_t bits = 0;
        for (int b = 0; b < bits_per_symbol; ++b) {
            size_t byte_idx = (bit_idx + b) / 8;
            int bit_pos = 7 - ((bit_idx + b) % 8);
            uint8_t bit = (encoded[byte_idx] >> bit_pos) & 1;
            bits = (bits << 1) | bit;
        }

        // Map bits to constellation point (simplified - matches modulator)
        float x = 0, y = 0;
        switch (mod) {
            case Modulation::BPSK:
                x = (bits & 1) ? 1.0f : -1.0f;
                y = 0;
                break;
            case Modulation::QPSK: {
                static const float s = 0.7071f;
                x = (bits & 2) ? s : -s;
                y = (bits & 1) ? s : -s;
                break;
            }
            case Modulation::QAM16: {
                static const float levels[] = {-3, -1, 3, 1};
                static const float scale = 0.3162f;
                x = levels[(bits >> 2) & 3] * scale;
                y = levels[bits & 3] * scale;
                break;
            }
            case Modulation::QAM64: {
                static const float levels[] = {-7, -5, -1, -3, 7, 5, 1, 3};
                static const float scale = 0.1543f;
                x = levels[(bits >> 3) & 7] * scale;
                y = levels[bits & 7] * scale;
                break;
            }
            case Modulation::QAM256: {
                static const float levels[] = {-15,-13,-9,-11,-1,-3,-7,-5,15,13,9,11,1,3,7,5};
                static const float scale = 0.0645f;
                x = levels[(bits >> 4) & 15] * scale;
                y = levels[bits & 15] * scale;
                break;
            }
            default:
                x = (bits & 1) ? 0.7f : -0.7f;
                y = (bits & 2) ? 0.7f : -0.7f;
        }
        symbols.emplace_back(x, y);
        bit_idx += bits_per_symbol;
    }
    return symbols;
}

SimulationResult SimulationEngine::runFrame(const ModemConfig& config) {
    SimulationResult result;
    result.snr_db = snr_db_;

    // Update encoder/decoder rate
    encoder_->setRate(config.code_rate);
    decoder_->setRate(config.code_rate);

    // Generate random test data (40 bytes = 320 bits)
    result.tx_data.resize(40);
    for (auto& b : result.tx_data) {
        b = rng_() & 0xFF;
    }

    // Encode with LDPC
    Bytes encoded = encoder_->encode(result.tx_data);

    // Extract TX symbols for constellation display
    result.tx_symbols = extractSymbols(encoded, config.modulation);

    // Add channel noise to symbols (simplified - direct AWGN on symbols)
    float noise_std = std::pow(10.0f, -snr_db_ / 20.0f);
    std::normal_distribution<float> noise(0.0f, noise_std);

    result.rx_symbols.reserve(result.tx_symbols.size());
    std::vector<float> soft_bits;

    // Channel model: AWGN + multipath effects (amplitude fading + extra noise)
    // Note: Phase rotation requires equalization which this simplified sim doesn't have
    // So we model multipath as additional noise and amplitude variation instead
    float multipath_noise_factor = 1.0f + multipath_delay_ms_ * 0.02f;  // Small ISI penalty

    for (const auto& sym : result.tx_symbols) {
        // Add AWGN (scaled by multipath factor for ISI modeling)
        float rx_i = sym.real() + noise(rng_) * multipath_noise_factor;
        float rx_q = sym.imag() + noise(rng_) * multipath_noise_factor;

        // Add Doppler-induced amplitude fading (gentle variation)
        if (doppler_hz_ > 0) {
            // Small amplitude variation (95% to 105%)
            float fade = 0.95f + 0.1f * ((rng_() % 1000) / 1000.0f);
            rx_i *= fade;
            rx_q *= fade;
        }

        result.rx_symbols.emplace_back(rx_i, rx_q);

        // Generate soft bits (LLRs) from received symbol
        // LDPC decoder convention: negative LLR = bit 1, positive LLR = bit 0
        // Constellation: bit 1 maps to positive symbol values
        // So LLR ∝ -rx (NEGATED: positive rx → negative LLR → bit 1)
        float noise_var = noise_std * noise_std;
        if (noise_var < 1e-6f) noise_var = 1e-6f;

        switch (config.modulation) {
            case Modulation::BPSK: {
                float llr = -2.0f * rx_i / noise_var;
                soft_bits.push_back(llr);
                break;
            }
            case Modulation::QPSK: {
                float scale = -2.0f * 0.7071f / noise_var;
                soft_bits.push_back(rx_i * scale);
                soft_bits.push_back(rx_q * scale);
                break;
            }
            case Modulation::QAM16: {
                float d = 0.6325f;
                float scale = -2.0f / noise_var;
                soft_bits.push_back(scale * rx_i);
                soft_bits.push_back(scale * (d - std::abs(rx_i)));
                soft_bits.push_back(scale * rx_q);
                soft_bits.push_back(scale * (d - std::abs(rx_q)));
                break;
            }
            case Modulation::QAM64: {
                float d2 = 0.3086f;
                float d4 = 0.6172f;
                float scale = -2.0f / noise_var;
                soft_bits.push_back(scale * rx_i);
                soft_bits.push_back(scale * (d4 - std::abs(rx_i)));
                soft_bits.push_back(scale * (d2 - std::abs(std::abs(rx_i) - d4)));
                soft_bits.push_back(scale * rx_q);
                soft_bits.push_back(scale * (d4 - std::abs(rx_q)));
                soft_bits.push_back(scale * (d2 - std::abs(std::abs(rx_q) - d4)));
                break;
            }
            case Modulation::QAM256: {
                float d2 = 0.1291f;
                float d4 = 0.2582f;
                float d8 = 0.5164f;
                float scale = -2.0f / noise_var;
                soft_bits.push_back(scale * rx_i);
                soft_bits.push_back(scale * (d8 - std::abs(rx_i)));
                soft_bits.push_back(scale * (d4 - std::abs(std::abs(rx_i) - d8)));
                soft_bits.push_back(scale * (d2 - std::abs(std::abs(std::abs(rx_i) - d8) - d4)));
                soft_bits.push_back(scale * rx_q);
                soft_bits.push_back(scale * (d8 - std::abs(rx_q)));
                soft_bits.push_back(scale * (d4 - std::abs(std::abs(rx_q) - d8)));
                soft_bits.push_back(scale * (d2 - std::abs(std::abs(std::abs(rx_q) - d8) - d4)));
                break;
            }
            default: {
                float scale = -2.0f * 0.7071f / noise_var;
                soft_bits.push_back(rx_i * scale);
                soft_bits.push_back(rx_q * scale);
            }
        }
    }

    // Decode with LDPC
    result.rx_data = decoder_->decodeSoft(soft_bits);
    result.decode_success = decoder_->lastDecodeSuccess();
    result.ldpc_iterations = decoder_->lastIterations();

    // Count bit errors
    result.total_bits = result.tx_data.size() * 8;
    result.bit_errors = 0;
    for (size_t i = 0; i < result.tx_data.size() && i < result.rx_data.size(); ++i) {
        uint8_t diff = result.tx_data[i] ^ result.rx_data[i];
        while (diff) {
            result.bit_errors += (diff & 1);
            diff >>= 1;
        }
    }
    result.ber = static_cast<float>(result.bit_errors) / result.total_bits;

    // Calculate throughput
    int bits_per_carrier = static_cast<int>(config.modulation);
    float code_rate = 0.5f;
    switch (config.code_rate) {
        case CodeRate::R1_4: code_rate = 0.25f; break;
        case CodeRate::R1_3: code_rate = 0.333f; break;
        case CodeRate::R1_2: code_rate = 0.5f; break;
        case CodeRate::R2_3: code_rate = 0.667f; break;
        case CodeRate::R3_4: code_rate = 0.75f; break;
        case CodeRate::R5_6: code_rate = 0.833f; break;
        case CodeRate::R7_8: code_rate = 0.875f; break;
    }
    float symbol_rate = config.getSymbolRate();
    result.throughput_bps = config.getDataCarriers() * bits_per_carrier * code_rate * symbol_rate;

    // Update statistics
    total_frames_++;
    if (result.decode_success && result.bit_errors == 0) {
        success_frames_++;
    }
    total_bit_errors_ += result.bit_errors;
    total_bits_ += result.total_bits;

    return result;
}

float SimulationEngine::getAverageBER() const {
    if (total_bits_ == 0) return 0;
    return static_cast<float>(total_bit_errors_) / total_bits_;
}

void SimulationEngine::resetStats() {
    total_frames_ = 0;
    success_frames_ = 0;
    total_bit_errors_ = 0;
    total_bits_ = 0;
}

} // namespace gui
} // namespace ultra
