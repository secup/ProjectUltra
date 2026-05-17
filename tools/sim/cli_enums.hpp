#pragma once

#include "ota_channel_core/models.hpp"
#include "protocol/waveform_selection.hpp"
#include "ultra/types.hpp"

#include <algorithm>
#include <cctype>
#include <optional>
#include <stdexcept>
#include <string>

using ChannelType = ultra::ota_channel_core::ChannelType;

namespace ultra::tools::cli {

enum class AllowAuto {
    No,
    Yes
};

enum class AllowAwgn {
    No,
    Yes
};

enum class AllowExperimentalModulation {
    No,
    Yes
};

enum class BareOFDMMode {
    Reject,
    Chirp,
    Cox
};

inline std::string normalizedToken(std::string value) {
    std::replace(value.begin(), value.end(), '-', '_');
    for (char& c : value) {
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    }
    return value;
}

inline const char* codeRateChoices(AllowAuto allow_auto = AllowAuto::No) {
    return allow_auto == AllowAuto::Yes
        ? "auto, r1_4, r1_2, r2_3, r3_4"
        : "r1_4, r1_2, r2_3, r3_4";
}

inline const char* modulationChoices(
    AllowAuto allow_auto = AllowAuto::No,
    AllowExperimentalModulation allow_experimental = AllowExperimentalModulation::Yes) {
    if (allow_experimental == AllowExperimentalModulation::No) {
        return allow_auto == AllowAuto::Yes ? "auto, dqpsk" : "dqpsk";
    }
    return allow_auto == AllowAuto::Yes
        ? "auto, dqpsk, d8psk, dbpsk, qpsk, bpsk, qam16, qam32, qam64"
        : "dqpsk, d8psk, dbpsk, qpsk, bpsk, qam16, qam32, qam64";
}

inline const char* waveformChoices() {
    return "mc_dpsk, dpsk, ofdm_chirp, chirp, ofdm_cox, cox, ofdm_narrow, narrow";
}

inline const char* channelChoices(AllowAwgn allow_awgn = AllowAwgn::Yes) {
    return allow_awgn == AllowAwgn::Yes
        ? "passthrough, awgn, good, moderate, poor, flutter"
        : "good, moderate, poor, flutter";
}

inline std::optional<CodeRate> parseCodeRate(const std::string& value,
                                             AllowAuto allow_auto = AllowAuto::No) {
    const std::string v = normalizedToken(value);
    if (allow_auto == AllowAuto::Yes && v == "auto") return CodeRate::AUTO;
    if (v == "r1_4" || v == "1/4") return CodeRate::R1_4;
    if (v == "r1_2" || v == "1/2") return CodeRate::R1_2;
    if (v == "r2_3" || v == "2/3") return CodeRate::R2_3;
    if (v == "r3_4" || v == "3/4") return CodeRate::R3_4;
    return std::nullopt;
}

inline CodeRate requireCodeRate(const std::string& value,
                                AllowAuto allow_auto = AllowAuto::No) {
    if (auto parsed = parseCodeRate(value, allow_auto)) return *parsed;
    throw std::invalid_argument("Unknown code rate: " + value +
                                " (use " + codeRateChoices(allow_auto) + ")");
}

inline std::optional<Modulation> parseModulation(const std::string& value,
                                                 AllowAuto allow_auto = AllowAuto::No,
                                                 AllowExperimentalModulation allow_experimental =
                                                     AllowExperimentalModulation::Yes) {
    const std::string v = normalizedToken(value);
    if (allow_auto == AllowAuto::Yes && v == "auto") return Modulation::AUTO;
    if (v == "dqpsk") return Modulation::DQPSK;
    if (allow_experimental == AllowExperimentalModulation::No) return std::nullopt;
    if (v == "d8psk") return Modulation::D8PSK;
    if (v == "dbpsk") return Modulation::DBPSK;
    if (v == "qpsk") return Modulation::QPSK;
    if (v == "bpsk") return Modulation::BPSK;
    if (v == "qam16") return Modulation::QAM16;
    if (v == "qam32") return Modulation::QAM32;
    if (v == "qam64") return Modulation::QAM64;
    return std::nullopt;
}

inline Modulation requireModulation(const std::string& value,
                                    AllowAuto allow_auto = AllowAuto::No,
                                    AllowExperimentalModulation allow_experimental =
                                        AllowExperimentalModulation::Yes) {
    if (auto parsed = parseModulation(value, allow_auto, allow_experimental)) return *parsed;
    throw std::invalid_argument("Unknown modulation: " + value +
                                " (use " + modulationChoices(allow_auto, allow_experimental) + ")");
}

inline bool isExpertOnlyModulation(Modulation mod) {
    return mod != Modulation::AUTO && mod != Modulation::DQPSK;
}

inline std::optional<protocol::WaveformMode> parseWaveformMode(
    const std::string& value,
    BareOFDMMode bare_ofdm = BareOFDMMode::Reject,
    AllowAuto allow_auto = AllowAuto::No) {
    const std::string v = normalizedToken(value);
    if (allow_auto == AllowAuto::Yes && v == "auto") return protocol::WaveformMode::AUTO;
    if (v == "mc_dpsk" || v == "mcdpsk" || v == "dpsk") return protocol::WaveformMode::MC_DPSK;
    if (v == "ofdm_chirp" || v == "chirp") return protocol::WaveformMode::OFDM_CHIRP;
    if (v == "ofdm_cox" || v == "cox") return protocol::WaveformMode::OFDM_COX;
    if (v == "ofdm_narrow" || v == "narrow") return protocol::WaveformMode::OFDM_NARROW;
    if (v == "ofdm") {
        if (bare_ofdm == BareOFDMMode::Chirp) return protocol::WaveformMode::OFDM_CHIRP;
        if (bare_ofdm == BareOFDMMode::Cox) return protocol::WaveformMode::OFDM_COX;
    }
    return std::nullopt;
}

inline protocol::WaveformMode requireWaveformMode(
    const std::string& value,
    BareOFDMMode bare_ofdm = BareOFDMMode::Reject,
    AllowAuto allow_auto = AllowAuto::No) {
    if (auto parsed = parseWaveformMode(value, bare_ofdm, allow_auto)) return *parsed;
    throw std::invalid_argument("Unknown waveform: " + value +
                                " (use " + waveformChoices() + ")");
}

inline std::optional<ChannelType> parseChannelType(
    const std::string& value,
    AllowAwgn allow_awgn = AllowAwgn::Yes) {
    const std::string v = normalizedToken(value);
    if (allow_awgn == AllowAwgn::Yes && (v == "passthrough" || v == "null")) {
        return ChannelType::PASSTHROUGH;
    }
    if (allow_awgn == AllowAwgn::Yes && v == "awgn") return ChannelType::AWGN;
    if (v == "good") return ChannelType::GOOD;
    if (v == "moderate") return ChannelType::MODERATE;
    if (v == "poor") return ChannelType::POOR;
    if (v == "flutter") return ChannelType::FLUTTER;
    return std::nullopt;
}

inline ChannelType requireChannelType(const std::string& value,
                                      AllowAwgn allow_awgn = AllowAwgn::Yes) {
    if (auto parsed = parseChannelType(value, allow_awgn)) return *parsed;
    throw std::invalid_argument("Unknown channel: " + value +
                                " (use " + channelChoices(allow_awgn) + ")");
}

}  // namespace ultra::tools::cli
