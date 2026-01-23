// Debug test - ModemEngine TX, direct OFDMDemodulator RX
// This isolates whether the issue is TX or RX side

#include "gui/modem/modem_engine.hpp"
#include "ultra/types.hpp"
#include "ultra/ofdm.hpp"
#include "ultra/fec.hpp"
#include "ultra/logging.hpp"
#include <iostream>
#include <random>
#include <cmath>

using namespace ultra;
using namespace ultra::gui;

int main() {
    setLogLevel(LogLevel::DEBUG);

    printf("=== ModemEngine TX vs Direct OFDMDemodulator RX ===\n\n");

    // Create ModemEngine for TX
    ModemEngine tx_modem;
    tx_modem.setLogPrefix("TX");
    tx_modem.setConnected(true);
    tx_modem.setHandshakeComplete(true);
    tx_modem.setWaveformMode(protocol::WaveformMode::OFDM);
    tx_modem.setDataMode(Modulation::DQPSK, CodeRate::R1_2);

    // Generate test payload
    std::string msg = "HELLO_TEST";
    Bytes payload(msg.begin(), msg.end());
    printf("Payload: \"%s\" (%zu bytes)\n", msg.c_str(), payload.size());

    // TX via ModemEngine
    auto tx_audio = tx_modem.transmit(payload);
    printf("TX audio: %zu samples (%.1f ms)\n\n", tx_audio.size(), tx_audio.size() / 48.0f);

    // Find where signal starts (skip lead-in silence)
    size_t signal_start = 0;
    for (size_t i = 0; i < tx_audio.size(); i++) {
        if (std::abs(tx_audio[i]) > 0.01f) {
            signal_start = i;
            break;
        }
    }
    printf("Signal starts at sample %zu (lead-in silence: %.1f ms)\n",
           signal_start, signal_start / 48.0f);

    // Create direct OFDMDemodulator for RX (matching TX config)
    ModemConfig rx_config;
    rx_config.sample_rate = 48000;
    rx_config.center_freq = 1500;
    rx_config.fft_size = 512;
    rx_config.num_carriers = 30;
    rx_config.pilot_spacing = 2;
    rx_config.modulation = Modulation::DQPSK;
    rx_config.code_rate = CodeRate::R1_2;
    rx_config.use_pilots = false;  // DQPSK doesn't need pilots

    OFDMDemodulator demod(rx_config);
    LDPCDecoder decoder(CodeRate::R1_2);

    printf("\n=== Direct Demod (skip lead-in, process signal only) ===\n");

    // Process audio starting from signal (skip lead-in)
    size_t chunk_size = 960;
    for (size_t i = signal_start; i < tx_audio.size(); i += chunk_size) {
        size_t len = std::min(chunk_size, tx_audio.size() - i);
        SampleSpan span(tx_audio.data() + i, len);
        demod.process(span);
    }

    auto soft = demod.getSoftBits();
    printf("Got %zu soft bits\n", soft.size());

    if (soft.size() >= 648) {
        std::span<const float> llrs(soft.data(), 648);
        Bytes decoded = decoder.decodeSoft(llrs);

        if (decoder.lastDecodeSuccess()) {
            printf("LDPC SUCCESS! Decoded: \"");
            for (size_t i = 0; i < std::min((size_t)10, decoded.size()); i++) {
                char c = decoded[i];
                printf("%c", c >= 32 && c < 127 ? c : '?');
            }
            printf("\"\n");
        } else {
            printf("LDPC FAILED (iters=%d)\n", decoder.lastIterations());
        }
    }

    // Now test full audio including lead-in
    printf("\n=== Direct Demod (process full audio including lead-in) ===\n");

    OFDMDemodulator demod2(rx_config);
    for (size_t i = 0; i < tx_audio.size(); i += chunk_size) {
        size_t len = std::min(chunk_size, tx_audio.size() - i);
        SampleSpan span(tx_audio.data() + i, len);
        demod2.process(span);
    }

    auto soft2 = demod2.getSoftBits();
    printf("Got %zu soft bits\n", soft2.size());

    if (soft2.size() >= 648) {
        std::span<const float> llrs(soft2.data(), 648);
        Bytes decoded = decoder.decodeSoft(llrs);

        if (decoder.lastDecodeSuccess()) {
            printf("LDPC SUCCESS! Decoded: \"");
            for (size_t i = 0; i < std::min((size_t)10, decoded.size()); i++) {
                char c = decoded[i];
                printf("%c", c >= 32 && c < 127 ? c : '?');
            }
            printf("\"\n");
        } else {
            printf("LDPC FAILED (iters=%d)\n", decoder.lastIterations());
        }
    }

    printf("\n=== Done ===\n");
    return 0;
}
