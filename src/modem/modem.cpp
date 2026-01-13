#include "ultra/modem.hpp"
#include "ultra/ofdm.hpp"
#include "ultra/fec.hpp"
#include "ultra/arq.hpp"
#include "ultra/dsp.hpp"

#include <queue>
#include <mutex>
#include <thread>
#include <atomic>
#include <condition_variable>

namespace ultra {

// Rate adaptation: recommend mode based on channel quality
std::pair<Modulation, CodeRate> recommendMode(const ChannelQuality& quality) {
    float snr = quality.snr_db;

    // Conservative thresholds for HF channel
    // These would be tuned based on real-world testing
    if (snr > 25) {
        return {Modulation::QAM64, CodeRate::R5_6};
    } else if (snr > 20) {
        return {Modulation::QAM64, CodeRate::R3_4};
    } else if (snr > 17) {
        return {Modulation::QAM16, CodeRate::R3_4};
    } else if (snr > 14) {
        return {Modulation::QAM16, CodeRate::R2_3};
    } else if (snr > 11) {
        return {Modulation::QPSK, CodeRate::R2_3};
    } else if (snr > 8) {
        return {Modulation::QPSK, CodeRate::R1_2};
    } else if (snr > 5) {
        return {Modulation::BPSK, CodeRate::R1_2};
    } else {
        return {Modulation::BPSK, CodeRate::R1_4};
    }
}

// Calculate theoretical max data rate
float calculateMaxDataRate(const ModemConfig& config, Modulation mod, CodeRate rate) {
    // Bits per carrier
    size_t bits_per_carrier = static_cast<size_t>(mod);

    // Assume ~36 data carriers (out of 48, rest are pilots)
    size_t data_carriers = config.num_carriers - config.num_carriers / config.pilot_spacing;

    // Bits per OFDM symbol
    size_t bits_per_symbol = data_carriers * bits_per_carrier;

    // Symbol duration (samples)
    float symbol_samples = config.getSymbolDuration();
    float symbol_duration = symbol_samples / config.sample_rate;

    // Raw bit rate
    float raw_bps = bits_per_symbol / symbol_duration;

    // Apply code rate
    float code_rate_val = 0.5f;
    switch (rate) {
        case CodeRate::R1_4: code_rate_val = 0.25f; break;
        case CodeRate::R1_3: code_rate_val = 0.333f; break;
        case CodeRate::R1_2: code_rate_val = 0.5f; break;
        case CodeRate::R2_3: code_rate_val = 0.667f; break;
        case CodeRate::R3_4: code_rate_val = 0.75f; break;
        case CodeRate::R5_6: code_rate_val = 0.833f; break;
        case CodeRate::R7_8: code_rate_val = 0.875f; break;
    }

    // Account for framing overhead (~10%)
    return raw_bps * code_rate_val * 0.9f;
}

struct Modem::Impl {
    ModemConfig config;

    // DSP components
    std::unique_ptr<OFDMModulator> modulator;
    std::unique_ptr<OFDMDemodulator> demodulator;
    std::unique_ptr<LDPCEncoder> encoder;
    std::unique_ptr<LDPCDecoder> decoder;
    std::unique_ptr<ARQController> arq;
    std::unique_ptr<FrameBuilder> frame_builder;
    std::unique_ptr<FrameParser> frame_parser;
    std::unique_ptr<Interleaver> interleaver;

    // State
    std::atomic<bool> running{false};
    std::atomic<bool> connected{false};
    bool adaptive_enabled = true;

    // TX queue and buffer
    std::queue<Bytes> tx_queue;
    Samples tx_buffer;
    std::mutex tx_mutex;

    // RX state
    std::mutex rx_mutex;

    // Callbacks
    DataCallback data_callback;
    StatsCallback stats_callback;

    // Statistics
    ModemStats stats;

    Impl(const ModemConfig& cfg)
        : config(cfg)
        , modulator(std::make_unique<OFDMModulator>(cfg))
        , demodulator(std::make_unique<OFDMDemodulator>(cfg))
        , encoder(std::make_unique<LDPCEncoder>(cfg.code_rate))
        , decoder(std::make_unique<LDPCDecoder>(cfg.code_rate))
        , arq(std::make_unique<ARQController>(cfg))
        , frame_builder(std::make_unique<FrameBuilder>(cfg))
        , frame_parser(std::make_unique<FrameParser>(cfg))
        , interleaver(std::make_unique<Interleaver>(32, 32))
    {
        setupARQCallbacks();
    }

    void setupARQCallbacks() {
        arq->setSendCallback([this](ByteSpan frame) {
            transmitFrame(frame);
        });

        arq->setDeliveryCallback([this](Bytes data) {
            if (data_callback) {
                data_callback(std::move(data));
            }
        });
    }

    void transmitFrame(ByteSpan frame) {
        // Encode with FEC
        Bytes coded = encoder->encode(frame);

        // Interleave
        Bytes interleaved = interleaver->interleave(coded);

        // Modulate
        Samples audio = modulator->modulate(interleaved, config.modulation);

        // Queue for transmission
        {
            std::lock_guard<std::mutex> lock(tx_mutex);
            tx_buffer.insert(tx_buffer.end(), audio.begin(), audio.end());
        }

        stats.frames_sent++;
        stats.bytes_sent += frame.size();
    }

    void processReceivedAudio(SampleSpan samples) {
        std::lock_guard<std::mutex> lock(rx_mutex);

        if (demodulator->process(samples)) {
            // Got enough samples for a frame
            auto soft_bits = demodulator->getSoftBits();

            // Deinterleave
            auto deinterleaved = interleaver->deinterleave(soft_bits);

            // FEC decode
            Bytes decoded = decoder->decodeSoft(deinterleaved);

            if (decoder->lastDecodeSuccess()) {
                // Parse frame
                auto parsed = frame_parser->parse(decoded);

                if (parsed.valid) {
                    handleFrame(parsed);
                    stats.frames_received++;
                    stats.bytes_received += parsed.payload.size();
                }
            }

            // Update stats
            auto quality = demodulator->getChannelQuality();
            stats.current_snr_db = quality.snr_db;

            // Rate adaptation
            if (adaptive_enabled) {
                auto [new_mod, new_rate] = recommendMode(quality);
                if (new_mod != config.modulation || new_rate != config.code_rate) {
                    config.modulation = new_mod;
                    config.code_rate = new_rate;
                    encoder->setRate(new_rate);
                    decoder->setRate(new_rate);
                    stats.current_modulation = new_mod;
                    stats.current_code_rate = new_rate;
                }
            }
        }
    }

    void handleFrame(const FrameParser::ParsedFrame& frame) {
        switch (frame.type) {
            case FrameType::DATA:
                arq->onDataReceived(frame.seq_num, frame.payload);
                // Send ACK
                {
                    auto ack = arq->generateAck();
                    transmitFrame(ack);
                }
                break;

            case FrameType::ACK:
                arq->onAckReceived(frame.seq_num, false, frame.remote_quality);
                break;

            case FrameType::NACK:
                arq->onAckReceived(frame.seq_num, true, frame.remote_quality);
                stats.frames_retransmitted++;
                break;

            case FrameType::CONNECT:
                connected = true;
                // Send ACK
                {
                    auto ack = frame_builder->buildAckFrame(0, demodulator->getChannelQuality());
                    transmitFrame(ack);
                }
                break;

            case FrameType::DISCONNECT:
                connected = false;
                arq->reset();
                break;

            default:
                break;
        }
    }
};

Modem::Modem(const ModemConfig& config)
    : impl_(std::make_unique<Impl>(config)) {}

Modem::~Modem() {
    stop();
}

void Modem::send(ByteSpan data) {
    if (!impl_->running) return;
    impl_->arq->sendData(data);
}

size_t Modem::getTxSamples(MutableSampleSpan buffer) {
    std::lock_guard<std::mutex> lock(impl_->tx_mutex);

    size_t to_copy = std::min(buffer.size(), impl_->tx_buffer.size());
    std::copy(impl_->tx_buffer.begin(), impl_->tx_buffer.begin() + to_copy,
              buffer.begin());
    impl_->tx_buffer.erase(impl_->tx_buffer.begin(),
                           impl_->tx_buffer.begin() + to_copy);

    return to_copy;
}

bool Modem::txPending() const {
    return !impl_->tx_buffer.empty();
}

void Modem::rxSamples(SampleSpan samples) {
    if (!impl_->running) return;
    impl_->processReceivedAudio(samples);
}

void Modem::setDataCallback(DataCallback cb) {
    impl_->data_callback = std::move(cb);
}

void Modem::start() {
    impl_->running = true;
}

void Modem::stop() {
    impl_->running = false;
}

bool Modem::isRunning() const {
    return impl_->running;
}

void Modem::connect() {
    auto connect_frame = impl_->frame_builder->buildConnectFrame();
    impl_->transmitFrame(connect_frame);

    // Also send preamble for sync
    auto preamble = impl_->modulator->generatePreamble();
    {
        std::lock_guard<std::mutex> lock(impl_->tx_mutex);
        impl_->tx_buffer.insert(impl_->tx_buffer.begin(),
                                preamble.begin(), preamble.end());
    }
}

void Modem::disconnect() {
    auto disconnect_frame = impl_->frame_builder->buildDisconnectFrame();
    impl_->transmitFrame(disconnect_frame);
    impl_->connected = false;
    impl_->arq->reset();
}

bool Modem::isConnected() const {
    return impl_->connected;
}

void Modem::setModulation(Modulation mod) {
    impl_->config.modulation = mod;
    impl_->adaptive_enabled = false;
}

void Modem::setCodeRate(CodeRate rate) {
    impl_->config.code_rate = rate;
    impl_->encoder->setRate(rate);
    impl_->decoder->setRate(rate);
    impl_->adaptive_enabled = false;
}

void Modem::setAdaptive(bool enable) {
    impl_->adaptive_enabled = enable;
}

ModemStats Modem::getStats() const {
    return impl_->stats;
}

void Modem::setStatsCallback(StatsCallback cb) {
    impl_->stats_callback = std::move(cb);
}

float Modem::getDataRate() const {
    return calculateMaxDataRate(impl_->config,
                                impl_->config.modulation,
                                impl_->config.code_rate);
}

ChannelQuality Modem::getChannelQuality() const {
    return impl_->demodulator->getChannelQuality();
}

} // namespace ultra
