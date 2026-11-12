// ModemEngine RX - Thread Management and Audio Input
// Decode logic is in modem_rx_decode.cpp

#include "modem_engine.hpp"
#include "modem_rx_constants.hpp"
#include "ultra/logging.hpp"
#include <fstream>

namespace ultra {
namespace gui {

using namespace rx_constants;

// ============================================================================
// RX/DECODE THREAD
// ============================================================================

void ModemEngine::setSynchronousMode(bool enabled) {
    if (synchronous_mode_ == enabled) return;

    synchronous_mode_ = enabled;

    if (enabled) {
        // Stop decode threads — caller will drive processRxBuffer() directly
        stopRxListenerThread();
        stopRxDecodeThread();
        LOG_MODEM(INFO, "[%s] Synchronous RX mode enabled (no decode thread)", log_prefix_.c_str());
    } else {
        // Restart decode threads for async operation
        startRxDecodeThread();
        startRxListenerThread();
        LOG_MODEM(INFO, "[%s] Asynchronous RX mode enabled (decode thread)", log_prefix_.c_str());
    }
}

void ModemEngine::drainDecodedFrames(StreamingDecoder* decoder) {
    // Frame delivery happens via callback (set when the decoder was wired); this queue is for
    // stats. A listener's failures are not this session's evidence (dual-listen design).
    const bool primary = streaming_decoder_.isPrimary(decoder);
    while (decoder->hasFrame()) {
        auto result = decoder->getFrame();
        if (result.success) {
            updateStats([&](LoopbackStats& s) {
                s.frames_received++;
                copyDecodeSnrMetrics(s, result);
                s.synced = true;
            });
        } else if (primary && !result.is_ping && result.codewords_failed > 0) {
            updateStats([](LoopbackStats& s) { s.frames_failed++; });
        }
    }
}

void ModemEngine::processRxBuffer() {
    if (!streaming_decoder_) return;

    // Same as one iteration of rxDecodeLoop — process + deliver frames (both decoders while
    // dual-listen runs; each slot is an object, whichever role it has)
    for (int slot = 0; slot < 2; ++slot) {
        StreamingDecoder* decoder = streaming_decoder_.slot(slot);
        if (decoder == nullptr ||
            (!streaming_decoder_.isPrimary(decoder) && !streaming_decoder_.listenerActive())) {
            continue;
        }
        decoder->processBuffer();
        drainDecodedFrames(decoder);
    }
}

void ModemEngine::startRxDecodeThread() {
    if (rx_decode_running_ || synchronous_mode_) return;

    rx_decode_running_ = true;
    rx_decode_thread_ = std::thread(&ModemEngine::rxDecodeLoop, this);
    LOG_MODEM(INFO, "[%s] RX decode thread started", log_prefix_.c_str());
}

void ModemEngine::stopRxDecodeThread() {
    if (!rx_decode_running_) return;

    rx_decode_running_ = false;
    rx_decode_cv_.notify_all();

    // Signal the slot-0 StreamingDecoder (this thread's object) to wake up and exit
    StreamingDecoder* decoder = streaming_decoder_.slot(0);
    if (decoder) {
        decoder->stop();
    }

    if (rx_decode_thread_.joinable()) {
        rx_decode_thread_.join();
    }

    // Clear shutdown flag so decoder can be reused:
    // - In synchronous mode via processRxBuffer()
    // - Or via a new decode thread (startRxDecodeThread)
    if (decoder) {
        decoder->clearShutdown();
    }

    LOG_MODEM(INFO, "[%s] RX decode thread stopped", log_prefix_.c_str());
}

void ModemEngine::rxDecodeLoop() {
    // The slot-0 decoder object: the primary unless dual-listen flipped the roles.
    rxDecodeLoopFor(streaming_decoder_.slot(0), rx_decode_running_);
}

void ModemEngine::rxDecodeLoopFor(StreamingDecoder* decoder, std::atomic<bool>& running) {
    LOG_MODEM(INFO, "[%s] RX decode loop starting", log_prefix_.c_str());

    while (running) {
        // ====================================================================
        // StreamingDecoder handles BOTH connected and disconnected modes:
        // - Circular buffer with sliding window chirp detection
        // - Correct IWaveform call sequence (fixes BUG-002)
        // - PING detection via energy ratio
        // - Frame delivery via callbacks (set when the decoder was wired)
        // This loop drives one decoder object; dual-listen runs a second loop on the other slot.
        // ====================================================================

        if (decoder == nullptr) {
            // Fallback: no decoder available, just sleep
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        // Drive the decode process (blocks until data available or timeout)
        decoder->processBuffer();
        drainDecodedFrames(decoder);
    }

    LOG_MODEM(INFO, "[%s] RX decode loop exiting", log_prefix_.c_str());
}

void ModemEngine::stopRxListenerThread() {
    if (!rx_listener_running_) {
        return;
    }
    rx_listener_running_ = false;
    StreamingDecoder* decoder = streaming_decoder_.slot(1);
    if (decoder) {
        decoder->stop();
    }
    if (rx_listener_thread_.joinable()) {
        rx_listener_thread_.join();
    }
    if (decoder) {
        decoder->clearShutdown();
    }
}

// ============================================================================
// AUDIO INPUT
// ============================================================================

void ModemEngine::feedAudio(const float* samples, size_t count) {
    if (count == 0 || samples == nullptr) return;

    // Lazy-start async decode thread on first audio, instead of in constructor.
    if (!synchronous_mode_ && !rx_decode_running_) {
        startRxDecodeThread();
    }

    // ========================================================================
    // AUDIO ROUTING - StreamingDecoder is PRIMARY
    // ========================================================================
    // StreamingDecoder handles BOTH connected and disconnected modes:
    // - Circular buffer with bounded size (4 seconds)
    // - Sliding window chirp detection
    // - Correct IWaveform call sequence (fixes BUG-002)
    // - Thread-safe with condition variable
    // ========================================================================

    if (streaming_decoder_) {
        streaming_decoder_->feedAudio(samples, count);
    }
    // Dual-listen: the listener on the other waveform hears the same audio.
    if (StreamingDecoder* listener = streaming_decoder_.listener()) {
        listener->feedAudio(samples, count);
    }

    // Carrier sense (listen-before-talk): feed the shared detector the same RX
    // audio. local_rx_blackout=false here — in listen mode we are not keyed.
    // (The TX-deaf blackout + the actual transmit gate land in a follow-up.)
    channel_busy_detector_.observeSamples(std::span<const float>(samples, count),
                                          /*local_rx_blackout=*/false);
    cca_samples_since_log_ += count;
    if (cca_samples_since_log_ >= 48000) {  // ~1 s at 48 kHz
        cca_samples_since_log_ = 0;
        LOG_MODEM(INFO, "[%s] CCA: rms=%.4f quiet_thresh=%.4f idle=%d ofdm_gated=%d",
                  log_prefix_.c_str(),
                  channel_busy_detector_.currentRms(),
                  channel_busy_detector_.quietThreshold(),
                  channel_busy_detector_.isIdle() ? 1 : 0,
                  carrierSenseActiveForTx() ? 1 : 0);
    }
}

void ModemEngine::feedAudio(const std::vector<float>& samples) {
    feedAudio(samples.data(), samples.size());
}

size_t ModemEngine::injectSignalFromFile(const std::string& filepath) {
    std::ifstream file(filepath, std::ios::binary | std::ios::ate);
    if (!file) {
        LOG_MODEM(ERROR, "Failed to open signal file: %s", filepath.c_str());
        return 0;
    }

    size_t file_size = file.tellg();
    size_t num_samples = file_size / sizeof(float);
    file.seekg(0);

    std::vector<float> samples(num_samples);
    file.read(reinterpret_cast<char*>(samples.data()), file_size);

    if (!file) {
        LOG_MODEM(ERROR, "Failed to read signal file: %s", filepath.c_str());
        return 0;
    }

    LOG_MODEM(INFO, "Injecting %zu samples from %s", num_samples, filepath.c_str());

    for (size_t offset = 0; offset < num_samples; offset += INJECT_CHUNK_SIZE) {
        size_t chunk = std::min(INJECT_CHUNK_SIZE, num_samples - offset);
        feedAudio(samples.data() + offset, chunk);
        std::this_thread::sleep_for(std::chrono::milliseconds(INJECT_CHUNK_DELAY_MS));
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(INJECT_FINAL_DELAY_MS));
    return num_samples;
}

} // namespace gui
} // namespace ultra
