#include "ota_channel_core/session_context.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

using ultra::ota_channel_core::ChannelType;
using ultra::ota_channel_core::SessionAudioBlock;
using ultra::ota_channel_core::SessionConfig;
using ultra::ota_channel_core::SessionContext;

namespace {

std::vector<float> constant(size_t count, float value) {
    return std::vector<float>(count, value);
}

void assertVectorNear(const std::vector<float>& actual,
                      const std::vector<float>& expected) {
    assert(actual.size() == expected.size());
    for (size_t i = 0; i < actual.size(); ++i) {
        assert(std::abs(actual[i] - expected[i]) < 1.0e-6f);
    }
}

const SessionAudioBlock* findBlock(const std::vector<SessionAudioBlock>& blocks,
                                   const std::string& station_id,
                                   uint64_t start_sample) {
    for (const auto& block : blocks) {
        if (block.station_id == station_id && block.start_sample == start_sample) {
            return &block;
        }
    }
    return nullptr;
}

std::unique_ptr<SessionContext> makeSession(const std::string& id, bool half_duplex = true) {
    SessionConfig config;
    config.half_duplex_radios = half_duplex;
    config.session_id = id;
    config.display_name = id;
    config.default_channel_model = ChannelType::PASSTHROUGH;
    config.default_snr_db = 80.0f;
    config.seed = 0x5150u;
    config.station_cap = 3;
    auto session = std::make_unique<SessionContext>(std::move(config));
    assert(session->registerStation("alice"));
    assert(session->registerStation("bob"));
    assert(session->registerStation("charlie"));
    return session;
}

}  // namespace

// Half-duplex radios (the default). The PTT is read from each station's own transmitted content:
// keyed from ptt_lead_ms before its first non-zero sample to ptt_tail_ms after its last, then
// tr_recovery_ms more of deafness. The others still hear the keyed station.
constexpr size_t kLead = 150 * 48;            // ptt_lead_ms
constexpr size_t kHang = (50 + 20) * 48;      // ptt_tail_ms + tr_recovery_ms

struct Rx {
    std::vector<float> alice, bob, charlie;
};

Rx runTicks(SessionContext& session, size_t ticks) {
    Rx rx;
    for (size_t k = 0; k < ticks; ++k) {
        const auto t = session.advanceSessionClock();
        const auto blocks = session.drainReceiveOutbox();
        const auto* a = findBlock(blocks, "alice", t.start_sample);
        const auto* b = findBlock(blocks, "bob", t.start_sample);
        const auto* c = findBlock(blocks, "charlie", t.start_sample);
        assert(a && b && c);
        rx.alice.insert(rx.alice.end(), a->samples.begin(), a->samples.end());
        rx.bob.insert(rx.bob.end(), b->samples.begin(), b->samples.end());
        rx.charlie.insert(rx.charlie.end(), c->samples.begin(), c->samples.end());
    }
    return rx;
}

// Paced queue path (the per-tick TX queue): alice keys half a tick at the start.
void checkHalfDuplexQueued() {
    auto session = makeSession("half-duplex-queued");
    const size_t tick = session->sessionTickSamples();
    const size_t keyed = tick / 2;
    const size_t ticks = (keyed + kHang) / tick + 2;
    const size_t total = ticks * tick;
    assert(session->enqueueTransmit("alice", constant(keyed, 0.25f)));
    assert(session->enqueueTransmit("bob", constant(total, -0.5f)));  // bob keyed throughout
    const Rx rx = runTicks(*session, ticks);
    for (size_t i = 0; i < total; ++i) {
        const bool alice_deaf = i < keyed + kHang;
        assert(std::abs(rx.alice[i] - (alice_deaf ? 0.0f : -0.5f)) < 1.0e-6f);
        assert(rx.bob[i] == 0.0f);
        const float alice_tx = i < keyed ? 0.25f : 0.0f;
        assert(std::abs(rx.charlie[i] - (alice_tx - 0.5f)) < 1.0e-6f);  // not keyed: hears both
    }
    std::cout << "half-duplex radios (queued TX): keyed + " << kHang << " samples deaf\n";
}

// Scheduled path (how the GUI's audio plane delivers TX: audio placed at session positions, zeros
// when idle): alice's burst sits ahead of the clock, so her PTT goes down kLead before it.
void checkHalfDuplexScheduled() {
    auto session = makeSession("half-duplex-scheduled");
    const size_t tick = session->sessionTickSamples();
    const size_t burst_at = kLead + 4 * tick;   // ahead of the clock by more than the lead
    const size_t burst_len = 3 * tick;
    const size_t ticks = (burst_at + burst_len + kHang) / tick + 2;
    const size_t total = ticks * tick;
    std::vector<float> alice_audio(burst_at + burst_len, 0.0f);   // idle zeros, then the burst
    std::fill(alice_audio.begin() + static_cast<std::ptrdiff_t>(burst_at), alice_audio.end(), 0.25f);
    assert(session->submitTransmit("alice", 0, alice_audio));
    assert(session->submitTransmit("bob", 0, constant(total, -0.5f)));
    const Rx rx = runTicks(*session, ticks);
    for (size_t i = 0; i < total; ++i) {
        const bool alice_deaf = i + kLead >= burst_at && i < burst_at + burst_len + kHang;
        assert(std::abs(rx.alice[i] - (alice_deaf ? 0.0f : -0.5f)) < 1.0e-6f);
    }
    std::cout << "half-duplex radios (scheduled TX): deaf from " << kLead
              << " samples before the burst to " << kHang << " after\n";
}

int main() {
    checkHalfDuplexQueued();
    checkHalfDuplexScheduled();
    auto session = makeSession("medium-mixer", /*half_duplex=*/false);
    const size_t tick_samples = session->sessionTickSamples();
    const auto alice_signal = constant(tick_samples, 0.25f);
    const auto bob_signal = constant(tick_samples, -0.5f);
    const auto charlie_signal = constant(tick_samples, 0.125f);

    assert(session->enqueueTransmit("alice", alice_signal));
    assert(session->enqueueTransmit("bob", bob_signal));
    assert(session->enqueueTransmit("charlie", charlie_signal));

    const auto tick = session->advanceSessionClock();
    const auto rx_blocks = session->drainReceiveOutbox();
    const auto* alice_rx = findBlock(rx_blocks, "alice", tick.start_sample);
    const auto* bob_rx = findBlock(rx_blocks, "bob", tick.start_sample);
    const auto* charlie_rx = findBlock(rx_blocks, "charlie", tick.start_sample);
    assert(alice_rx);
    assert(bob_rx);
    assert(charlie_rx);

    std::vector<float> alice_expected(tick_samples);
    std::vector<float> bob_expected(tick_samples);
    std::vector<float> charlie_expected(tick_samples);
    for (size_t i = 0; i < tick_samples; ++i) {
        alice_expected[i] = bob_signal[i] + charlie_signal[i];
        bob_expected[i] = alice_signal[i] + charlie_signal[i];
        charlie_expected[i] = alice_signal[i] + bob_signal[i];
    }

    assertVectorNear(alice_rx->samples, alice_expected);
    assertVectorNear(bob_rx->samples, bob_expected);
    assertVectorNear(charlie_rx->samples, charlie_expected);

    std::cout << "session medium mixer carries simultaneous stations and only removes self audio\n";
    return 0;
}
