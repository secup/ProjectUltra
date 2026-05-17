#include "ota_channel_core/session_context.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

using ultra::ota_channel_core::ChannelType;
using ultra::ota_channel_core::SessionAudioBlock;
using ultra::ota_channel_core::SessionConfig;
using ultra::ota_channel_core::SessionContext;

namespace {

void assertEqual(const std::vector<float>& actual, const std::vector<float>& expected) {
    assert(actual.size() == expected.size());
    for (size_t i = 0; i < actual.size(); ++i) {
        assert(actual[i] == expected[i]);
    }
}

std::vector<float> ramp(size_t count, float scale = 0.001f) {
    std::vector<float> out(count);
    for (size_t i = 0; i < out.size(); ++i) {
        out[i] = static_cast<float>(i + 1) * scale;
    }
    return out;
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

}  // namespace

int main() {
    SessionContext session({
        .session_id = "alpha",
        .display_name = "Alpha",
        .default_channel_model = ChannelType::PASSTHROUGH,
        .default_snr_db = 40.0f,
        .seed = 0x1010u,
        .station_cap = 2,
    });

    assert(session.id() == "alpha");
    assert(session.stationCount() == 0);
    assert(session.registerStation("alice"));
    assert(session.registerStation("bob"));
    assert(!session.registerStation("alice"));
    assert(!session.registerStation("carol"));
    assert(session.hasStation("alice"));
    assert(session.hasStation("bob"));
    assert(session.stationCount() == 2);

    const auto stations = session.listStations();
    assert(stations.size() == 2);
    assert(stations[0] == "alice");
    assert(stations[1] == "bob");

    assert(session.leaveStation("alice"));
    assert(!session.hasStation("alice"));
    assert(!session.leaveStation("alice"));
    assert(session.stationCount() == 1);

    SessionContext same_seed({
        .session_id = "alpha",
        .display_name = "Alpha Again",
        .default_channel_model = ChannelType::PASSTHROUGH,
        .default_snr_db = 40.0f,
        .seed = 0x1010u,
    });
    SessionContext different_seed({
        .session_id = "alpha",
        .display_name = "Alpha Different Seed",
        .default_channel_model = ChannelType::PASSTHROUGH,
        .default_snr_db = 40.0f,
        .seed = 0x2020u,
    });

    auto a = session.rngStream("operator");
    auto b = same_seed.rngStream("operator");
    auto c = different_seed.rngStream("operator");
    for (int i = 0; i < 16; ++i) {
        assert(a.nextU32() == b.nextU32());
    }

    bool any_different = false;
    a = session.rngStream("operator");
    for (int i = 0; i < 16; ++i) {
        if (a.nextU32() != c.nextU32()) {
            any_different = true;
            break;
        }
    }
    assert(any_different);

    const std::vector<float> tx{0.1f, -0.2f, 0.3f};
    assert(!session.submitTransmit("alice", 0, tx));
    assert(session.submitTransmit("bob", 0, tx));
    assert(session.pendingAudioBlocks() == 1);
    session.discardBefore(tx.size());
    assert(session.pendingAudioBlocks() == 0);

    SessionContext clocked({
        .session_id = "clocked",
        .display_name = "Clocked",
        .default_channel_model = ChannelType::PASSTHROUGH,
        .default_snr_db = 80.0f,
        .seed = 0x3030u,
        .station_cap = 2,
    });
    assert(clocked.registerStation("alice"));
    assert(clocked.registerStation("bob"));
    assert(clocked.sessionClockSamples() == 0);
    assert(clocked.sessionTickSamples() == 480);

    const auto tick_tx = ramp(clocked.sessionTickSamples());
    assert(clocked.enqueueTransmit("alice", tick_tx));
    auto tick = clocked.advanceSessionClock();
    assert(tick.start_sample == 0);
    assert(tick.sample_count == clocked.sessionTickSamples());
    assert(clocked.sessionClockSamples() == clocked.sessionTickSamples());

    auto rx_blocks = clocked.drainReceiveOutbox();
    const auto* bob_rx = findBlock(rx_blocks, "bob", 0);
    assert(bob_rx);
    assertEqual(bob_rx->samples, tick_tx);
    const auto* alice_rx = findBlock(rx_blocks, "alice", 0);
    assert(alice_rx);
    assertEqual(alice_rx->samples, std::vector<float>(clocked.sessionTickSamples(), 0.0f));

    tick = clocked.advanceSessionClock();
    assert(tick.start_sample == clocked.sessionTickSamples());
    rx_blocks = clocked.drainReceiveOutbox();
    bob_rx = findBlock(rx_blocks, "bob", clocked.sessionTickSamples());
    assert(bob_rx);
    assertEqual(bob_rx->samples, std::vector<float>(clocked.sessionTickSamples(), 0.0f));

    SessionContext fast_sender({
        .session_id = "fast-sender",
        .display_name = "Fast Sender",
        .default_channel_model = ChannelType::PASSTHROUGH,
        .default_snr_db = 80.0f,
        .seed = 0x4040u,
        .station_cap = 2,
    });
    assert(fast_sender.registerStation("fast"));
    assert(fast_sender.registerStation("slow"));

    constexpr double kFastPpm = 73.0;
    constexpr size_t kTicks = 1000;
    double sender_cursor = 0.0;
    uint64_t sent_samples = 0;
    uint64_t delivered_samples = 0;
    for (size_t i = 0; i < kTicks; ++i) {
        sender_cursor += static_cast<double>(fast_sender.sessionTickSamples()) *
                         (1.0 + kFastPpm / 1'000'000.0);
        const uint64_t should_have_sent =
            static_cast<uint64_t>(std::floor(sender_cursor));
        const size_t send_count = static_cast<size_t>(should_have_sent - sent_samples);
        sent_samples = should_have_sent;
        assert(fast_sender.enqueueTransmit("fast", std::vector<float>(send_count, 1.0f)));

        tick = fast_sender.advanceSessionClock();
        assert(tick.start_sample == i * fast_sender.sessionTickSamples());
        rx_blocks = fast_sender.drainReceiveOutbox();
        const auto* slow_rx = findBlock(rx_blocks, "slow", tick.start_sample);
        assert(slow_rx);
        assert(slow_rx->samples.size() == fast_sender.sessionTickSamples());
        delivered_samples += slow_rx->samples.size();
    }
    assert(delivered_samples == kTicks * fast_sender.sessionTickSamples());
    assert(sent_samples > delivered_samples);
    assert(fast_sender.pendingTransmitSamples("fast") == sent_samples - delivered_samples);

    SessionContext bounded({
        .session_id = "bounded",
        .display_name = "Bounded",
        .default_channel_model = ChannelType::PASSTHROUGH,
        .default_snr_db = 80.0f,
        .seed = 0x5050u,
        .station_cap = 2,
    });
    assert(bounded.registerStation("fast"));
    assert(bounded.enqueueTransmit(
        "fast",
        std::vector<float>(bounded.maxQueuedSamples() + bounded.sessionTickSamples(), 0.25f)));
    assert(bounded.pendingTransmitSamples("fast") == bounded.maxQueuedSamples());

    std::cout << "session context lifecycle deterministic\n";
    return 0;
}
