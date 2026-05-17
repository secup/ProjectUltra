#include "ota_channel_core/session_context.hpp"

#include <cassert>
#include <iostream>
#include <vector>

using ultra::ota_channel_core::ChannelType;
using ultra::ota_channel_core::SessionConfig;
using ultra::ota_channel_core::SessionContext;

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

    std::cout << "session context lifecycle deterministic\n";
    return 0;
}
