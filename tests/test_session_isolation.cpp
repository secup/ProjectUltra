#include "ota_channel_core/session_manager.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <utility>
#include <vector>

using ultra::ota_channel_core::ChannelType;
using ultra::ota_channel_core::SessionConfig;
using ultra::ota_channel_core::SessionContext;
using ultra::ota_channel_core::SessionManager;

namespace {

void assertNear(const std::vector<float>& a,
                const std::vector<float>& b,
                float eps = 1.0e-6f) {
    assert(a.size() == b.size());
    for (size_t i = 0; i < a.size(); ++i) {
        assert(std::abs(a[i] - b[i]) <= eps);
    }
}

bool anyDifferent(const std::vector<float>& a,
                  const std::vector<float>& b,
                  float eps = 1.0e-6f) {
    assert(a.size() == b.size());
    for (size_t i = 0; i < a.size(); ++i) {
        if (std::abs(a[i] - b[i]) > eps) {
            return true;
        }
    }
    return false;
}

std::vector<float> receiveFromFreshControl(SessionConfig config,
                                           const std::vector<float>& samples) {
    SessionContext control(std::move(config));
    assert(control.registerStation("alice"));
    assert(control.registerStation("bob"));
    assert(control.submitTransmit("alice", 0, samples));
    return control.receiveForStation("bob", 0, samples.size());
}

}  // namespace

int main() {
    SessionManager manager({
        .session_id = "lobby",
        .display_name = "Lobby",
        .default_channel_model = ChannelType::PASSTHROUGH,
        .default_snr_db = 80.0f,
        .seed = 0x9999u,
        .station_cap = 4,
        .is_lobby = true,
    });

    SessionConfig awgn_config{
        .display_name = "AWGN Room",
        .default_channel_model = ChannelType::AWGN,
        .default_snr_db = 18.0f,
        .seed = 0xaaaau,
        .station_cap = 4,
    };
    SessionConfig good_config{
        .display_name = "Good HF Room",
        .default_channel_model = ChannelType::GOOD,
        .default_snr_db = 18.0f,
        .seed = 0xbbbbu,
        .station_cap = 4,
    };

    auto awgn = manager.createSession("room-a", awgn_config);
    auto good = manager.createSession("room-b", good_config);
    assert(awgn);
    assert(good);

    assert(awgn->registerStation("alice"));
    assert(awgn->registerStation("bob"));
    assert(good->registerStation("alice"));
    assert(good->registerStation("bob"));
    assert(awgn->stationCount() == 2);
    assert(good->stationCount() == 2);

    awgn->setCaptureEnabled(true);
    const auto good_rng_before = good->rngChildSeed("operator");

    const std::vector<float> tx{
        0.20f, 0.18f, 0.12f, 0.05f, -0.04f, -0.11f, -0.17f, -0.21f,
        -0.18f, -0.10f, 0.00f, 0.09f, 0.16f, 0.20f, 0.17f, 0.08f,
    };

    assert(awgn->submitTransmit("alice", 0, tx));
    const auto awgn_rx = awgn->receiveForStation("bob", 0, tx.size());
    awgn->appendEvent("operator_note", "alice", 3);

    assert(good->submitTransmit("alice", 0, tx));
    const auto good_rx = good->receiveForStation("bob", 0, tx.size());
    const auto good_control_rx = receiveFromFreshControl({
        .session_id = "room-b",
        .display_name = good_config.display_name,
        .default_channel_model = good_config.default_channel_model,
        .default_snr_db = good_config.default_snr_db,
        .seed = good_config.seed,
        .station_cap = good_config.station_cap,
    }, tx);

    assert(anyDifferent(awgn_rx, good_rx));
    assertNear(good_rx, good_control_rx);

    const auto awgn_capture = awgn->captureState();
    const auto good_capture = good->captureState();
    assert(awgn_capture.enabled);
    assert(awgn_capture.tx_samples == tx.size());
    assert(awgn_capture.rx_samples == tx.size());
    assert(!good_capture.enabled);
    assert(good_capture.tx_samples == 0);
    assert(good_capture.rx_samples == 0);

    const auto awgn_events = awgn->eventLog();
    const auto good_events = good->eventLog();
    bool awgn_has_operator_note = false;
    bool good_has_operator_note = false;
    for (const auto& event : awgn_events) {
        awgn_has_operator_note = awgn_has_operator_note ||
                                 event.type == "operator_note";
    }
    for (const auto& event : good_events) {
        good_has_operator_note = good_has_operator_note ||
                                 event.type == "operator_note";
    }
    assert(awgn_has_operator_note);
    assert(!good_has_operator_note);

    assert(good->rngChildSeed("operator") == good_rng_before);
    assert(awgn->rngChildSeed("operator") != good->rngChildSeed("operator"));

    std::cout << "session isolation deterministic\n";
    return 0;
}
