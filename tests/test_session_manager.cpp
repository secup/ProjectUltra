#include "ota_channel_core/session_manager.hpp"

#include <algorithm>
#include <cassert>
#include <iostream>
#include <vector>

using ultra::ota_channel_core::ChannelType;
using ultra::ota_channel_core::SessionConfig;
using ultra::ota_channel_core::SessionManager;
using ultra::ota_channel_core::kLobbySessionId;

namespace {

bool contains(const std::vector<std::string>& values, const std::string& value) {
    return std::find(values.begin(), values.end(), value) != values.end();
}

}  // namespace

int main() {
    SessionConfig lobby{
        .session_id = kLobbySessionId,
        .display_name = "Default Lobby",
        .default_channel_model = ChannelType::GOOD,
        .default_snr_db = 17.5f,
        .seed = 0x7777u,
        .station_cap = 4,
        .is_lobby = true,
    };

    SessionManager manager(lobby);
    auto lobby_session = manager.getSession(kLobbySessionId);
    assert(lobby_session);
    assert(lobby_session->config().is_lobby);
    assert(lobby_session->channelType() == ChannelType::GOOD);
    assert(lobby_session->snrDb() == 17.5f);
    assert(lobby_session->seed() == 0x7777u);
    assert(lobby_session->config().station_cap == 4);

    auto ids = manager.listSessions();
    assert(ids.size() == 1);
    assert(contains(ids, kLobbySessionId));

    SessionConfig private_config{
        .display_name = "Night Test",
        .default_channel_model = ChannelType::AWGN,
        .default_snr_db = 25.0f,
        .seed = 0x1111u,
        .station_cap = 2,
    };

    auto night = manager.createSession("night", private_config);
    auto dawn = manager.createSession("dawn", {
        .display_name = "Dawn",
        .default_channel_model = ChannelType::MODERATE,
        .default_snr_db = 13.0f,
        .seed = 0x2222u,
        .station_cap = 3,
    });
    assert(night);
    assert(dawn);
    assert(!night->config().is_lobby);
    assert(night->config().session_id == "night");
    assert(night->channelType() == ChannelType::AWGN);
    assert(!manager.createSession("night", private_config));

    ids = manager.listSessions();
    assert(ids.size() == 3);
    assert(contains(ids, kLobbySessionId));
    assert(contains(ids, "night"));
    assert(contains(ids, "dawn"));

    assert(manager.closeSession("night"));
    assert(!manager.getSession("night"));
    assert(!manager.closeSession("night"));
    assert(!manager.closeSession(kLobbySessionId));
    assert(manager.getSession(kLobbySessionId));

    ids = manager.listSessions();
    assert(ids.size() == 2);
    assert(contains(ids, kLobbySessionId));
    assert(contains(ids, "dawn"));

    std::cout << "session manager lifecycle deterministic\n";
    return 0;
}
