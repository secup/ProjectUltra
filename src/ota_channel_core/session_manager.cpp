#include "ota_channel_core/session_manager.hpp"

#include <utility>

namespace ultra::ota_channel_core {

SessionConfig SessionManager::defaultLobbyConfig() {
    return {.session_id = kLobbySessionId,
            .display_name = "Lobby",
            .default_channel_model = ChannelType::AWGN,
            .default_snr_db = 20.0f,
            .seed = 42,
            .station_cap = 16,
            .is_lobby = true};
}

SessionManager::SessionManager(SessionConfig lobby_config)
    : lobby_config_(std::move(lobby_config)) {
    if (lobby_config_.session_id.empty()) {
        lobby_config_.session_id = kLobbySessionId;
    }
    if (lobby_config_.display_name.empty()) {
        lobby_config_.display_name = "Lobby";
    }
    lobby_config_.is_lobby = true;

    auto lobby = std::make_shared<SessionContext>(lobby_config_);
    sessions_.emplace(lobby_config_.session_id, std::move(lobby));
}

std::shared_ptr<SessionContext> SessionManager::createSession(std::string session_id,
                                                              SessionConfig config) {
    if (session_id.empty()) {
        return nullptr;
    }

    config.session_id = std::move(session_id);
    if (config.display_name.empty()) {
        config.display_name = config.session_id;
    }
    config.is_lobby = false;

    auto context = std::make_shared<SessionContext>(std::move(config));

    std::lock_guard<std::mutex> lock(mutex_);
    auto [it, inserted] = sessions_.emplace(context->id(), context);
    if (!inserted) {
        return nullptr;
    }
    return it->second;
}

std::shared_ptr<SessionContext> SessionManager::getSession(
    std::string_view session_id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = sessions_.find(std::string(session_id));
    if (it == sessions_.end()) {
        return nullptr;
    }
    return it->second;
}

std::vector<std::string> SessionManager::listSessions() const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<std::string> ids;
    ids.reserve(sessions_.size());
    for (const auto& [id, _] : sessions_) {
        ids.push_back(id);
    }
    return ids;
}

bool SessionManager::closeSession(std::string_view session_id) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = sessions_.find(std::string(session_id));
    if (it == sessions_.end() || it->second->config().is_lobby) {
        return false;
    }
    sessions_.erase(it);
    return true;
}

}  // namespace ultra::ota_channel_core
