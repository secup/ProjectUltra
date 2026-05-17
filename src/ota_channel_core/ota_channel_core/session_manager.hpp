#pragma once

#include "ota_channel_core/session_config.hpp"
#include "ota_channel_core/session_context.hpp"

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <vector>

namespace ultra::ota_channel_core {

inline constexpr const char* kLobbySessionId = "lobby";

class SessionManager {
public:
    static SessionConfig defaultLobbyConfig();

    explicit SessionManager(SessionConfig lobby_config = defaultLobbyConfig());

    std::shared_ptr<SessionContext> createSession(std::string session_id,
                                                  SessionConfig config);
    std::shared_ptr<SessionContext> getSession(std::string_view session_id) const;
    std::vector<std::string> listSessions() const;
    bool closeSession(std::string_view session_id);

private:
    SessionConfig lobby_config_;
    std::map<std::string, std::shared_ptr<SessionContext>> sessions_;
    mutable std::mutex mutex_;
};

}  // namespace ultra::ota_channel_core
