#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>

namespace ultra::ota_simulator_service {

struct AuthPrincipal {
    std::string token;
    std::string callsign;
    std::string label;
    // Admin role grants permission to call destructive / shared-state-
    // mutating RPCs (SetChannel, InjectEffect, CancelEffect, CreateSession,
    // StartCapture, StopCapture). Operator-role tokens can join sessions
    // and exchange audio but cannot reconfigure the channel mid-QSO.
    bool admin = false;
};

class AuthAllowlist {
public:
    bool loadFromFile(const std::filesystem::path& path, std::string* error = nullptr);
    bool empty() const { return entries_.empty(); }
    size_t size() const { return entries_.size(); }

    std::optional<AuthPrincipal> authenticate(std::string_view token) const;

private:
    std::unordered_map<std::string, AuthPrincipal> entries_;
};

std::optional<std::string> bearerTokenFromAuthorization(std::string_view authorization);

}  // namespace ultra::ota_simulator_service
