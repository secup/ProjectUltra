#include "ota_simulator_service/auth_allowlist.hpp"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <sstream>
#include <utility>

namespace ultra::ota_simulator_service {
namespace {

std::string trim(std::string value) {
    auto is_space = [](unsigned char c) { return std::isspace(c) != 0; };
    value.erase(value.begin(),
                std::find_if(value.begin(), value.end(),
                             [&](char c) { return !is_space(static_cast<unsigned char>(c)); }));
    value.erase(std::find_if(value.rbegin(), value.rend(),
                             [&](char c) { return !is_space(static_cast<unsigned char>(c)); }).base(),
                value.end());
    return value;
}

bool startsWithBearer(std::string_view value) {
    constexpr std::string_view prefix = "Bearer ";
    if (value.size() < prefix.size()) {
        return false;
    }
    for (size_t i = 0; i < prefix.size(); ++i) {
        if (value[i] != prefix[i]) {
            return false;
        }
    }
    return true;
}

}  // namespace

bool AuthAllowlist::loadFromFile(const std::filesystem::path& path, std::string* error) {
    std::ifstream in(path);
    if (!in) {
        if (error) {
            *error = "failed to open token file: " + path.string();
        }
        return false;
    }

    std::unordered_map<std::string, AuthPrincipal> parsed;
    std::string line;
    size_t line_number = 0;
    while (std::getline(in, line)) {
        ++line_number;
        line = trim(line);
        if (line.empty() || line.front() == '#') {
            continue;
        }

        const auto first = line.find(':');
        const auto second = first == std::string::npos ? std::string::npos
                                                       : line.find(':', first + 1);
        if (first == std::string::npos || second == std::string::npos) {
            if (error) {
                *error = "invalid token line " + std::to_string(line_number);
            }
            return false;
        }

        AuthPrincipal principal{
            .token = trim(line.substr(0, first)),
            .callsign = trim(line.substr(first + 1, second - first - 1)),
            .label = trim(line.substr(second + 1)),
        };
        if (principal.token.empty() || principal.callsign.empty()) {
            if (error) {
                *error = "empty token or callsign on line " + std::to_string(line_number);
            }
            return false;
        }
        parsed[principal.token] = principal;
    }

    entries_ = std::move(parsed);
    return true;
}

std::optional<AuthPrincipal> AuthAllowlist::authenticate(std::string_view token) const {
    const auto it = entries_.find(std::string(token));
    if (it == entries_.end()) {
        return std::nullopt;
    }
    return it->second;
}

std::optional<std::string> bearerTokenFromAuthorization(std::string_view authorization) {
    if (!startsWithBearer(authorization)) {
        return std::nullopt;
    }
    std::string token(authorization.substr(7));
    token = trim(std::move(token));
    if (token.empty()) {
        return std::nullopt;
    }
    return token;
}

}  // namespace ultra::ota_simulator_service
