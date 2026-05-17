#include "ota_simulator_service/auth_allowlist.hpp"
#include "helpers/temp_dir.hpp"

#include <cassert>
#include <fstream>
#include <iostream>

using ultra::ota_simulator_service::AuthAllowlist;
using ultra::ota_simulator_service::bearerTokenFromAuthorization;

int main() {
    ultra::test::TempDir temp("otasim_auth_allowlist");
    assert(temp.valid());
    const auto path = temp.child("tokens.conf");
    {
        std::ofstream out(path);
        out << "# format: <token>:<callsign>:<label>\n";
        out << "alice_token:8P9QC:Mathieu\n";
        out << "bob_token:KC3VPB:Field laptop\n";
        out << "\n";
    }

    AuthAllowlist allowlist;
    std::string error;
    assert(allowlist.loadFromFile(path, &error));
    assert(error.empty());
    assert(allowlist.size() == 2);

    auto alice = allowlist.authenticate("alice_token");
    assert(alice);
    assert(alice->callsign == "8P9QC");
    assert(alice->label == "Mathieu");
    assert(!allowlist.authenticate("missing_token"));
    assert(!allowlist.authenticate(""));

    auto token = bearerTokenFromAuthorization("Bearer bob_token");
    assert(token);
    assert(*token == "bob_token");
    assert(!bearerTokenFromAuthorization("bob_token"));
    assert(!bearerTokenFromAuthorization("Bearer "));

    const auto bad_path = temp.child("bad_tokens.conf");
    {
        std::ofstream out(bad_path);
        out << "not enough fields\n";
    }
    AuthAllowlist bad;
    assert(!bad.loadFromFile(bad_path, &error));
    assert(!error.empty());

    std::cout << "auth allowlist parses and authenticates bearer tokens\n";
    return 0;
}
