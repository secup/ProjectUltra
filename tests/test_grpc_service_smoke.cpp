#include "helpers/temp_dir.hpp"
#include "ota_channel_core/session_manager.hpp"
#include "ota_simulator.grpc.pb.h"
#include "ota_simulator_service/auth_allowlist.hpp"
#include "ota_simulator_service/ota_simulator_service.hpp"

#include <cassert>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <utility>

#include <grpcpp/grpcpp.h>

namespace {

namespace otasim = projectultra::otasim::v1;

void addToken(grpc::ClientContext& context, const std::string& token) {
    context.AddMetadata("authorization", "Bearer " + token);
}

bool hasSession(const otasim::ListSessionsResponse& response, const std::string& session_id) {
    for (const auto& session : response.sessions()) {
        if (session.session_id() == session_id) {
            return true;
        }
    }
    return false;
}

}  // namespace

int main() {
    ultra::test::TempDir temp("grpc_service_smoke");
    assert(temp.valid());

    const auto token_path = temp.child("tokens.conf");
    {
        std::ofstream out(token_path);
        // Alice runs the smoke test end-to-end including admin RPCs
        // (CreateSession + SetChannel), so she carries the admin role.
        // Bob stays operator-only to mirror a normal joined station.
        out << "alice_token:ALPHA:Alpha station:admin\n";
        out << "bob_token:BRAVO:Bravo station\n";
    }

    ultra::ota_simulator_service::AuthAllowlist auth;
    std::string error;
    assert(auth.loadFromFile(token_path, &error));

    ultra::ota_simulator_service::OtaSimulatorServiceConfig config;
    config.udp_bind_host = "127.0.0.1";
    config.udp_bind_port = 0;
    config.capture_root = temp.child("captures");

    ultra::ota_simulator_service::OtaSimulatorService service(std::move(auth), config);
    assert(service.start(&error));

    grpc::ServerBuilder builder;
    builder.RegisterService(&service);
    std::unique_ptr<grpc::Server> server = builder.BuildAndStart();
    assert(server);

    auto channel = server->InProcessChannel(grpc::ChannelArguments());
    auto stub = otasim::OtaSimulatorControl::NewStub(channel);

    otasim::RegisterStationRequest register_request;
    otasim::StationLease lease;
    grpc::ClientContext context;
    addToken(context, "alice_token");
    auto status = stub->RegisterStation(&context, register_request, &lease);
    assert(status.ok());
    assert(lease.station_id() == "ALPHA");
    assert(lease.callsign() == "ALPHA");

    otasim::StationLease invalid_lease;
    grpc::ClientContext invalid_context;
    addToken(invalid_context, "invalid_token");
    status = stub->RegisterStation(&invalid_context, register_request, &invalid_lease);
    assert(!status.ok());
    assert(status.error_code() == grpc::StatusCode::UNAUTHENTICATED);

    otasim::HealthResponse health;
    grpc::ClientContext health_context;
    addToken(health_context, "alice_token");
    status = stub->Health(&health_context, otasim::HealthRequest{}, &health);
    assert(status.ok());
    assert(health.ok());

    otasim::CreateSessionRequest create_request;
    create_request.set_session_id("field");
    create_request.set_display_name("Field test");
    create_request.set_channel_model("awgn");
    create_request.set_snr_db(15.0);
    create_request.set_seed(7);
    create_request.set_station_cap(4);
    otasim::SessionInfo created;
    grpc::ClientContext create_context;
    addToken(create_context, "alice_token");
    status = stub->CreateSession(&create_context, create_request, &created);
    assert(status.ok());
    assert(created.session_id() == "field");

    otasim::JoinSessionRequest join_request;
    join_request.set_session_id("field");
    join_request.set_station_id("ALPHA");
    otasim::JoinSessionResponse join_response;
    grpc::ClientContext join_context;
    addToken(join_context, "alice_token");
    status = stub->JoinSession(&join_context, join_request, &join_response);
    assert(status.ok());
    assert(join_response.session().session_id() == "field");

    otasim::ListSessionsResponse sessions;
    grpc::ClientContext list_context;
    addToken(list_context, "alice_token");
    status = stub->ListSessions(&list_context, otasim::ListSessionsRequest{}, &sessions);
    assert(status.ok());
    assert(hasSession(sessions, ultra::ota_channel_core::kLobbySessionId));
    assert(hasSession(sessions, "field"));

    otasim::LeaveSessionRequest leave_request;
    leave_request.set_session_id("field");
    leave_request.set_station_id("ALPHA");
    google::protobuf::Empty empty;
    grpc::ClientContext leave_context;
    addToken(leave_context, "alice_token");
    status = stub->LeaveSession(&leave_context, leave_request, &empty);
    assert(status.ok());

    otasim::SetChannelRequest set_channel;
    set_channel.set_session_id(ultra::ota_channel_core::kLobbySessionId);
    set_channel.set_model("good");
    set_channel.set_snr_db(12.0);
    set_channel.set_seed(99);
    otasim::CommandAck ack;
    grpc::ClientContext set_channel_context;
    addToken(set_channel_context, "alice_token");
    status = stub->SetChannel(&set_channel_context, set_channel, &ack);
    assert(status.ok());
    assert(ack.accepted());

    service.beginDraining();
    otasim::HealthResponse draining_health;
    grpc::ClientContext draining_context;
    addToken(draining_context, "alice_token");
    status = stub->Health(&draining_context, otasim::HealthRequest{}, &draining_health);
    assert(status.ok());
    assert(!draining_health.ok());

    server->Shutdown();
    server->Wait();
    service.shutdown();

    std::cout << "grpc service smoke covered auth, sessions, channel, health, shutdown\n";
    return 0;
}
