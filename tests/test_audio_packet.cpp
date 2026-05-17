#include "ota_simulator_service/audio_packet.hpp"

#include <cassert>
#include <cmath>
#include <iostream>

using namespace ultra::ota_simulator_service;

namespace {

void assertNear(float a, float b) {
    assert(std::abs(a - b) < 1.0e-6f);
}

}  // namespace

int main() {
    OtaAudioPacket packet;
    packet.header.lease_id = 0x1122334455667788ull;
    packet.header.seq = 17;
    packet.header.start_sample = 480;
    packet.samples = {0.25f, -0.5f, 0.75f};

    const auto bytes = serializeAudioPacket(packet);
    assert(bytes.size() == kOtaAudioHeaderBytes + packet.samples.size() * sizeof(float));
    assert(bytes[0] == 'O');
    assert(bytes[1] == 'A');
    assert(bytes[2] == 'S');
    assert(bytes[3] == 'M');

    auto parsed = parseAudioPacket(bytes);
    assert(parsed);
    assert(parsed->header.magic == kOtaAudioMagic);
    assert(parsed->header.version == kOtaAudioVersion);
    assert(parsed->header.lease_id == packet.header.lease_id);
    assert(parsed->header.seq == 17);
    assert(parsed->header.start_sample == 480);
    assert(parsed->header.sample_count == 3);
    assert(parsed->samples.size() == 3);
    assertNear(parsed->samples[0], 0.25f);
    assertNear(parsed->samples[1], -0.5f);
    assertNear(parsed->samples[2], 0.75f);

    auto bad_magic = bytes;
    bad_magic[0] = 0;
    AudioPacketError error;
    assert(!parseAudioPacket(bad_magic, &error));
    assert(error.message.find("magic") != std::string::npos);

    auto bad_version = bytes;
    bad_version[4] = 2;
    assert(!parseAudioPacket(bad_version, &error));
    assert(error.message.find("version") != std::string::npos);

    std::cout << "audio packet header and payload serialization deterministic\n";
    return 0;
}
