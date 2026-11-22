// ack_noise_h0: the tone-ACK detector on NOISE ONLY (H0). Production ToneBurstDetector, one
// staircase symbol duration and one carrier-offset search width per run. Each window is
// decoded once at a LOW candidate floor; every CRC-12-passing decode is printed with its
// candidate's noise ratio z (it would also pass at any floor <= z: candidates are tried
// strongest first and only above-floor ones count against the budget) and its payload, so the
// false-ACK rate can be read off for any floor >= the run's floor and any sender state.
//
// Usage: ack_noise_h0 SYMBOL_MS HALFWIDTH_HZ WINDOWS SEED [FLOOR=1.4]
// Output: "W ms hw idx zmax"           one per window (strongest candidate ratio)
//         "P ms hw idx z type seq mask epoch rung rate drive"  one per CRC pass
#include "waveform/tone_burst_ack/tone_burst_constants.hpp"
#include "waveform/tone_burst_ack/tone_burst_detector.hpp"

#include <cstdio>
#include <cstdlib>
#include <random>
#include <vector>

using namespace ultra::waveform::tone_burst_ack;

int main(int argc, char** argv) {
    if (argc < 5) {
        std::fprintf(stderr, "usage: %s SYMBOL_MS HALFWIDTH_HZ WINDOWS SEED [FLOOR]\n", argv[0]);
        return 2;
    }
    const uint32_t ms = static_cast<uint32_t>(std::atoi(argv[1]));
    const float halfwidth = static_cast<float>(std::atof(argv[2]));
    const int windows = std::atoi(argv[3]);
    const unsigned seed = static_cast<unsigned>(std::atoi(argv[4]));
    const float floor = argc > 5 ? static_cast<float>(std::atof(argv[5])) : 1.4f;

    std::mt19937 rng(seed);
    std::normal_distribution<float> n01(0.0f, 1.0f);
    const uint32_t spp = (48000u * ms) / 1000u;
    // The monitor's window: the burst plus a quarter guard plus 200 ms.
    const size_t n = static_cast<size_t>(kTotalSymbols) * spp * 5 / 4 + 9600;
    std::vector<float> x(n);
    for (int r = 0; r < windows; ++r) {
        for (float& v : x) v = 0.05f * n01(rng);
        ToneBurstDetector det;
        det.setFrequencyPrior(0.0f, halfwidth);
        det.setNoiseRatioFloor(floor);
        const auto res = det.detectAndDecode(x.data(), x.size(), ms, 32, {});
        std::printf("W %u %.1f %d %.4f\n", ms, halfwidth, r, res.strongest_candidate_noise_ratio);
        if (res.decode.stats.crc_ok && res.decode.payload) {
            const auto& p = *res.decode.payload;
            std::printf("P %u %.1f %d %.4f %d %u %u %u %u %u %u\n", ms, halfwidth, r,
                        res.costas_noise_ratio, static_cast<int>(p.type), p.group_seq,
                        p.frame_mask, p.move_epoch, p.rung_cmd, p.rate_hint, p.drive_advisory);
        }
        std::fflush(stdout);
    }
    return 0;
}
