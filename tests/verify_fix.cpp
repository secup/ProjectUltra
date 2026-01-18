/**
 * Quick verification of the correlation fix
 */
#include <iostream>
#include <vector>
#include <iomanip>
#include "ultra/ofdm.hpp"
#include "ultra/types.hpp"

using namespace ultra;

int main() {
    ModemConfig config;
    config.modulation = Modulation::BPSK;

    OFDMModulator mod(config);
    OFDMDemodulator demod(config);

    Samples preamble = mod.generatePreamble();
    Bytes test_data(81, 0x00);
    Samples data = mod.modulate(test_data, config.modulation);

    size_t preamble_start = 1000;

    std::vector<float> signal(preamble_start, 0.0f);
    signal.insert(signal.end(), preamble.begin(), preamble.end());
    signal.insert(signal.end(), data.begin(), data.end());

    float max_val = 0;
    for (float s : signal) max_val = std::max(max_val, std::abs(s));
    for (float& s : signal) s *= 0.5f / max_val;

    // Feed to demodulator
    SampleSpan span(signal.data(), signal.size());
    demod.process(span);

    if (demod.isSynced()) {
        size_t detected = demod.getLastSyncOffset();
        int error = static_cast<int>(detected) - static_cast<int>(preamble_start);
        std::cout << "True start: " << preamble_start << std::endl;
        std::cout << "Detected: " << detected << std::endl;
        std::cout << "Error: " << error << " samples" << std::endl;
        std::cout << "Result: " << (std::abs(error) <= 16 ? "PASS" : "FAIL") << std::endl;
    } else {
        std::cout << "NO SYNC FOUND" << std::endl;
    }

    return 0;
}
