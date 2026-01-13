#include "ultra/fec.hpp"
#include <stdexcept>
#include <bitset>
#include <random>

namespace ultra {

/**
 * LDPC Code Implementation
 *
 * Using quasi-cyclic LDPC codes similar to those in IEEE 802.11n/ac
 * These have efficient encoder/decoder implementations and excellent performance.
 *
 * Code structure:
 * - Block length: 648 bits (81 bytes) - good for HF frame sizes
 * - Code rates: 1/2, 2/3, 3/4, 5/6
 * - Designed for iterative belief-propagation decoding
 *
 * H matrix structure: H = [H_data | I]
 * - H_data: m x k sparse matrix with irregular LDPC structure
 * - I: m x m identity matrix for easy systematic encoding
 * - This allows: parity = H_data * info_bits (mod 2)
 */

namespace {

constexpr int SUBBLOCK_SIZE = 27;  // Circulant size
constexpr int BLOCK_COLS = 24;     // Number of column blocks
constexpr int BLOCK_LENGTH = SUBBLOCK_SIZE * BLOCK_COLS;  // 648 bits

struct CodeParams {
    int info_bits;
    int parity_bits;
    int num_check_rows;
};

CodeParams getCodeParams(CodeRate rate) {
    switch (rate) {
        case CodeRate::R1_2:
            return {324, 324, 324};
        case CodeRate::R2_3:
            return {432, 216, 216};
        case CodeRate::R3_4:
            return {486, 162, 162};
        case CodeRate::R5_6:
            return {540, 108, 108};
        default:
            return {324, 324, 324};
    }
}

} // anonymous namespace

struct LDPCEncoder::Impl {
    CodeRate rate;
    CodeParams params;

    // Sparse parity check matrix representation
    // For each check node: list of connected variable nodes (info bits only)
    // H = [H_data | I], so we only store H_data connections
    std::vector<std::vector<int>> H_data_rows;

    Impl(CodeRate r) : rate(r), params(getCodeParams(r)) {
        buildMatrix();
    }

    void buildMatrix() {
        // Build H_data: m x k sparse matrix
        // Using PEG-like construction for good distance properties

        int k = params.info_bits;
        int m = params.parity_bits;

        std::mt19937 rng(0x12345678 + static_cast<int>(rate));

        H_data_rows.clear();
        H_data_rows.resize(m);

        // Target: each info bit connects to ~3 checks (variable degree)
        // Each check connects to ~(3*k/m) info bits (check degree)
        int target_var_degree = 3;

        // Track how many connections each check has
        std::vector<int> check_degrees(m, 0);
        int max_check_degree = (target_var_degree * k) / m + 4;  // Allow some variance

        for (int j = 0; j < k; ++j) {
            // Find checks with room for more connections
            std::vector<int> available_checks;
            for (int i = 0; i < m; ++i) {
                if (check_degrees[i] < max_check_degree) {
                    available_checks.push_back(i);
                }
            }

            // Connect this info bit to target_var_degree checks
            std::shuffle(available_checks.begin(), available_checks.end(), rng);
            int connections = std::min(target_var_degree, static_cast<int>(available_checks.size()));

            for (int d = 0; d < connections; ++d) {
                int check = available_checks[d];
                H_data_rows[check].push_back(j);
                check_degrees[check]++;
            }
        }

        // Ensure every check has at least one connection
        for (int i = 0; i < m; ++i) {
            if (H_data_rows[i].empty()) {
                // Connect to a random info bit
                int j = rng() % k;
                H_data_rows[i].push_back(j);
            }
        }
    }

    Bytes encodeSystematic(ByteSpan data) {
        // Systematic encoding: output = [info | parity]
        // With H = [H_data | I], we have:
        // H * [info | parity]^T = 0
        // H_data * info + I * parity = 0
        // parity = H_data * info (mod 2)

        int k = params.info_bits;
        int m = params.parity_bits;
        int n = k + m;

        // Convert input bytes to bits
        std::vector<uint8_t> info_bits(k, 0);
        size_t bit_idx = 0;
        for (size_t i = 0; i < data.size() && bit_idx < static_cast<size_t>(k); ++i) {
            for (int b = 7; b >= 0 && bit_idx < static_cast<size_t>(k); --b) {
                info_bits[bit_idx++] = (data[i] >> b) & 1;
            }
        }

        // Calculate parity bits: parity[i] = XOR of info bits connected to check i
        std::vector<uint8_t> parity(m, 0);
        for (int i = 0; i < m; ++i) {
            uint8_t sum = 0;
            for (int j : H_data_rows[i]) {
                sum ^= info_bits[j];
            }
            parity[i] = sum;
        }

        // Combine info + parity into codeword
        std::vector<uint8_t> codeword(n);
        std::copy(info_bits.begin(), info_bits.end(), codeword.begin());
        std::copy(parity.begin(), parity.end(), codeword.begin() + k);

        // Convert bits to bytes
        Bytes output;
        output.reserve((n + 7) / 8);
        uint8_t byte = 0;
        int bit_count = 0;
        for (uint8_t bit : codeword) {
            byte = (byte << 1) | bit;
            ++bit_count;
            if (bit_count == 8) {
                output.push_back(byte);
                byte = 0;
                bit_count = 0;
            }
        }
        if (bit_count > 0) {
            output.push_back(byte << (8 - bit_count));
        }

        return output;
    }
};

LDPCEncoder::LDPCEncoder(CodeRate rate)
    : impl_(std::make_unique<Impl>(rate)) {}

LDPCEncoder::~LDPCEncoder() = default;

Bytes LDPCEncoder::encode(ByteSpan data) {
    return impl_->encodeSystematic(data);
}

size_t LDPCEncoder::getCodedSize(size_t input_size) const {
    int k = impl_->params.info_bits;
    int n = k + impl_->params.parity_bits;

    size_t input_bits = input_size * 8;
    size_t num_blocks = (input_bits + k - 1) / k;
    size_t output_bits = num_blocks * n;

    return (output_bits + 7) / 8;
}

CodeRate LDPCEncoder::getRate() const {
    return impl_->rate;
}

void LDPCEncoder::setRate(CodeRate rate) {
    impl_->rate = rate;
    impl_->params = getCodeParams(rate);
    impl_->buildMatrix();
}

} // namespace ultra
