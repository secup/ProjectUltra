#include "ultra/fec.hpp"
#include <cmath>
#include <algorithm>
#include <limits>
#include <random>

namespace ultra {

namespace {

constexpr int SUBBLOCK_SIZE = 27;
constexpr int BLOCK_COLS = 24;
constexpr int BLOCK_LENGTH = SUBBLOCK_SIZE * BLOCK_COLS;

struct CodeParams {
    int info_bits;
    int parity_bits;
    int num_check_rows;
};

CodeParams getCodeParams(CodeRate rate) {
    switch (rate) {
        case CodeRate::R1_4:
            return {162, 486, 486};  // Most robust - for very poor conditions
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

struct LDPCDecoder::Impl {
    CodeRate rate;
    CodeParams params;
    int max_iterations = 50;
    bool last_success = false;
    int last_iters = 0;

    // Full H matrix representation: H = [H_data | I]
    // H_rows[i] contains all variable node indices connected to check i
    std::vector<std::vector<int>> H_rows;

    // Column view: for each variable node, which checks it connects to
    std::vector<std::vector<int>> H_cols;

    // Message passing buffers
    std::vector<std::vector<float>> var_to_check;  // var_to_check[i][e] = message from var H_rows[i][e] to check i
    std::vector<std::vector<float>> check_to_var;  // check_to_var[i][e] = message from check i to var H_rows[i][e]
    std::vector<float> llr_in;        // Channel LLRs
    std::vector<float> llr_total;     // Total LLRs

    Impl(CodeRate r) : rate(r), params(getCodeParams(r)) {
        buildMatrix();
    }

    void buildMatrix() {
        // Build H = [H_data | I]
        // Must match encoder exactly!

        int k = params.info_bits;
        int m = params.parity_bits;
        int n = k + m;

        std::mt19937 rng(0x12345678 + static_cast<int>(rate));

        H_rows.clear();
        H_rows.resize(m);
        H_cols.clear();
        H_cols.resize(n);

        // Build H_data part (same algorithm as encoder)
        // For low-rate codes (k < m), we need higher variable degree
        // to ensure each check has enough connections for good decoding
        int target_check_degree = 4;
        int target_var_degree = std::max(3, (target_check_degree * m) / k);
        target_var_degree = std::min(target_var_degree, m / 2);

        std::vector<int> check_degrees(m, 0);
        int max_check_degree = target_check_degree + 2;

        for (int j = 0; j < k; ++j) {
            std::vector<int> available_checks;
            for (int i = 0; i < m; ++i) {
                if (check_degrees[i] < max_check_degree) {
                    available_checks.push_back(i);
                }
            }

            std::shuffle(available_checks.begin(), available_checks.end(), rng);
            int connections = std::min(target_var_degree, static_cast<int>(available_checks.size()));

            for (int d = 0; d < connections; ++d) {
                int check = available_checks[d];
                H_rows[check].push_back(j);
                H_cols[j].push_back(check);
                check_degrees[check]++;
            }
        }

        // Ensure every check has at least one info bit connection
        for (int i = 0; i < m; ++i) {
            if (H_rows[i].empty()) {
                int j = rng() % k;
                H_rows[i].push_back(j);
                H_cols[j].push_back(i);
            }
        }

        // Add identity matrix part: parity bit (k+i) connects to check i
        for (int i = 0; i < m; ++i) {
            int parity_idx = k + i;
            H_rows[i].push_back(parity_idx);
            H_cols[parity_idx].push_back(i);
        }

        // Allocate message buffers
        var_to_check.resize(m);
        check_to_var.resize(m);
        for (int i = 0; i < m; ++i) {
            var_to_check[i].resize(H_rows[i].size(), 0);
            check_to_var[i].resize(H_rows[i].size(), 0);
        }
    }

    bool checkParity(const std::vector<uint8_t>& bits) {
        // Check if H * bits = 0
        for (size_t i = 0; i < H_rows.size(); ++i) {
            uint8_t sum = 0;
            for (int j : H_rows[i]) {
                if (j < static_cast<int>(bits.size())) {
                    sum ^= bits[j];
                }
            }
            if (sum != 0) return false;
        }
        return true;
    }

    Bytes decodeBP(std::span<const float> llrs) {
        // Belief Propagation (Min-Sum) decoding
        int n = params.info_bits + params.parity_bits;
        int k = params.info_bits;
        int m = params.parity_bits;

        // Initialize channel LLRs
        llr_in.assign(n, 0);
        llr_total.assign(n, 0);

        for (int j = 0; j < n && j < static_cast<int>(llrs.size()); ++j) {
            llr_in[j] = llrs[j];
            llr_total[j] = llrs[j];
        }

        // Initialize variable-to-check messages with channel LLRs
        for (int i = 0; i < m; ++i) {
            for (size_t e = 0; e < H_rows[i].size(); ++e) {
                int j = H_rows[i][e];
                var_to_check[i][e] = llr_in[j];
            }
            std::fill(check_to_var[i].begin(), check_to_var[i].end(), 0);
        }

        // Iterative decoding
        last_success = false;
        for (last_iters = 0; last_iters < max_iterations; ++last_iters) {
            // Check-to-variable messages (min-sum approximation)
            for (int i = 0; i < m; ++i) {
                const auto& row = H_rows[i];
                size_t degree = row.size();

                for (size_t e = 0; e < degree; ++e) {
                    // Compute product of signs and minimum of magnitudes, excluding edge e
                    float sign = 1.0f;
                    float min_abs = std::numeric_limits<float>::max();

                    for (size_t e2 = 0; e2 < degree; ++e2) {
                        if (e2 != e) {
                            float msg = var_to_check[i][e2];
                            if (msg < 0) sign = -sign;
                            float abs_msg = std::abs(msg);
                            if (abs_msg < min_abs) min_abs = abs_msg;
                        }
                    }

                    // Min-sum with scaling factor (0.75 improves performance)
                    check_to_var[i][e] = sign * min_abs * 0.75f;
                }
            }

            // Variable-to-check messages and total LLRs
            // First, compute total LLR at each variable node
            llr_total = llr_in;  // Start with channel LLR

            for (int i = 0; i < m; ++i) {
                for (size_t e = 0; e < H_rows[i].size(); ++e) {
                    int j = H_rows[i][e];
                    llr_total[j] += check_to_var[i][e];
                }
            }

            // Update var-to-check messages (total LLR minus incoming from that check)
            for (int i = 0; i < m; ++i) {
                for (size_t e = 0; e < H_rows[i].size(); ++e) {
                    int j = H_rows[i][e];
                    var_to_check[i][e] = llr_total[j] - check_to_var[i][e];

                    // Clamp to avoid numerical issues
                    var_to_check[i][e] = std::max(-50.0f, std::min(50.0f, var_to_check[i][e]));
                }
            }

            // Make hard decisions and check parity
            std::vector<uint8_t> hard_bits(n);
            for (int j = 0; j < n; ++j) {
                hard_bits[j] = (llr_total[j] < 0) ? 1 : 0;
            }

            if (checkParity(hard_bits)) {
                last_success = true;
                break;
            }
        }

        // Extract information bits and convert to bytes
        Bytes output;
        output.reserve((k + 7) / 8);

        uint8_t byte = 0;
        int bit_count = 0;
        for (int j = 0; j < k; ++j) {
            uint8_t bit = (llr_total[j] < 0) ? 1 : 0;
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

LDPCDecoder::LDPCDecoder(CodeRate rate)
    : impl_(std::make_unique<Impl>(rate)) {}

LDPCDecoder::~LDPCDecoder() = default;

Bytes LDPCDecoder::decode(ByteSpan coded_data) {
    // Convert bytes to LLRs (+value for 0, -value for 1)
    std::vector<float> llrs;
    llrs.reserve(coded_data.size() * 8);

    for (uint8_t byte : coded_data) {
        for (int b = 7; b >= 0; --b) {
            uint8_t bit = (byte >> b) & 1;
            // Use moderate LLR magnitude for hard decisions
            llrs.push_back(bit ? -6.0f : 6.0f);
        }
    }

    return impl_->decodeBP(llrs);
}

Bytes LDPCDecoder::decodeSoft(std::span<const float> llrs) {
    return impl_->decodeBP(llrs);
}

bool LDPCDecoder::lastDecodeSuccess() const {
    return impl_->last_success;
}

int LDPCDecoder::lastIterations() const {
    return impl_->last_iters;
}

void LDPCDecoder::setRate(CodeRate rate) {
    impl_->rate = rate;
    impl_->params = getCodeParams(rate);
    impl_->buildMatrix();
}

void LDPCDecoder::setMaxIterations(int max_iter) {
    impl_->max_iterations = max_iter;
}

// ============ Interleaver ============

Interleaver::Interleaver(size_t rows, size_t cols)
    : rows_(rows), cols_(cols) {
    // Create interleaving permutation (row-column transpose)
    size_t n = rows * cols;
    permutation_.resize(n);
    for (size_t i = 0; i < n; ++i) {
        size_t row = i / cols;
        size_t col = i % cols;
        permutation_[i] = col * rows + row;
    }
}

Bytes Interleaver::interleave(ByteSpan data) {
    size_t n = rows_ * cols_;
    size_t byte_size = (n + 7) / 8;

    // Convert to bits
    std::vector<uint8_t> bits(n, 0);
    for (size_t i = 0; i < data.size() && i * 8 < n; ++i) {
        for (int b = 0; b < 8 && i * 8 + b < n; ++b) {
            bits[i * 8 + b] = (data[i] >> (7 - b)) & 1;
        }
    }

    // Apply permutation
    std::vector<uint8_t> interleaved(n);
    for (size_t i = 0; i < n; ++i) {
        interleaved[permutation_[i]] = bits[i];
    }

    // Convert back to bytes
    Bytes output(byte_size, 0);
    for (size_t i = 0; i < n; ++i) {
        if (interleaved[i]) {
            output[i / 8] |= (1 << (7 - (i % 8)));
        }
    }

    return output;
}

Bytes Interleaver::deinterleave(ByteSpan data) {
    size_t n = rows_ * cols_;
    size_t byte_size = (n + 7) / 8;

    // Convert to bits
    std::vector<uint8_t> bits(n, 0);
    for (size_t i = 0; i < data.size() && i * 8 < n; ++i) {
        for (int b = 0; b < 8 && i * 8 + b < n; ++b) {
            bits[i * 8 + b] = (data[i] >> (7 - b)) & 1;
        }
    }

    // Apply inverse permutation
    std::vector<uint8_t> deinterleaved(n);
    for (size_t i = 0; i < n; ++i) {
        deinterleaved[i] = bits[permutation_[i]];
    }

    // Convert back to bytes
    Bytes output(byte_size, 0);
    for (size_t i = 0; i < n; ++i) {
        if (deinterleaved[i]) {
            output[i / 8] |= (1 << (7 - (i % 8)));
        }
    }

    return output;
}

std::vector<float> Interleaver::interleave(std::span<const float> soft_bits) {
    size_t n = soft_bits.size();
    std::vector<float> output(n);
    for (size_t i = 0; i < n && i < permutation_.size(); ++i) {
        output[permutation_[i]] = soft_bits[i];
    }
    return output;
}

std::vector<float> Interleaver::deinterleave(std::span<const float> soft_bits) {
    size_t n = soft_bits.size();
    std::vector<float> output(n);
    for (size_t i = 0; i < n && i < permutation_.size(); ++i) {
        output[i] = soft_bits[permutation_[i]];
    }
    return output;
}

} // namespace ultra
