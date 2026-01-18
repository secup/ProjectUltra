#include <iostream>
#include <random>
#include <vector>
#include <algorithm>

int main() {
    // Same seed as LDPC encoder for R1/4
    std::mt19937 rng(0x12345678 + 0);  // R1_4 = 0 in enum

    std::cout << "First 10 random numbers from mt19937(0x12345678):\n";
    for (int i = 0; i < 10; i++) {
        std::cout << "  " << rng() << "\n";
    }

    // Reset and test std::shuffle (BROKEN - implementation-defined!)
    rng.seed(0x12345678);
    std::vector<int> v = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
    std::shuffle(v.begin(), v.end(), rng);

    std::cout << "\nstd::shuffle (BROKEN - differs by platform): ";
    for (int x : v) std::cout << x << " ";
    std::cout << "\n";

    // Reset and test Fisher-Yates shuffle (FIXED - deterministic!)
    rng.seed(0x12345678);
    std::vector<int> v2 = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};

    // Fisher-Yates shuffle using direct RNG calls
    for (size_t i = v2.size(); i > 1; --i) {
        size_t j = rng() % i;
        std::swap(v2[i - 1], v2[j]);
    }

    std::cout << "Fisher-Yates (FIXED - same on all platforms): ";
    for (int x : v2) std::cout << x << " ";
    std::cout << "\n";

    // Expected result (must match on both Mac and Linux):
    std::cout << "\nExpected Fisher-Yates result: 7 6 0 8 5 1 2 4 9 3\n";

    return 0;
}
