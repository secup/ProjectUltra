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
    
    // Reset and test shuffle
    rng.seed(0x12345678);
    std::vector<int> v = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
    std::shuffle(v.begin(), v.end(), rng);
    
    std::cout << "\nAfter shuffle: ";
    for (int x : v) std::cout << x << " ";
    std::cout << "\n";
    
    return 0;
}
