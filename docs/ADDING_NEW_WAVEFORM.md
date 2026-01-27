# Adding a New Waveform

**Purpose:** Step-by-step guide for implementing new waveforms (e.g., OTFS, AFDM).

---

## Overview

Adding a new waveform requires:
1. Implement `IWaveform` interface
2. Register in `WaveformFactory`
3. Add to `WaveformMode` enum (if needed)
4. Update test tools
5. Update documentation

**Time estimate:** 1-3 days depending on complexity

---

## Step 1: Understand IWaveform Interface

**Location:** `src/waveform/waveform_interface.hpp`

```cpp
class IWaveform {
public:
    virtual ~IWaveform() = default;

    // === RX: Synchronization ===
    // Find preamble, estimate CFO, return sample position
    virtual bool detectSync(SampleSpan samples, SyncResult& result,
                           float threshold = 0.15f) = 0;

    // === RX: Demodulation ===
    // Process samples starting at training position
    // Returns true when soft bits are ready
    virtual bool process(SampleSpan samples) = 0;

    // Get demodulated soft bits (call after process() returns true)
    virtual std::vector<float> getSoftBits() = 0;

    // Check if more data pending
    virtual bool hasPendingData() const = 0;

    // === TX: Modulation ===
    // Generate preamble (chirp, Schmidl-Cox, etc.)
    virtual Samples generatePreamble() = 0;

    // Modulate encoded data to audio
    virtual Samples modulate(const Bytes& encoded_data) = 0;

    // === Configuration ===
    virtual void setFrequencyOffset(float cfo_hz) = 0;
    virtual void reset() = 0;

    // === Status ===
    virtual bool isSynced() const = 0;
    virtual float getEstimatedSNR() const = 0;
    virtual float getEstimatedCFO() const = 0;
};
```

### SyncResult Structure
```cpp
struct SyncResult {
    bool success = false;
    size_t start_sample = 0;   // Position of TRAINING START (not data!)
    float cfo_hz = 0.0f;       // Estimated CFO in Hz
    float correlation = 0.0f;  // Sync correlation strength
    float snr_estimate = 0.0f; // SNR estimate (if available)
};
```

---

## Step 2: Create Waveform Implementation

### File Structure
```
src/waveform/
├── my_waveform.hpp
└── my_waveform.cpp
```

### Header Template (`my_waveform.hpp`)
```cpp
#pragma once
#include "waveform_interface.hpp"
#include <memory>

namespace ultra {

class MyModulator;    // Forward declare
class MyDemodulator;
class MySyncMethod;

class MyWaveform : public IWaveform {
public:
    explicit MyWaveform(int some_param = 8);
    ~MyWaveform() override;

    // IWaveform interface
    bool detectSync(SampleSpan samples, SyncResult& result,
                   float threshold = 0.15f) override;
    bool process(SampleSpan samples) override;
    std::vector<float> getSoftBits() override;
    bool hasPendingData() const override;
    Samples generatePreamble() override;
    Samples modulate(const Bytes& encoded_data) override;
    void setFrequencyOffset(float cfo_hz) override;
    void reset() override;
    bool isSynced() const override;
    float getEstimatedSNR() const override;
    float getEstimatedCFO() const override;

private:
    std::unique_ptr<MyModulator> modulator_;
    std::unique_ptr<MyDemodulator> demodulator_;
    std::unique_ptr<MySyncMethod> sync_;

    float cfo_hz_ = 0.0f;
    bool synced_ = false;
};

} // namespace ultra
```

### Implementation Template (`my_waveform.cpp`)
```cpp
#include "my_waveform.hpp"
#include "my_modulator.hpp"      // Your modulator
#include "my_demodulator.hpp"    // Your demodulator
#include "sync/chirp_sync.hpp"   // Or your sync method

namespace ultra {

MyWaveform::MyWaveform(int some_param) {
    // Initialize components
    modulator_ = std::make_unique<MyModulator>(some_param);
    demodulator_ = std::make_unique<MyDemodulator>(some_param);

    // Use existing chirp sync or implement your own
    sync::ChirpConfig chirp_cfg;
    chirp_cfg.sample_rate = 48000;
    chirp_cfg.f_start = 300.0f;
    chirp_cfg.f_end = 2700.0f;
    chirp_cfg.duration_ms = 500.0f;
    sync_ = std::make_unique<sync::ChirpSync>(chirp_cfg);
}

MyWaveform::~MyWaveform() = default;

bool MyWaveform::detectSync(SampleSpan samples, SyncResult& result,
                            float threshold) {
    // Use chirp detection (or your own sync)
    auto chirp_result = sync_->detectDualChirp(samples, threshold);

    if (!chirp_result.success) {
        result.success = false;
        return false;
    }

    // Calculate position of TRAINING START (after preamble)
    size_t chirp_samples = sync_->getChirpSamples();  // 24000
    size_t gap_samples = 4800;  // 100ms gap

    // CRITICAL: Return TRAINING position, not data position!
    result.start_sample = chirp_result.up_chirp_start +
                          2 * chirp_samples + 2 * gap_samples;
    result.cfo_hz = chirp_result.cfo_hz;
    result.correlation = chirp_result.correlation;
    result.success = true;

    return true;
}

void MyWaveform::setFrequencyOffset(float cfo_hz) {
    cfo_hz_ = cfo_hz;
    // Pass to demodulator
    demodulator_->setFrequencyOffset(cfo_hz);
}

bool MyWaveform::process(SampleSpan samples) {
    // IMPORTANT: Apply CFO correction BEFORE demodulation
    // The demodulator should handle this internally

    // Demodulate
    return demodulator_->process(samples);
}

std::vector<float> MyWaveform::getSoftBits() {
    return demodulator_->getSoftBits();
}

bool MyWaveform::hasPendingData() const {
    return demodulator_->hasPendingData();
}

Samples MyWaveform::generatePreamble() {
    // Generate preamble (dual chirp + training)
    auto chirp = sync_->generateDualChirp();

    // Add training symbols specific to your waveform
    auto training = modulator_->generateTraining();

    // Concatenate
    Samples result;
    result.reserve(chirp.size() + training.size());
    result.insert(result.end(), chirp.begin(), chirp.end());
    result.insert(result.end(), training.begin(), training.end());

    return result;
}

Samples MyWaveform::modulate(const Bytes& encoded_data) {
    return modulator_->modulate(encoded_data);
}

void MyWaveform::reset() {
    demodulator_->reset();
    synced_ = false;
    cfo_hz_ = 0.0f;  // IMPORTANT: Clear CFO on reset!
}

bool MyWaveform::isSynced() const {
    return synced_;
}

float MyWaveform::getEstimatedSNR() const {
    return demodulator_->getEstimatedSNR();
}

float MyWaveform::getEstimatedCFO() const {
    return cfo_hz_;
}

} // namespace ultra
```

---

## Step 3: Register in WaveformFactory

**File:** `src/waveform/waveform_factory.cpp`

```cpp
#include "waveform_factory.hpp"
#include "mc_dpsk_waveform.hpp"
#include "ofdm_chirp_waveform.hpp"
#include "ofdm_cox_waveform.hpp"
#include "my_waveform.hpp"  // ADD THIS

namespace ultra {

std::unique_ptr<IWaveform> WaveformFactory::create(protocol::WaveformMode mode) {
    switch (mode) {
        case protocol::WaveformMode::MC_DPSK:
            return std::make_unique<MCDPSKWaveform>(8);

        case protocol::WaveformMode::OFDM_CHIRP:
            return std::make_unique<OFDMChirpWaveform>();

        case protocol::WaveformMode::OFDM_COX:
            return std::make_unique<OFDMCoxWaveform>();

        // ADD YOUR WAVEFORM HERE
        case protocol::WaveformMode::MY_WAVEFORM:
            return std::make_unique<MyWaveform>(8);

        default:
            return nullptr;
    }
}

} // namespace ultra
```

---

## Step 4: Add to WaveformMode Enum

**File:** `src/protocol/frame_v2.hpp`

```cpp
enum class WaveformMode : uint8_t {
    OFDM_COX = 0x00,
    OTFS_EQ = 0x01,
    OTFS_RAW = 0x02,
    MFSK = 0x03,
    MC_DPSK = 0x04,
    OFDM_CHIRP = 0x05,
    MY_WAVEFORM = 0x06,  // ADD THIS (use next available ID)
    AUTO = 0xFF,
};
```

Also update `waveformModeToString()` if it exists.

---

## Step 5: Add to CMakeLists.txt

```cmake
add_library(ultra_core STATIC
    # ... existing files ...
    src/waveform/my_waveform.cpp
)
```

---

## Step 6: Add to Test Tools

### Add to test_iwaveform.cpp

```cpp
// In waveform selection logic:
if (waveform_name == "my_waveform") {
    waveform = std::make_unique<MyWaveform>(8);
}
```

### Add to regression_matrix.sh

```bash
# My Waveform tests
run_test "MY_WAVEFORM AWGN SNR=10 CFO=0" \
    "$BUILD_DIR/test_iwaveform --snr 10 --cfo 0 --channel awgn -w my_waveform --frames 5" \
    100
```

---

## Step 7: Document in CLAUDE.md

Add to the waveform table and SNR thresholds.

---

## Critical Invariants to Follow

See `docs/INVARIANTS.md` for complete list. Key points:

1. **detectSync() returns TRAINING position, not data position**
   ```cpp
   // Frame layout: [PREAMBLE][TRAINING][DATA...]
   //                         ^-- start_sample here
   ```

2. **CFO must be applied BEFORE demodulation**
   ```cpp
   waveform->setFrequencyOffset(cfo_hz);  // THEN
   waveform->process(samples);             // NOT before setFrequencyOffset!
   ```

3. **reset() must clear CFO state**
   ```cpp
   void reset() {
       cfo_hz_ = 0.0f;  // MUST clear!
       // ...
   }
   ```

4. **Trust chirp CFO over training CFO**
   - Chirp uses 1+ second of signal
   - Training uses ~100ms
   - Don't let training overwrite chirp CFO

---

## Testing Your Waveform

### Basic Functionality
```bash
./test_iwaveform --snr 10 --cfo 0 --channel awgn -w my_waveform --frames 10
# Expected: 100% decode rate
```

### CFO Tolerance
```bash
./test_iwaveform --snr 10 --cfo 30 --channel awgn -w my_waveform --frames 10
./test_iwaveform --snr 10 --cfo -50 --channel awgn -w my_waveform --frames 10
# Expected: 90%+ decode rate
```

### Fading Channels
```bash
./test_iwaveform --snr 10 --cfo 30 --channel moderate -w my_waveform --frames 10
# Expected: 80%+ decode rate (depends on waveform design)
```

### Full Regression
```bash
./tests/regression_matrix.sh --full
# All tests must pass
```

---

## Example Implementations

Reference these existing implementations:

| File | Waveform | Sync Method | Notes |
|------|----------|-------------|-------|
| `mc_dpsk_waveform.cpp` | MC-DPSK | Dual chirp | Good example for chirp sync |
| `ofdm_chirp_waveform.cpp` | OFDM+DQPSK | Dual chirp | OFDM with chirp |
| `ofdm_cox_waveform.cpp` | OFDM | Schmidl-Cox | Different sync method |

---

## Checklist

- [ ] Create `my_waveform.hpp` and `my_waveform.cpp`
- [ ] Implement all IWaveform methods
- [ ] Follow position invariant (return TRAINING start)
- [ ] Follow CFO invariants (apply before process, clear on reset)
- [ ] Register in WaveformFactory
- [ ] Add WaveformMode enum value
- [ ] Add to CMakeLists.txt
- [ ] Add to test_iwaveform
- [ ] Add to regression_matrix.sh
- [ ] Test with CFO = 0, ±30, ±50 Hz
- [ ] Test on AWGN and fading channels
- [ ] Update CLAUDE.md
- [ ] Add entry to CHANGELOG.md
