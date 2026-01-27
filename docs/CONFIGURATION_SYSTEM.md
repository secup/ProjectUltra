# Configuration System

**Purpose:** Document all configuration structures and how settings flow through the system.

---

## Configuration Layers

| Layer | Scope | Persistence | Location |
|-------|-------|-------------|----------|
| AppSettings | User preferences | INI file | `~/.config/ultra/settings.ini` |
| ModemConfig | DSP parameters | Memory (presets) | Runtime |
| FilterConfig | Audio filtering | Part of AppSettings | INI file |
| WaveformMode | Protocol mode | Negotiated | Per-connection |

---

## AppSettings (User Preferences)

**File:** `src/gui/widgets/settings.hpp`

**Storage:** INI format at platform-specific locations:
- Linux: `~/.config/ultra/settings.ini`
- Windows: `%APPDATA%\ProjectUltra\settings.ini`
- macOS: `~/.config/ultra/settings.ini`

### Station Information
```cpp
char callsign[16] = "N0CALL";     // Required for ARQ
char grid_square[8] = "";          // Maidenhead locator
char name[32] = "";                // Operator name
```

### Radio Control (Future Hamlib)
```cpp
char rig_model[32] = "None";
char rig_port[64] = "";
int rig_baud = 9600;
bool use_cat_ptt = false;
```

### Audio Configuration
```cpp
char input_device[128] = "Default";
char output_device[128] = "Default";
int tx_delay_ms = 50;              // PTT → TX delay
int tx_tail_ms = 50;               // TX → PTT release
float tx_drive = 0.8f;             // TX level (0.0-1.0)
```

### Filter Settings
```cpp
bool filter_enabled = false;
float filter_center = 1500.0f;     // Hz
float filter_bandwidth = 2900.0f;  // Hz (covers 2.8 kHz modem)
int filter_taps = 101;             // FIR taps
```

### Expert Mode
```cpp
uint8_t forced_waveform = 0xFF;    // 0xFF = AUTO
uint8_t forced_modulation = 0xFF;  // 0xFF = AUTO
uint8_t forced_code_rate = 0xFF;   // 0xFF = AUTO
```

### INI File Format
```ini
[Station]
callsign=N0CALL
grid_square=FN35
name=John Doe

[Audio]
input_device=Default
output_device=Default
tx_delay_ms=50
tx_drive=0.8

[Filter]
enabled=0
center=1500.0
bandwidth=2900.0
taps=101

[Expert]
forced_waveform=255
forced_modulation=255
forced_code_rate=255
```

---

## ModemConfig (DSP Parameters)

**File:** `include/ultra/types.hpp`

### Core Parameters
```cpp
struct ModemConfig {
    uint32_t sample_rate = 48000;
    uint32_t center_freq = 1500;
    uint32_t fft_size = 512;         // or 1024 for NVIS
    uint32_t num_carriers = 30;      // or 59 for NVIS
    CyclicPrefixMode cp_mode;        // SHORT/MEDIUM/LONG
    uint32_t symbol_guard = 4;
};
```

### Modulation Settings
```cpp
Modulation modulation = Modulation::QPSK;
CodeRate code_rate = CodeRate::R1_2;
bool use_pilots = true;              // false for differential modes
uint32_t pilot_spacing = 2;
```

### Helper Methods
```cpp
uint32_t getCyclicPrefix() const;
uint32_t getSymbolDuration() const;
float getSymbolRate() const;
uint32_t getDataCarriers() const;
float getTheoreticalThroughput(Modulation mod, CodeRate rate) const;
```

---

## Presets (Speed Profiles)

**Location:** `include/ultra/types.hpp` namespace `presets`

### Conservative (Poor HF)
```cpp
cp_mode = CyclicPrefixMode::LONG;   // 64 samples
symbol_guard = 8;
pilot_spacing = 2;
modulation = Modulation::QPSK;
code_rate = CodeRate::R1_2;
```

### Balanced (Typical HF) - DEFAULT
```cpp
cp_mode = CyclicPrefixMode::MEDIUM; // 48 samples
symbol_guard = 4;
pilot_spacing = 2;
modulation = Modulation::QAM64;
code_rate = CodeRate::R3_4;
```

### Turbo (Excellent Conditions)
```cpp
cp_mode = CyclicPrefixMode::SHORT;  // 32 samples
symbol_guard = 0;
pilot_spacing = 2;
modulation = Modulation::QAM256;
code_rate = CodeRate::R5_6;
```

### NVIS Mode (Stable Paths)
```cpp
fft_size = 1024;
num_carriers = 59;
use_pilots = false;                  // All carriers = data
modulation = Modulation::DQPSK;
code_rate = CodeRate::R3_4;
```

---

## Enumerations

### Modulation
```cpp
enum class Modulation : uint8_t {
    DBPSK = 0,    // 1 bit/symbol, differential
    BPSK = 1,     // 1 bit/symbol, coherent
    DQPSK = 2,    // 2 bits/symbol, differential
    QPSK = 3,     // 2 bits/symbol, coherent
    D8PSK = 4,    // 3 bits/symbol, differential
    QAM16 = 6,    // 4 bits/symbol
    QAM32 = 7,    // 5 bits/symbol
    QAM64 = 8,    // 6 bits/symbol
    QAM256 = 10,  // 8 bits/symbol (needs 30+ dB)
    AUTO = 0xFF,
};
```

### CodeRate
```cpp
enum class CodeRate : uint8_t {
    R1_4,   // 162 info bits / 648 codeword
    R1_3,   // 216 info bits
    R1_2,   // 324 info bits
    R2_3,   // 432 info bits
    R3_4,   // 486 info bits
    R5_6,   // 540 info bits
    AUTO = 0xFF,
};
```

### WaveformMode
```cpp
enum class WaveformMode : uint8_t {
    OFDM_COX = 0x00,    // Schmidl-Cox sync (17+ dB)
    OTFS_EQ = 0x01,     // OTFS with equalization
    MC_DPSK = 0x04,     // Multi-carrier DPSK (-3 to 10 dB)
    OFDM_CHIRP = 0x05,  // Chirp + DQPSK (10-17 dB)
    AUTO = 0xFF,
};
```

---

## Configuration Flow

### Startup
```
1. App::App() constructor
   ├─ settings_.load()                        // Load INI
   ├─ protocol_.setLocalCallsign(callsign)
   ├─ protocol_.setPreferredMode(forced_waveform)
   ├─ protocol_.setForcedModulation(forced_modulation)
   ├─ protocol_.setForcedCodeRate(forced_code_rate)
   └─ modem_.setFilterConfig(filter)

2. ModemEngine::ModemEngine()
   ├─ config_ = presets::balanced()           // Default preset
   ├─ encoder_ = LDPCEncoder(code_rate)
   └─ Create modulators/demodulators
```

### Settings Window → System
```
User changes setting in UI
    ↓
SettingsWindow callback fires
    ↓
├─ on_callsign_changed_    → protocol_.setLocalCallsign()
├─ on_filter_changed_      → modem_.setFilterConfig()
├─ on_expert_settings_     → protocol_.setForcedModulation/CodeRate()
    ↓
settings_.save()           // Persist to INI
```

### Runtime Mode Changes
```
Protocol negotiates mode
    ↓
DataModeChangedCallback(mod, rate, snr)
    ↓
ModemEngine::setDataMode(mod, rate)
    ↓
├─ Update config_.modulation/code_rate
├─ Set use_pilots = !is_differential
├─ Recreate encoder with new rate
└─ Recreate OFDM modulator/demodulator
```

---

## Expert Mode Override

When expert settings are not AUTO (0xFF):

```cpp
// Protocol uses these in CONNECT negotiation:
// - If 0xFF: responder chooses based on SNR
// - Otherwise: force specific mode

protocol_.setForcedModulation(settings_.forced_modulation);
protocol_.setForcedCodeRate(settings_.forced_code_rate);
protocol_.setPreferredMode(settings_.forced_waveform);
```

---

## Decision Points

| Decision | What Controls It | Default |
|----------|------------------|---------|
| Waveform mode | `forced_waveform` + SNR threshold | AUTO |
| Modulation | `forced_modulation` or CONNECT negotiation | AUTO |
| Code rate | `forced_code_rate` or CONNECT negotiation | AUTO |
| Use pilots | Automatic (false for DQPSK/D8PSK) | true |
| Cyclic prefix | Preset-dependent | MEDIUM |
| FFT size | Preset-dependent | 512 |

---

## Adding New Settings

1. **Add to AppSettings:**
   ```cpp
   // In settings.hpp
   int my_new_setting = 0;
   ```

2. **Add to load/save:**
   ```cpp
   // In settings.cpp load()
   if (key == "my_new_setting") my_new_setting = std::atoi(value.c_str());

   // In settings.cpp save()
   file << "my_new_setting=" << my_new_setting << "\n";
   ```

3. **Add to SettingsWindow UI:**
   ```cpp
   // In settings.cpp render()
   ImGui::InputInt("My Setting", &settings->my_new_setting);
   ```

4. **Add callback if needed:**
   ```cpp
   // In settings.hpp
   std::function<void(int)> on_my_setting_changed_;

   // In app.cpp
   settings_window_.setMySettingChangedCallback([this](int val) {
       // Apply to system
   });
   ```
