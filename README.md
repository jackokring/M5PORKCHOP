# 🐷 M5Porkchop - ML-Enhanced Piglet Security Companion

```
  /   \
 (o  o)
  (__)
```

A tamagotchi-like security companion for the M5Cardputer, featuring:
- **OINK Mode**: Packet sniffing, network discovery, and handshake capture
- **WARHOG Mode**: GPS-enabled wardriving with export to Wigle/Kismet formats
- **ML-powered detection**: Heuristic + Edge Impulse for rogue AP detection

## Features

### 🐷 Piglet Personality
Your digital piglet companion reacts to discoveries:
- Gets excited when capturing handshakes
- Becomes sleepy during quiet periods
- Shows hunting focus when scanning

### 📡 OINK Mode
- Channel hopping WiFi scanner
- Promiscuous mode packet capture
- EAPOL/WPA handshake detection
- Deauth capability (for authorized testing only)
- ML-based network classification

### 🗺️ WARHOG Mode
- GPS-enabled wardriving
- Automatic network logging
- Export to CSV, Wigle, or Kismet formats
- Real-time statistics display

### 🧠 Machine Learning
- 32-feature extraction from WiFi beacon frames
- Enhanced heuristic classifier detecting:
  - **Rogue APs** - Strong signal, abnormal beacon timing, missing vendor IEs
  - **Evil Twins** - Hidden networks with suspiciously strong signal
  - **Vulnerable Networks** - Open, WEP, WPA1-only, WPS enabled
  - **Deauth Targets** - Non-WPA3 networks without PMF protection
- Edge Impulse SDK scaffold for custom model training
- ML training data export (CSV with all 32 features + GPS coords)

## Hardware Requirements

- M5Cardputer (ESP32-S3)
- AT6668 GPS Module (optional, for WARHOG mode)
- MicroSD card for data storage

## Quick Start

1. Flash the firmware via PlatformIO: `pio run -t upload -e m5cardputer`
2. Press `O` for OINK mode, `W` for WARHOG mode
3. Use `` ` `` (backtick) to access the menu
4. Configure settings via Settings menu (persistent to SPIFFS)

## Controls

| Key | Action |
|-----|--------|
| `O` | Enter OINK mode |
| `W` | Enter WARHOG mode |
| `S` | Enter Settings |
| `` ` `` | Toggle menu / Back |
| `;` | Navigate up / Decrease value |
| `.` | Navigate down / Increase value |
| `Enter` | Select / Toggle / Confirm |

## Settings

| Setting | Description | Default |
|---------|-------------|---------|
| Sound | Enable/disable beeps | ON |
| Brightness | Display brightness | 80% |
| CH Hop | Channel hop interval (ms) | 500 |
| Scan Time | Scan duration (ms) | 2000 |
| Deauth | Enable deauth attacks | ON |
| GPS | Enable GPS module | ON |
| GPS PwrSave | GPS power saving mode | ON |

## Building

```bash
# Install PlatformIO
pip install platformio

# Build
pio run

# Upload
pio run -t upload

# Monitor
pio device monitor
```

## ML Training

### Collecting Training Data

1. Run WARHOG mode to scan networks with GPS
2. Networks are automatically feature-extracted
3. Export ML training data to SD card:
   ```cpp
   WarhogMode::exportMLTraining("/sd/training.csv");
   ```
4. Label the data manually (edit CSV):
   - 0 = unknown
   - 1 = normal
   - 2 = rogue_ap
   - 3 = evil_twin
   - 4 = vulnerable

### Training with Edge Impulse

1. Create project at [studio.edgeimpulse.com](https://studio.edgeimpulse.com)
2. Upload labeled CSV (32 features per sample)
3. Design impulse: Raw data → Neural Network classifier
4. Train and test the model
5. Export as "C++ Library" for ESP32
6. Copy `edge-impulse-sdk/` folder to `lib/`
7. Uncomment `#define EDGE_IMPULSE_ENABLED` in `src/ml/edge_impulse.h`
8. Rebuild - real ML inference replaces heuristics!

## File Structure

```
porkchop/
├── src/
│   ├── main.cpp              # Entry point
│   ├── core/
│   │   ├── porkchop.cpp/h    # Main state machine
│   │   └── config.cpp/h      # Configuration management
│   ├── ui/
│   │   ├── display.cpp/h     # Triple-canvas display system
│   │   ├── menu.cpp/h        # Main menu
│   │   └── settings_menu.cpp/h  # Interactive settings
│   ├── piglet/
│   │   ├── avatar.cpp/h      # Derpy ASCII piglet with direction flip
│   │   └── mood.cpp/h        # Context-aware phrase system
│   ├── gps/
│   │   └── gps.cpp/h         # TinyGPS++ wrapper
│   ├── ml/
│   │   ├── features.cpp/h    # 32-feature WiFi extraction
│   │   ├── inference.cpp/h   # Heuristic + Edge Impulse classifier
│   │   └── edge_impulse.h    # Edge Impulse SDK scaffold
│   └── modes/
│       ├── oink.cpp/h        # WiFi scanning mode
│       └── warhog.cpp/h      # GPS wardriving mode
├── .github/
│   └── copilot-instructions.md  # AI coding assistant context
└── platformio.ini            # Build configuration
```

## Legal Disclaimer

This tool is intended for **authorized security research and educational purposes only**. 

- Only use on networks you own or have explicit permission to test
- Deauth attacks may be illegal in your jurisdiction
- The authors assume no liability for misuse

## Credits

- Inspired by [pwnagotchi](https://github.com/evilsocket/pwnagotchi)
- Built for [M5Cardputer](https://docs.m5stack.com/en/core/Cardputer)
- ML powered by [Edge Impulse](https://edgeimpulse.com/)

## License

MIT License - See LICENSE file for details

---

*Oink oink! 🐷*
