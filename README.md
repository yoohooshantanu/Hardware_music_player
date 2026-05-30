# ESP32-S3 Dual-Core Music Player

A high-performance MP3 music player firmware for ESP32-S3 with dedicated audio processing on Core 1 and system management on Core 0.

## Architecture

```
┌──────────────────┐     FreeRTOS Queue     ┌───────────────────────┐
│    CORE 0         │  ──── commands ──────▶ │      CORE 1           │
│  (System Task)    │                        │   (Audio Task)        │
│                   │  ◀── shared state ──── │                       │
│ • SD card mount   │   (volatile struct)    │ • MP3 decode (helix)  │
│ • File scanning   │                        │ • Ring buffer fill    │
│ • Button polling  │                        │ • Volume scaling      │
│ • Serial debug    │                        │ • I2S DMA output      │
└──────────────────┘                        └───────────────────────┘

Data Flow:
  SD Card ──▶ MP3 Buffer ──▶ Helix Decode ──▶ Ring Buffer ──▶ I2S DMA ──▶ DAC
                                                    ▲
                                             Volume Scaling
```

## Hardware Requirements

| Component | Specification |
|-----------|--------------|
| MCU | ESP32-S3 (16MB Flash, 8MB PSRAM, dual-core LX7) |
| SD Card | FAT32 formatted, connected via SDMMC interface |
| Audio DAC | External I2S DAC (MAX98357A, PCM5102, UDA1334A, etc.) |
| Buttons | 5× momentary push buttons (active LOW) |
| LED | Built-in or external LED for status indication |

> **Note:** The ESP32-S3 has **NO internal DAC** (unlike the original ESP32). An external I2S DAC module is required.

## Wiring

### I2S DAC Connection

| ESP32-S3 GPIO | DAC Pin | Function |
|--------------|---------|----------|
| GPIO 15 | BCLK | Bit Clock |
| GPIO 16 | LRCLK / WS | Left/Right Clock (Word Select) |
| GPIO 17 | DIN / DATA | Serial Data Out |
| 3.3V | VCC | Power |
| GND | GND | Ground |

### SD Card (SDMMC Mode)

| ESP32-S3 GPIO | SD Card Pin | Function |
|--------------|------------|----------|
| GPIO 39 | CLK | Clock |
| GPIO 38 | CMD | Command |
| GPIO 40 | D0 | Data 0 |
| GPIO 41 | D1 | Data 1 (4-bit mode) |
| GPIO 42 | D2 | Data 2 (4-bit mode) |
| GPIO 2 | D3 / CS | Data 3 (4-bit mode) |

> **Important:** Add 10kΩ external pull-up resistors on CMD and D0-D3 lines for reliable SDMMC operation. The firmware tries 4-bit mode first, then falls back to 1-bit mode automatically.

### Buttons (Active LOW, Internal Pull-up)

| ESP32-S3 GPIO | Function |
|--------------|----------|
| GPIO 6 | Play / Pause Toggle |
| GPIO 7 | Volume Up |
| GPIO 8 | Volume Down |
| GPIO 9 | Previous Track |
| GPIO 10 | Next Track |

Wire each button between the GPIO pin and GND. Internal pull-ups are enabled in firmware.

### Status LED

| ESP32-S3 GPIO | Function |
|--------------|----------|
| GPIO 48 | Status LED (built-in on DevKitC) |

- **Solid ON** = Playing
- **Blinking 1Hz** = Paused
- **Rapid blink** = Fatal error (SD mount failure)
- **Slow blink** = No MP3 files found

## Software Setup

### Prerequisites

1. [PlatformIO](https://platformio.org/) installed (VS Code extension recommended)
2. ESP32-S3 board with USB cable

### Build & Flash

```bash
# Clone or copy the project
cd music_player

# Build
pio run

# Upload to ESP32-S3
pio run -t upload

# Monitor serial output
pio device monitor
```

### SD Card Preparation

1. Format your SD card as **FAT32**
2. Copy `.mp3` files to the root directory or one level of subdirectories
3. The firmware scans root + 1 subdirectory level deep
4. Files are sorted alphabetically for consistent ordering
5. Maximum 512 tracks supported

## Pin Customization

All pins are defined as `#define` macros at the top of `src/main.cpp`. Modify them to match your board:

```cpp
// I2S DAC pins
#define I2S_BCLK_PIN       15
#define I2S_LRCLK_PIN      16
#define I2S_DOUT_PIN       17

// SDMMC pins
#define SDMMC_CLK_PIN      39
#define SDMMC_CMD_PIN      38
#define SDMMC_D0_PIN       40

// Button pins
#define BTN_PLAY_PAUSE      6
#define BTN_VOL_UP          7
// ... etc
```

## Volume Control

- **16 steps** (0 = mute, 16 = full volume)
- **Default:** Step 8 (50%)
- Software PCM scaling using fixed-point arithmetic
- Applied before I2S output for zero-latency response

## Memory Layout

| Allocation | Location | Size |
|-----------|----------|------|
| MP3 input buffer | PSRAM | 8 KB |
| PCM decode buffer | PSRAM | ~9 KB |
| PCM ring buffer | PSRAM | 128 KB |
| I2S write buffer | PSRAM | 4 KB |
| Playlist array | PSRAM | ~130 KB max |
| Audio task stack | Internal RAM | 16 KB |
| System task stack | Internal RAM | 8 KB |
| I2S DMA buffers | Internal RAM | ~32 KB |

## Error Handling

| Error | Behavior |
|-------|----------|
| SD mount failure | Rapid LED blink, halt, serial error |
| No MP3 files | Slow LED blink, halt, serial error |
| Decode error | Log to serial, skip after 50 consecutive errors |
| File open failure | Log to serial, set error state |
| Buffer allocation failure | Halt with LED blink |

## Serial Monitor Output

Connect at **115200 baud**. Example output:

```
╔══════════════════════════════════════╗
║   ESP32-S3  Music Player  v1.0       ║
║   Dual-Core | PSRAM | I2S DAC        ║
╚══════════════════════════════════════╝
Free heap:  303456 bytes
PSRAM free: 8375296 bytes
CPU freq:   240 MHz
[SD] Mounted in 4-bit SDMMC mode
[SD] Card type: SDHC, Size: 14832 MB
[PLAYLIST] Found 24 tracks:
  [  0] /music/01_song.mp3
  [  1] /music/02_song.mp3
  ...
[BUFFER] PCM ring buffer: 128 KB in PSRAM
[I2S] Configured: 44100 Hz, 16-bit stereo, 8 DMA buffers x 1024
[AUDIO] Playing [0] /music/01_song.mp3
[STATUS] PLAYING | Track 1/24 | Vol 8/16 | 128 kbps 44100 Hz 2ch | Buf: 98304/131072
```

## Technical Details

### Core Assignment

| Core | Task | Priority | Rationale |
|------|------|----------|-----------|
| Core 1 | Audio Task | MAX-1 | Uninterrupted decode + I2S feed prevents glitches |
| Core 0 | System Task | 2 | Buttons + debug are not time-critical |
| Core 1 | Arduino loop() | 1 | Idle (sleeps forever) |

### MP3 Decoder

Uses the **Helix fixed-point MP3 decoder** via [arduino-libhelix](https://github.com/pschatzmann/arduino-libhelix). The low-level C API (`MP3InitDecoder`, `MP3FindSyncWord`, `MP3Decode`) provides direct control over the decode pipeline for robust error recovery.

### Ring Buffer

Lock-free SPSC (Single Producer Single Consumer) design:
- Monotonically increasing indices with bitmask modulo
- Memory barriers for cross-core visibility
- Power-of-2 sizing for efficient arithmetic
- PSRAM-backed for large capacity without using internal SRAM

## License

This firmware is provided as-is for educational and personal use. The Helix MP3 decoder is licensed under its original terms (RealNetworks Public Source License).
