/*
 * =============================================================================
 *  ESP32-S3  Dual-Core  Music  Player  Firmware
 * =============================================================================
 *
 *  Architecture
 *  ────────────
 *  ┌──────────────────┐     FreeRTOS Queue     ┌───────────────────────┐
 *  │    CORE 0         │  ──── commands ──────▶ │      CORE 1           │
 *  │  (System Task)    │                        │   (Audio Task)        │
 *  │                   │  ◀── shared state ──── │                       │
 *  │ • SD card mount   │   (volatile struct)    │ • MP3 decode (helix)  │
 *  │ • File scanning   │                        │ • Ring buffer fill    │
 *  │ • Button polling  │                        │ • Volume scaling      │
 *  │ • Serial debug    │                        │ • I2S DMA output      │
 *  └──────────────────┘                        └───────────────────────┘
 *
 *  Data Flow (Audio Pipeline on Core 1)
 *  ─────────────────────────────────────
 *  SD Card ──▶ MP3 Input Buffer ──▶ Helix Decoder ──▶ PCM Ring Buffer
 *                                                         │
 *                                           volume scaling │
 *                                                         ▼
 *                                                  I2S DMA Output
 *                                                         │
 *                                                         ▼
 *                                                   External DAC
 *
 *  NOTE: The ESP32-S3 has NO internal DAC (unlike the original ESP32).
 *        An external I2S DAC (e.g. MAX98357A, PCM5102, UDA1334A) is required.
 *
 *  Build target : PlatformIO, espressif32, Arduino framework
 *  PSRAM        : 8 MB OPI – large allocations use ps_malloc()
 *  Library      : arduino-libhelix (pschatzmann) for fixed-point MP3 decoding
 * =============================================================================
 */

#include <Arduino.h>
#include <SD_MMC.h>
#include <FS.h>
#include <driver/i2s.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <algorithm>  // std::sort

/*
 * The arduino-libhelix library exposes the raw Helix fixed-point C decoder.
 * We use the low-level C API directly for maximum control over the decode
 * pipeline (sync-word search, frame-by-frame decode, error recovery).
 */
extern "C" {
#include "mp3dec.h"
}

#include "ring_buffer.h"
#include <TFT_eSPI.h>


/* ═══════════════════════════════════════════════════════════════════════════
 *  PIN DEFINITIONS  (all configurable – adjust to your board)
 * ═══════════════════════════════════════════════════════════════════════════ */

// --- I2S External DAC ---------------------------------------------------
#define I2S_BCLK_PIN       15   // Bit clock
#define I2S_LRCLK_PIN      16   // Left/Right clock (Word Select)
#define I2S_DOUT_PIN       17   // Serial data out

// --- SD Card (SDMMC) – ESP32-S3 uses the GPIO matrix, no fixed pins ------
//     Adjust these to match your board's SD card slot wiring.
#define SDMMC_CLK_PIN      39
#define SDMMC_CMD_PIN      38
#define SDMMC_D0_PIN       40
// For 4-bit mode (optional, set to -1 if unused):
#define SDMMC_D1_PIN       41
#define SDMMC_D2_PIN       42
#define SDMMC_D3_PIN        2

// --- Buttons (active LOW, internal pull-up) --------------------------------
#define BTN_PLAY_PAUSE      6
#define BTN_VOL_UP          7
#define BTN_VOL_DOWN        8
#define BTN_PREV            9
#define BTN_NEXT           10

// --- Status LED (built-in on many ESP32-S3-DevKitC boards) -----------------
#define LED_PIN            48

/* ═══════════════════════════════════════════════════════════════════════════
 *  CONSTANTS
 * ═══════════════════════════════════════════════════════════════════════════ */
#define SERIAL_BAUD        115200

// Audio buffer sizes
#define MP3_BUF_SIZE       (8 * 1024)       // 8 KB MP3 input buffer (PSRAM)
#define PCM_FRAME_MAX      (1152 * 2)       // Max PCM samples per MP3 frame (stereo)
#define RING_BUF_SIZE      (128 * 1024)     // 128 KB PCM ring buffer (PSRAM, power-of-2)
#define I2S_WRITE_CHUNK    (4 * 1024)       // 4 KB I2S write chunk

// I2S DMA configuration
#define I2S_SAMPLE_RATE    44100
#define I2S_DMA_BUF_COUNT  8
#define I2S_DMA_BUF_LEN    1024             // samples per DMA buffer

// Playlist
#define MAX_TRACKS         512
#define MAX_PATH_LEN       256
#define MAX_SCAN_DEPTH     1                // scan root + 1 level deep

// Volume
#define VOLUME_STEPS       16
#define VOLUME_DEFAULT      8

// Buttons
#define BTN_DEBOUNCE_MS    20

// Tasks
#define AUDIO_TASK_STACK   (16 * 1024)      // 16 KB stack for audio (generous for helix)
#define SYSTEM_TASK_STACK  (8 * 1024)       // 8 KB stack for system

// Error recovery
#define MAX_DECODE_ERRORS  50               // skip track after this many consecutive errors

/* ═══════════════════════════════════════════════════════════════════════════
 *  ENUMS & STRUCTS
 * ═══════════════════════════════════════════════════════════════════════════ */

// Commands sent from Core 0 → Core 1 via FreeRTOS queue
enum AudioCommand : uint8_t {
    CMD_PLAY,           // Start/resume playback (param = track index if fresh play)
    CMD_PAUSE,          // Pause playback
    CMD_RESUME,         // Resume after pause
    CMD_STOP,           // Stop playback entirely
    CMD_NEXT,           // Next track
    CMD_PREV,           // Previous track
    CMD_VOL_UP,         // Increase volume by 1 step
    CMD_VOL_DOWN        // Decrease volume by 1 step
};

struct CommandMsg {
    AudioCommand cmd;
    uint16_t     param;    // optional parameter (e.g. track index for CMD_PLAY)
};

// Audio state communicated from Core 1 → Core 0 via volatile struct
enum AudioState : uint8_t {
    STATE_IDLE = 0,
    STATE_PLAYING,
    STATE_PAUSED,
    STATE_BUFFERING,
    STATE_ERROR,
    STATE_STOPPED
};

/*
 * SharedAudioState — written ONLY by the audio task (Core 1),
 * read by the system task (Core 0) for status display.
 * The `volatile` qualifier prevents the compiler from caching stale reads
 * across loop iterations on Core 0.
 */
struct SharedAudioState {
    volatile AudioState state;
    volatile uint8_t    volume;
    volatile uint16_t   current_track;
    volatile int        sample_rate;
    volatile int        channels;
    volatile int        bitrate_kbps;
};

// Button debounce state
struct ButtonState {
    uint8_t  pin;
    bool     last_reading;      // raw GPIO reading from previous poll
    bool     stable_state;      // debounced stable state (HIGH = released)
    uint32_t last_change_ms;    // millis() when last_reading changed
};

/* ═══════════════════════════════════════════════════════════════════════════
 *  GLOBALS
 * ═══════════════════════════════════════════════════════════════════════════ */

// Playlist (allocated in PSRAM)
static char**   playlist     = nullptr;
static uint16_t track_count  = 0;

// Inter-core communication
static QueueHandle_t    cmd_queue        = nullptr;   // Core 0 → Core 1
static SharedAudioState audio_state      = {};        // Core 1 → Core 0

// PCM ring buffer (PSRAM-backed)
static RingBuffer pcm_ring_buffer;

// TFT Display
static TFT_eSPI tft = TFT_eSPI();
static TFT_eSprite sprite = TFT_eSprite(&tft);

// Task handles

static TaskHandle_t audio_task_handle  = nullptr;
static TaskHandle_t system_task_handle = nullptr;

// Buttons array
static ButtonState buttons[] = {
    { BTN_PLAY_PAUSE, true, true, 0 },
    { BTN_VOL_UP,     true, true, 0 },
    { BTN_VOL_DOWN,   true, true, 0 },
    { BTN_PREV,       true, true, 0 },
    { BTN_NEXT,       true, true, 0 }
};
static constexpr int NUM_BUTTONS = sizeof(buttons) / sizeof(buttons[0]);

/* ═══════════════════════════════════════════════════════════════════════════
 *  LED FUNCTIONS
 * ═══════════════════════════════════════════════════════════════════════════ */

static void led_init() {
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
}

/*
 * Blink the LED at the given interval and HALT (never returns).
 * Used for fatal errors:
 *   • 100 ms interval = rapid blink  (SD mount failure)
 *   • 500 ms interval = slow blink   (no MP3 files found)
 */
static void led_blink_halt(uint16_t interval_ms) {
    Serial.printf("[HALT] Blinking LED every %d ms\n", interval_ms);
    while (true) {
        digitalWrite(LED_PIN, HIGH);
        delay(interval_ms);
        digitalWrite(LED_PIN, LOW);
        delay(interval_ms);
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  SD CARD FUNCTIONS
 * ═══════════════════════════════════════════════════════════════════════════ */

/*
 * Mount the SD card via SDMMC peripheral.
 * Tries 4-bit mode first, then falls back to 1-bit mode for broader
 * compatibility with different board designs.
 */
static bool mount_sd_card() {
    // Configure SDMMC pins (ESP32-S3 requires explicit pin assignment)
    SD_MMC.setPins(SDMMC_CLK_PIN, SDMMC_CMD_PIN, SDMMC_D0_PIN,
                   SDMMC_D1_PIN, SDMMC_D2_PIN, SDMMC_D3_PIN);

    // Attempt 4-bit mode first (mode1bit = false)
    if (SD_MMC.begin("/sdcard", false)) {
        Serial.println("[SD] Mounted in 4-bit SDMMC mode");
    } else {
        Serial.println("[SD] 4-bit mode failed, trying 1-bit...");
        // setPins again for 1-bit (only CLK, CMD, D0 matter)
        SD_MMC.setPins(SDMMC_CLK_PIN, SDMMC_CMD_PIN, SDMMC_D0_PIN);
        if (!SD_MMC.begin("/sdcard", true)) {
            Serial.println("[SD] ERROR: SD card mount failed in both modes!");
            return false;
        }
        Serial.println("[SD] Mounted in 1-bit SDMMC mode");
    }

    // Print card info
    uint8_t  card_type = SD_MMC.cardType();
    uint64_t card_size = SD_MMC.cardSize() / (1024 * 1024);
    const char* type_str = (card_type == CARD_MMC)  ? "MMC" :
                           (card_type == CARD_SD)   ? "SDSC" :
                           (card_type == CARD_SDHC) ? "SDHC" : "UNKNOWN";
    Serial.printf("[SD] Card type: %s, Size: %llu MB\n", type_str, card_size);
    return true;
}

// Check if filename ends with ".mp3" (case-insensitive)
static bool is_mp3_file(const char* name) {
    size_t len = strlen(name);
    if (len < 5) return false;   // minimum: "x.mp3"
    const char* ext = name + len - 4;
    return (strcasecmp(ext, ".mp3") == 0);
}

/*
 * Recursively scan directory for MP3 files (root + MAX_SCAN_DEPTH levels).
 * Paths are stored in the PSRAM-backed playlist array.
 */
static void scan_mp3_files(fs::FS& fs, const char* dir_path, int depth) {
    if (depth > MAX_SCAN_DEPTH) return;
    if (track_count >= MAX_TRACKS) return;

    File dir = fs.open(dir_path);
    if (!dir || !dir.isDirectory()) {
        Serial.printf("[SCAN] Cannot open directory: %s\n", dir_path);
        return;
    }

    File entry;
    while ((entry = dir.openNextFile()) && track_count < MAX_TRACKS) {
        if (entry.isDirectory()) {
            // Recurse one level deeper
            scan_mp3_files(fs, entry.path(), depth + 1);
        } else if (is_mp3_file(entry.name())) {
            // Store full path in PSRAM
            size_t path_len = strlen(entry.path());
            if (path_len < MAX_PATH_LEN) {
                playlist[track_count] = (char*)ps_malloc(path_len + 1);
                if (playlist[track_count]) {
                    strcpy(playlist[track_count], entry.path());
                    track_count++;
                }
            }
        }
        entry.close();
    }
    dir.close();
}

// Comparison function for sorting playlist alphabetically
static int playlist_cmp(const void* a, const void* b) {
    return strcasecmp(*(const char**)a, *(const char**)b);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  BUTTON HANDLING  (Core 0 only)
 * ═══════════════════════════════════════════════════════════════════════════ */

static void init_buttons() {
    for (int i = 0; i < NUM_BUTTONS; i++) {
        pinMode(buttons[i].pin, INPUT_PULLUP);
        buttons[i].last_reading   = true;   // pulled high = released
        buttons[i].stable_state   = true;
        buttons[i].last_change_ms = 0;
    }
}

/*
 * Debounce algorithm (per-button):
 *  1. Read current GPIO level.
 *  2. If it differs from last_reading, restart the debounce timer.
 *  3. If debounce period has elapsed and the reading differs from
 *     stable_state, update stable_state.
 *  4. Detect falling edge (HIGH→LOW = button press) and send command.
 *
 * Called every ~10 ms from the system task on Core 0.
 */
static void poll_buttons() {
    uint32_t now = millis();

    for (int i = 0; i < NUM_BUTTONS; i++) {
        bool reading = digitalRead(buttons[i].pin);

        // If reading changed, restart debounce timer
        if (reading != buttons[i].last_reading) {
            buttons[i].last_change_ms = now;
        }
        buttons[i].last_reading = reading;

        // Has debounce period elapsed?
        if ((now - buttons[i].last_change_ms) < BTN_DEBOUNCE_MS) {
            continue;
        }

        // Stable reading differs from current stable state → edge detected
        if (reading != buttons[i].stable_state) {
            bool was_high = buttons[i].stable_state;
            buttons[i].stable_state = reading;

            // We only act on the falling edge (press, HIGH→LOW)
            if (was_high && !reading) {
                CommandMsg msg = { CMD_PLAY, 0 };  // default, overwritten below

                switch (buttons[i].pin) {
                    case BTN_PLAY_PAUSE:
                        // Toggle: if playing → pause, else → resume/play
                        if (audio_state.state == STATE_PLAYING) {
                            msg.cmd = CMD_PAUSE;
                        } else {
                            msg.cmd = CMD_RESUME;
                        }
                        break;
                    case BTN_VOL_UP:
                        msg.cmd = CMD_VOL_UP;
                        break;
                    case BTN_VOL_DOWN:
                        msg.cmd = CMD_VOL_DOWN;
                        break;
                    case BTN_PREV:
                        msg.cmd = CMD_PREV;
                        break;
                    case BTN_NEXT:
                        msg.cmd = CMD_NEXT;
                        break;
                }
                // Non-blocking send (don't block Core 0 if queue is full)
                xQueueSend(cmd_queue, &msg, 0);
            }
        }
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  VOLUME SCALING
 * ═══════════════════════════════════════════════════════════════════════════
 *
 *  Applied to decoded PCM samples BEFORE writing to the ring buffer.
 *  Uses fixed-point arithmetic (8-bit fractional) for efficiency:
 *
 *    scale_factor = (volume * 256) / VOLUME_STEPS
 *    sample_out   = (sample_in * scale_factor) >> 8
 *
 *  Volume 0  = mute  (memset to zero)
 *  Volume 16 = full  (no scaling, pass-through)
 */
static void apply_volume(int16_t* samples, size_t sample_count, uint8_t volume) {
    if (volume >= VOLUME_STEPS) return;   // Full volume — no work needed

    if (volume == 0) {
        memset(samples, 0, sample_count * sizeof(int16_t));
        return;
    }

    int32_t scale = ((int32_t)volume << 8) / VOLUME_STEPS;
    for (size_t i = 0; i < sample_count; i++) {
        samples[i] = (int16_t)(((int32_t)samples[i] * scale) >> 8);
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  I2S SETUP
 * ═══════════════════════════════════════════════════════════════════════════
 *
 *  Configures the I2S peripheral in standard (Philips) mode for driving
 *  an external I2S DAC.  DMA is set up with 8 buffers of 1024 samples each,
 *  giving ~186 ms of buffer depth at 44100 Hz stereo 16-bit.
 *
 *  The ESP32-S3 has NO internal DAC.  You MUST connect an external I2S DAC
 *  to the BCLK / LRCLK / DOUT pins defined above.
 */
static void i2s_setup(int sample_rate) {
    i2s_config_t i2s_config = {};
    i2s_config.mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
    i2s_config.sample_rate          = sample_rate;
    i2s_config.bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT;
    i2s_config.channel_format       = I2S_CHANNEL_FMT_RIGHT_LEFT;
    i2s_config.communication_format = I2S_COMM_FORMAT_STAND_I2S;
    i2s_config.intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1;
    i2s_config.dma_buf_count        = I2S_DMA_BUF_COUNT;
    i2s_config.dma_buf_len          = I2S_DMA_BUF_LEN;
    i2s_config.use_apll             = true;     // APLL for accurate clocks
    i2s_config.tx_desc_auto_clear   = true;     // Auto-clear DMA on underflow

    i2s_pin_config_t pin_config = {};
    pin_config.mck_io_num   = I2S_PIN_NO_CHANGE;   // No MCLK (most DACs don't need it)
    pin_config.bck_io_num   = I2S_BCLK_PIN;
    pin_config.ws_io_num    = I2S_LRCLK_PIN;
    pin_config.data_out_num = I2S_DOUT_PIN;
    pin_config.data_in_num  = I2S_PIN_NO_CHANGE;

    esp_err_t err;
    err = i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
    if (err != ESP_OK) {
        Serial.printf("[I2S] ERROR: driver install failed (%d)\n", err);
    }
    err = i2s_set_pin(I2S_NUM_0, &pin_config);
    if (err != ESP_OK) {
        Serial.printf("[I2S] ERROR: set pin failed (%d)\n", err);
    }
    i2s_zero_dma_buffer(I2S_NUM_0);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  MONO → STEREO EXPANSION
 * ═══════════════════════════════════════════════════════════════════════════
 *
 *  I2S is configured for stereo (L+R).  If the MP3 file is mono,
 *  we duplicate each sample into both channels, in-place.
 *  The buffer must be large enough to hold 2× the original samples.
 */
static void mono_to_stereo(int16_t* buf, int mono_samples) {
    // Work backwards to avoid overwriting unprocessed data
    for (int i = mono_samples - 1; i >= 0; i--) {
        buf[i * 2 + 1] = buf[i];   // Right channel
        buf[i * 2]     = buf[i];   // Left channel
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  AUDIO TASK  (Core 1)
 * ═══════════════════════════════════════════════════════════════════════════
 *
 *  This task is pinned to Core 1 and runs at the highest priority.
 *  It owns the entire audio pipeline:
 *
 *    1. Read compressed MP3 data from the SD card file
 *    2. Find sync word & decode one frame at a time (Helix fixed-point)
 *    3. Apply volume scaling to decoded PCM
 *    4. Write PCM to the PSRAM ring buffer
 *    5. Drain the ring buffer to I2S (DMA blocking provides rate control)
 *    6. Process commands from Core 0 between iterations
 *
 *  The ring buffer decouples the bursty decode from the constant-rate
 *  I2S DMA consumption, preventing audio glitches.
 */

static void audio_task(void* param) {
    // ---- Allocate working buffers in PSRAM ----
    uint8_t* mp3_buf = (uint8_t*)ps_malloc(MP3_BUF_SIZE);
    // PCM buffer: enough for max frame size, stereo expansion
    int16_t* pcm_buf = (int16_t*)ps_malloc(PCM_FRAME_MAX * 2 * sizeof(int16_t));
    // I2S output chunk buffer
    uint8_t* i2s_buf = (uint8_t*)ps_malloc(I2S_WRITE_CHUNK);

    if (!mp3_buf || !pcm_buf || !i2s_buf) {
        Serial.println("[AUDIO] FATAL: Failed to allocate audio buffers!");
        audio_state.state = STATE_ERROR;
        vTaskDelete(NULL);
        return;
    }

    // ---- Initialize Helix MP3 decoder ----
    HMP3Decoder decoder = MP3InitDecoder();
    if (!decoder) {
        Serial.println("[AUDIO] FATAL: MP3InitDecoder failed!");
        audio_state.state = STATE_ERROR;
        vTaskDelete(NULL);
        return;
    }

    // ---- Local state ----
    File     current_file;
    bool     file_open        = false;
    int      mp3_bytes_in_buf = 0;      // Valid bytes in mp3_buf
    bool     file_ended       = false;
    int      current_sr       = I2S_SAMPLE_RATE;
    int      decode_errors    = 0;

    Serial.println("[AUDIO] Audio task started on Core 1");

    // ── MAIN LOOP ──────────────────────────────────────────────────────────
    while (true) {

        // ── 1. Process commands from Core 0 (non-blocking) ─────────────
        CommandMsg msg;
        while (xQueueReceive(cmd_queue, &msg, 0) == pdTRUE) {
            switch (msg.cmd) {

                case CMD_PLAY: {
                    // Close any existing file
                    if (file_open) { current_file.close(); file_open = false; }

                    uint16_t idx = msg.param;
                    if (idx >= track_count) idx = 0;
                    audio_state.current_track = idx;

                    current_file = SD_MMC.open(playlist[idx], FILE_READ);
                    if (!current_file) {
                        Serial.printf("[AUDIO] ERROR: Cannot open %s\n", playlist[idx]);
                        audio_state.state = STATE_ERROR;
                        break;
                    }

                    file_open        = true;
                    file_ended       = false;
                    mp3_bytes_in_buf = 0;
                    decode_errors    = 0;
                    pcm_ring_buffer.reset();
                    // Re-init decoder for new file
                    MP3FreeDecoder(decoder);
                    decoder = MP3InitDecoder();
                    audio_state.state = STATE_PLAYING;
                    Serial.printf("[AUDIO] Playing [%d] %s\n", idx, playlist[idx]);
                    break;
                }

                case CMD_PAUSE:
                    if (audio_state.state == STATE_PLAYING) {
                        audio_state.state = STATE_PAUSED;
                        i2s_zero_dma_buffer(I2S_NUM_0);
                        Serial.println("[AUDIO] Paused");
                    }
                    break;

                case CMD_RESUME:
                    if (audio_state.state == STATE_PAUSED) {
                        audio_state.state = STATE_PLAYING;
                        Serial.println("[AUDIO] Resumed");
                    } else if (audio_state.state == STATE_IDLE ||
                               audio_state.state == STATE_STOPPED) {
                        // Fresh play from current track
                        CommandMsg play_msg = { CMD_PLAY, audio_state.current_track };
                        xQueueSend(cmd_queue, &play_msg, 0);
                    }
                    break;

                case CMD_STOP:
                    if (file_open) { current_file.close(); file_open = false; }
                    audio_state.state = STATE_STOPPED;
                    pcm_ring_buffer.reset();
                    i2s_zero_dma_buffer(I2S_NUM_0);
                    Serial.println("[AUDIO] Stopped");
                    break;

                case CMD_NEXT: {
                    uint16_t next = (audio_state.current_track + 1) % track_count;
                    CommandMsg play_msg = { CMD_PLAY, next };
                    xQueueSend(cmd_queue, &play_msg, 0);
                    break;
                }

                case CMD_PREV: {
                    uint16_t prev = (audio_state.current_track == 0)
                                    ? track_count - 1
                                    : audio_state.current_track - 1;
                    CommandMsg play_msg = { CMD_PLAY, prev };
                    xQueueSend(cmd_queue, &play_msg, 0);
                    break;
                }

                case CMD_VOL_UP:
                    if (audio_state.volume < VOLUME_STEPS) {
                        audio_state.volume++;
                        Serial.printf("[AUDIO] Volume: %d/%d\n",
                                      audio_state.volume, VOLUME_STEPS);
                    }
                    break;

                case CMD_VOL_DOWN:
                    if (audio_state.volume > 0) {
                        audio_state.volume--;
                        Serial.printf("[AUDIO] Volume: %d/%d\n",
                                      audio_state.volume, VOLUME_STEPS);
                    }
                    break;
            }
        }

        // ── 2. PLAYING state: decode + buffer + I2S ────────────────────
        if (audio_state.state == STATE_PLAYING && file_open) {

            /*
             * DECODE PHASE: Fill the ring buffer with decoded PCM.
             * Decode multiple frames per iteration while the ring buffer
             * has room, to build up a comfortable buffer ahead of I2S.
             */
            int frames_decoded = 0;
            while (pcm_ring_buffer.available_write() >= (PCM_FRAME_MAX * sizeof(int16_t))
                   && !file_ended && frames_decoded < 4) {

                // ---- Refill the MP3 input buffer from SD ----
                if (mp3_bytes_in_buf < MP3_BUF_SIZE) {
                    int space = MP3_BUF_SIZE - mp3_bytes_in_buf;
                    int avail = current_file.available();
                    int to_read = (avail < space) ? avail : space;
                    if (to_read > 0) {
                        int rd = current_file.read(mp3_buf + mp3_bytes_in_buf, to_read);
                        if (rd > 0) mp3_bytes_in_buf += rd;
                    }
                    if (current_file.available() == 0 && mp3_bytes_in_buf == 0) {
                        file_ended = true;
                        break;
                    }
                }

                if (mp3_bytes_in_buf < 4) {
                    file_ended = true;
                    break;
                }

                // ---- Find MP3 sync word ----
                int sync_offset = MP3FindSyncWord(mp3_buf, mp3_bytes_in_buf);
                if (sync_offset < 0) {
                    // No sync found – discard buffer, refill next iteration
                    mp3_bytes_in_buf = 0;
                    if (current_file.available() == 0) {
                        file_ended = true;
                    }
                    break;
                }
                if (sync_offset > 0) {
                    // Skip ID3 tags / junk bytes before sync word
                    memmove(mp3_buf, mp3_buf + sync_offset,
                            mp3_bytes_in_buf - sync_offset);
                    mp3_bytes_in_buf -= sync_offset;
                }

                // ---- Decode one frame ----
                unsigned char* read_ptr = mp3_buf;
                int bytes_left = mp3_bytes_in_buf;

                int err = MP3Decode(decoder, &read_ptr, &bytes_left,
                                    pcm_buf, 0);

                // Calculate consumed bytes and shift buffer
                int consumed = mp3_bytes_in_buf - bytes_left;
                if (consumed > 0 && bytes_left > 0) {
                    memmove(mp3_buf, mp3_buf + consumed, bytes_left);
                }
                mp3_bytes_in_buf = bytes_left;

                if (err == 0) {
                    // ---- Success: get frame info ----
                    MP3FrameInfo info;
                    MP3GetLastFrameInfo(decoder, &info);

                    decode_errors = 0;  // Reset error counter on success

                    // Update shared state
                    audio_state.sample_rate  = info.samprate;
                    audio_state.channels     = info.nChans;
                    audio_state.bitrate_kbps = info.bitrate / 1000;

                    // Reconfigure I2S if sample rate changed
                    if (info.samprate != current_sr && info.samprate > 0) {
                        i2s_set_sample_rates(I2S_NUM_0, info.samprate);
                        current_sr = info.samprate;
                        Serial.printf("[AUDIO] Sample rate: %d Hz\n", current_sr);
                    }

                    int total_samples = info.outputSamps; // Total int16_t samples

                    // Handle mono: duplicate into stereo for I2S
                    if (info.nChans == 1) {
                        mono_to_stereo(pcm_buf, total_samples);
                        total_samples *= 2;
                    }

                    // Apply software volume scaling
                    apply_volume(pcm_buf, total_samples, audio_state.volume);

                    // Write decoded PCM to ring buffer
                    size_t pcm_bytes = total_samples * sizeof(int16_t);
                    size_t written = 0;
                    int retries = 0;
                    while (written < pcm_bytes && retries < 100) {
                        size_t w = pcm_ring_buffer.write(
                            (uint8_t*)pcm_buf + written,
                            pcm_bytes - written
                        );
                        written += w;
                        if (written < pcm_bytes) {
                            // Ring buffer full — drain some to I2S first
                            size_t drain = pcm_ring_buffer.available_read();
                            if (drain > 0) {
                                size_t chunk = (drain < (size_t)I2S_WRITE_CHUNK)
                                               ? drain : I2S_WRITE_CHUNK;
                                pcm_ring_buffer.read(i2s_buf, chunk);
                                size_t i2s_written = 0;
                                i2s_write(I2S_NUM_0, i2s_buf, chunk,
                                          &i2s_written, pdMS_TO_TICKS(100));
                            }
                            retries++;
                        }
                    }

                    frames_decoded++;

                } else if (err == ERR_MP3_INDATA_UNDERFLOW) {
                    // Need more input data — will refill on next iteration
                    if (current_file.available() == 0) {
                        file_ended = true;
                    }
                    break;

                } else {
                    // Decode error — try to recover
                    decode_errors++;
                    Serial.printf("[AUDIO] Decode error %d (count: %d)\n",
                                  err, decode_errors);

                    if (decode_errors >= MAX_DECODE_ERRORS) {
                        Serial.println("[AUDIO] Too many errors, skipping track");
                        CommandMsg next_msg = { CMD_NEXT, 0 };
                        xQueueSend(cmd_queue, &next_msg, 0);
                        break;
                    }

                    // Skip one byte and try to re-sync
                    if (mp3_bytes_in_buf > 1) {
                        memmove(mp3_buf, mp3_buf + 1, mp3_bytes_in_buf - 1);
                        mp3_bytes_in_buf--;
                    } else {
                        mp3_bytes_in_buf = 0;
                    }
                }
            }

            /*
             * I2S OUTPUT PHASE: Drain the ring buffer to I2S.
             * i2s_write() blocks until DMA buffers have space, which provides
             * natural rate-limiting — the task sleeps inside i2s_write() when
             * DMA is full, giving CPU time back to the system.
             */
            size_t avail = pcm_ring_buffer.available_read();
            if (avail > 0) {
                size_t chunk = (avail < (size_t)I2S_WRITE_CHUNK)
                               ? avail : I2S_WRITE_CHUNK;
                size_t read_bytes = pcm_ring_buffer.read(i2s_buf, chunk);
                if (read_bytes > 0) {
                    size_t written = 0;
                    i2s_write(I2S_NUM_0, i2s_buf, read_bytes,
                              &written, pdMS_TO_TICKS(500));
                }
            } else if (file_ended) {
                // File ended and ring buffer drained → advance to next track
                Serial.println("[AUDIO] Track finished, advancing...");
                CommandMsg next_msg = { CMD_NEXT, 0 };
                xQueueSend(cmd_queue, &next_msg, 0);
                file_ended = false;   // reset for next track
            } else {
                taskYIELD();  // Nothing to do, yield briefly
            }

        } else if (audio_state.state == STATE_PAUSED) {
            // Paused — sleep to avoid spinning
            vTaskDelay(pdMS_TO_TICKS(50));

        } else {
            // Idle / Stopped / Error — sleep longer
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }

    // Cleanup (unreachable in normal operation)
    MP3FreeDecoder(decoder);
    free(mp3_buf);
    free(pcm_buf);
    free(i2s_buf);
    vTaskDelete(NULL);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  UI RENDERING (Core 0)
 * ═══════════════════════════════════════════════════════════════════════════
 */
static void draw_ui() {
    sprite.fillSprite(TFT_BLACK);
    
    // --- Header ---
    sprite.fillRect(0, 0, 240, 40, TFT_DARKGREY);
    sprite.setTextColor(TFT_WHITE, TFT_DARKGREY);
    sprite.setTextDatum(MC_DATUM);
    
    const char* state_str = "IDLE";
    uint16_t state_color = TFT_WHITE;
    switch(audio_state.state) {
        case STATE_PLAYING:   state_str = "PLAYING";   state_color = TFT_GREEN;  break;
        case STATE_PAUSED:    state_str = "PAUSED";    state_color = TFT_ORANGE; break;
        case STATE_STOPPED:   state_str = "STOPPED";   state_color = TFT_RED;    break;
        case STATE_ERROR:     state_str = "ERROR";     state_color = TFT_RED;    break;
        case STATE_BUFFERING: state_str = "BUFFERING"; state_color = TFT_YELLOW; break;
        default: break;
    }
    
    sprite.setTextColor(state_color, TFT_DARKGREY);
    sprite.drawString(state_str, 60, 20, 4);
    
    // Volume bar
    sprite.drawRect(130, 10, 100, 20, TFT_WHITE);
    int vol_w = (audio_state.volume * 100) / VOLUME_STEPS;
    sprite.fillRect(130, 10, vol_w, 20, TFT_GREEN);
    
    // --- Body (Track Info) ---
    sprite.setTextColor(TFT_WHITE, TFT_BLACK);
    sprite.setTextDatum(MC_DATUM);
    
    char track_num[32];
    sprintf(track_num, "Track %d / %d", audio_state.current_track + 1, track_count);
    sprite.drawString(track_num, 120, 100, 4);
    
    if (audio_state.current_track < track_count && track_count > 0) {
        // Extract filename from path
        const char* path = playlist[audio_state.current_track];
        const char* name = strrchr(path, '/');
        name = name ? name + 1 : path;
        
        char short_name[32];
        strncpy(short_name, name, 31);
        short_name[31] = '\0';
        sprite.drawString(short_name, 120, 160, 4);
    }
    
    // --- Footer (Format Info) ---
    sprite.fillRect(0, 280, 240, 40, TFT_NAVY);
    sprite.setTextColor(TFT_LIGHTGREY, TFT_NAVY);
    sprite.setTextDatum(MC_DATUM);
    
    if (audio_state.state == STATE_PLAYING || audio_state.state == STATE_PAUSED) {
        char fmt[64];
        sprintf(fmt, "%d kbps | %d Hz | %d ch", 
                audio_state.bitrate_kbps, audio_state.sample_rate, audio_state.channels);
        sprite.drawString(fmt, 120, 300, 2);
    }
    
    sprite.pushSprite(0, 0);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  SYSTEM TASK  (Core 0)
 * ═══════════════════════════════════════════════════════════════════════════
 */
static void system_task(void* param) {
    uint32_t last_status_ms = 0;
    AudioState prev_state = STATE_IDLE;
    uint8_t prev_volume = 255;
    uint16_t prev_track = 65535;

    Serial.println("[SYSTEM] System task started on Core 0");

    // Draw initial UI
    draw_ui();

    while (true) {
        // ── Poll buttons ───────────────────────────────────────────────
        poll_buttons();

        uint32_t now = millis();
        bool needs_redraw = false;

        // Only redraw UI when essential state changes
        if (audio_state.state != prev_state || 
            audio_state.volume != prev_volume ||
            audio_state.current_track != prev_track) {
            
            prev_state = audio_state.state;
            prev_volume = audio_state.volume;
            prev_track = audio_state.current_track;
            needs_redraw = true;
        }

        if (needs_redraw) {
            draw_ui();
        }

        // ── Periodic serial status output ─────────────────────────────────────
        if (now - last_status_ms > 2000) {
            last_status_ms = now;
            
            // We also periodically redraw the UI to recover from any potential glitches
            draw_ui();

            const char* state_names[] = { "IDLE", "PLAYING", "PAUSED", "BUFFERING", "ERROR", "STOPPED" };
            uint8_t st = audio_state.state;
            const char* st_name = (st < 6) ? state_names[st] : "???";

            Serial.printf("[STATUS] %s | Track %d/%d | Vol %d/%d\n",
                          st_name, audio_state.current_track + 1, track_count,
                          audio_state.volume, VOLUME_STEPS);

            // LED: solid while playing, blink while paused
            if (st == STATE_PLAYING) {
                digitalWrite(LED_PIN, HIGH);
            } else if (st == STATE_PAUSED) {
                digitalWrite(LED_PIN, (now / 500) % 2);  // Blink 1 Hz
            } else {
                digitalWrite(LED_PIN, LOW);
            }
        }

        // ── 10 ms poll interval ────────────────────────────────────────
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}


/* ═══════════════════════════════════════════════════════════════════════════
 *  SETUP  (runs on Core 1 in Arduino ESP32)
 * ═══════════════════════════════════════════════════════════════════════════ */
void setup() {
    Serial.begin(SERIAL_BAUD);
    delay(1000);   // Allow USB-CDC serial to connect

    Serial.println();
    Serial.println("╔══════════════════════════════════════╗");
    Serial.println("║   ESP32-S3  Music Player  v1.0       ║");
    Serial.println("║   Dual-Core | PSRAM | I2S DAC        ║");
    Serial.println("╚══════════════════════════════════════╝");
    Serial.printf("Free heap:  %u bytes\n", ESP.getFreeHeap());
    Serial.printf("PSRAM free: %u bytes\n", ESP.getFreePsram());
    Serial.printf("CPU freq:   %d MHz\n",   ESP.getCpuFreqMHz());

    // ── LED ────────────────────────────────────────────────────────────
    led_init();

    // ── SD Card ────────────────────────────────────────────────────────
    if (!mount_sd_card()) {
        Serial.println("[FATAL] SD card mount failed!");
        led_blink_halt(100);   // Rapid blink and halt
    }

    // ── Scan for MP3 files ─────────────────────────────────────────────
    // Allocate playlist pointer array in PSRAM
    playlist = (char**)ps_calloc(MAX_TRACKS, sizeof(char*));
    if (!playlist) {
        Serial.println("[FATAL] Playlist allocation failed!");
        led_blink_halt(100);
    }

    scan_mp3_files(SD_MMC, "/", 0);

    if (track_count == 0) {
        Serial.println("[FATAL] No MP3 files found on SD card!");
        led_blink_halt(500);   // Slow blink and halt
    }

    // Sort playlist alphabetically
    qsort(playlist, track_count, sizeof(char*), playlist_cmp);

    Serial.printf("[PLAYLIST] Found %d tracks:\n", track_count);
    for (int i = 0; i < track_count && i < 32; i++) {
        Serial.printf("  [%3d] %s\n", i, playlist[i]);
    }
    if (track_count > 32) {
        Serial.printf("  ... and %d more\n", track_count - 32);
    }

    // ── Ring Buffer (PSRAM) ────────────────────────────────────────────
    if (!pcm_ring_buffer.init(RING_BUF_SIZE)) {
        Serial.println("[FATAL] Ring buffer PSRAM allocation failed!");
        led_blink_halt(100);
    }
    Serial.printf("[BUFFER] PCM ring buffer: %d KB in PSRAM\n",
                  RING_BUF_SIZE / 1024);

    // ── Command Queue ──────────────────────────────────────────────────
    cmd_queue = xQueueCreate(16, sizeof(CommandMsg));
    if (!cmd_queue) {
        Serial.println("[FATAL] Queue creation failed!");
        led_blink_halt(100);
    }

    // ── Shared Audio State ─────────────────────────────────────────────
    memset((void*)&audio_state, 0, sizeof(audio_state));
    audio_state.volume = VOLUME_DEFAULT;
    audio_state.state  = STATE_IDLE;

    // ── I2S DAC ────────────────────────────────────────────────────────
    i2s_setup(I2S_SAMPLE_RATE);
    Serial.printf("[I2S] Configured: %d Hz, 16-bit stereo, %d DMA buffers x %d\n",
                  I2S_SAMPLE_RATE, I2S_DMA_BUF_COUNT, I2S_DMA_BUF_LEN);
    Serial.printf("[I2S] Pins: BCLK=%d, LRCLK=%d, DOUT=%d\n",
                  I2S_BCLK_PIN, I2S_LRCLK_PIN, I2S_DOUT_PIN);

    // ── Buttons ────────────────────────────────────────────────────────
    init_buttons();
    Serial.printf("[BTN] 5 buttons configured (pins %d,%d,%d,%d,%d)\n",
                  BTN_PLAY_PAUSE, BTN_VOL_UP, BTN_VOL_DOWN,
                  BTN_PREV, BTN_NEXT);

    // ── TFT Display ────────────────────────────────────────────────────
    Serial.println("[TFT] Initializing ST7789 display...");
    tft.begin();
    tft.setRotation(0);
    tft.fillScreen(TFT_BLACK);
    if (sprite.createSprite(240, 320) == nullptr) {
        Serial.println("[FATAL] Failed to create TFT sprite!");
        led_blink_halt(100);
    }

    // ── Create FreeRTOS Tasks ──────────────────────────────────────────
    /*
     * CORE PINNING STRATEGY:
     *
     * Core 1 (Audio Task) — Highest priority
     *   The audio pipeline must run uninterrupted to avoid I2S buffer
     *   underflows (audible glitches). By giving it the highest priority
     *   on Core 1, it preempts the Arduino loop() task, which we idle.
     *
     * Core 0 (System Task) — Low priority
     *   Button polling and serial output are not time-critical.  Core 0
     *   is also used by WiFi/BT (which we don't use), so it's free for
     *   our system task.
     */
    xTaskCreatePinnedToCore(
        audio_task,                     // Task function
        "AudioTask",                    // Name
        AUDIO_TASK_STACK,               // Stack: 16 KB (helix uses ~6 KB stack)
        NULL,                           // Parameter
        configMAX_PRIORITIES - 1,       // Highest priority
        &audio_task_handle,             // Handle
        1                               // ◀── Core 1 (audio core)
    );

    xTaskCreatePinnedToCore(
        system_task,                    // Task function
        "SystemTask",                   // Name
        SYSTEM_TASK_STACK,              // Stack: 8 KB
        NULL,                           // Parameter
        2,                              // Low priority
        &system_task_handle,            // Handle
        0                               // ◀── Core 0 (system core)
    );

    // ── Auto-play first track ──────────────────────────────────────────
    CommandMsg play_msg = { CMD_PLAY, 0 };
    xQueueSend(cmd_queue, &play_msg, portMAX_DELAY);

    Serial.println();
    Serial.println("═══════════════════════════════════════");
    Serial.println("  Initialization complete. Playing...");
    Serial.println("═══════════════════════════════════════");
    Serial.printf("PSRAM remaining: %u bytes\n", ESP.getFreePsram());
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  LOOP  (runs on Core 1 — we idle it since audio_task owns Core 1)
 * ═══════════════════════════════════════════════════════════════════════════ */
void loop() {
    /*
     * The Arduino loop() task runs on Core 1 at priority 1.
     * Our audio task also runs on Core 1 at the highest priority,
     * so it always preempts loop().
     *
     * We simply sleep forever here — all real work happens in
     * the two FreeRTOS tasks created in setup().
     */
    vTaskDelay(portMAX_DELAY);
}
