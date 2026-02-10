// ==========================================================================
// main.cpp — 3DScanner Firmware Entry Point
// ==========================================================================
//
// This is the main firmware file for the ESP32-S3 based 3D scanner.
// It handles:
//   1. Boot sequence — chip diagnostics, IMU init, WiFi, web server
//   2. CPU utilization measurement — per-core idle counters with
//      self-calibrating rolling-max baseline
//   3. BNO085 IMU polling — quaternion-based orientation at 100Hz
//   4. SSE data push — IMU at 20Hz, device metrics at 1Hz
//   5. WiFi auto-reconnect — retries 3 times, then falls back to AP mode
//
// The ESP32-S3 is a dual-core chip. Core 0 runs WiFi/networking (and our
// async web server). Core 1 runs this Arduino loop(). We measure CPU usage
// on both cores independently using FreeRTOS idle counter tasks.
// ==========================================================================

#include <Arduino.h>
#include <LittleFS.h>
#include <WiFi.h>
#include <Wire.h>
#include <SparkFun_BNO08x_Arduino_Library.h>
#include "ConfigStore.h"
#include "WiFiManager.h"
#include "WebPortal.h"

// Global instances of our three main modules.
// These persist for the lifetime of the firmware and are passed
// by pointer to each other during setup().
ConfigStore configStore;
ScannerWiFiManager wifiManager;
WebPortal webPortal;

// Timing trackers for periodic tasks in loop().
// Using uint32_t with millis() gives ~49 days before overflow,
// and the subtraction trick (now - last >= interval) handles
// overflow correctly thanks to unsigned arithmetic.
static uint32_t lastHeartbeat = 0;
static uint32_t lastReconnectCheck = 0;
static uint8_t reconnectFailures = 0;
static uint32_t lastImuPush = 0;
static uint32_t lastDevicePush = 0;

// ==========================================================================
// Per-Core CPU Utilization — Idle Counter Technique
// ==========================================================================
//
// How it works:
//   - We create a FreeRTOS task at priority 0 (lowest) on each core.
//   - Each task does nothing but increment a counter in a tight loop.
//   - When the core is idle (no higher-priority work), the counter
//     increments rapidly (~1000/sec due to vTaskDelay(0) context switches).
//   - When the core is busy, higher-priority tasks preempt the counter,
//     so it increments less.
//
// Self-calibrating rolling-max baseline:
//   - We track the HIGHEST idle count ever observed per core.
//   - That becomes the "100% idle" reference (baseline).
//   - Usage = (1 - count/baseline) * 100%.
//   - If count exceeds baseline (scheduler conditions changed), we
//     update the baseline upward — never shows negative usage.
//
// Why not static calibration?
//   - ESP32's WiFi stack changes FreeRTOS scheduler behavior at runtime.
//   - A baseline measured during setup() doesn't match runtime conditions.
//   - Core 0's idle count can INCREASE after WiFi init completes,
//     exceeding a static baseline and showing 0% incorrectly.
// ==========================================================================

static volatile uint32_t idleCount0 = 0;  // core 0 idle counter
static volatile uint32_t idleCount1 = 0;  // core 1 idle counter
static uint32_t idleBaseline0 = 0;        // highest idle count seen on core 0
static uint32_t idleBaseline1 = 0;        // highest idle count seen on core 1
static float cpuUsage0 = 0.0f;            // core 0 usage percentage (0-100)
static float cpuUsage1 = 0.0f;            // core 1 usage percentage (0-100)

// These run on their respective cores at priority 0 (below everything else).
// vTaskDelay(0) yields to the scheduler without sleeping — it lets
// higher-priority tasks run, then resumes counting immediately.
// Without the yield, the counter would starve the actual idle task
// and mess up FreeRTOS internal accounting.
void idleCounterTask0(void* param) {
    for (;;) { idleCount0++; vTaskDelay(0); }
}

void idleCounterTask1(void* param) {
    for (;;) { idleCount1++; vTaskDelay(0); }
}

// Called once per second from loop() to compute CPU usage.
// Reads and resets counters atomically enough for our purposes
// (a few missed increments don't matter at 1-second resolution).
void sampleCpuUsage() {
    // Snapshot and reset the counters
    uint32_t count0 = idleCount0;
    uint32_t count1 = idleCount1;
    idleCount0 = 0;
    idleCount1 = 0;

    // First sample after boot: use as initial baseline, skip reporting.
    // The first interval is always partial (counter started mid-second),
    // so it's not a reliable measurement.
    if (idleBaseline0 == 0 && count0 > 0) {
        idleBaseline0 = count0;
        idleBaseline1 = count1;
        return;
    }

    // Rolling max: if the core was MORE idle this second than ever before,
    // adopt this as the new "100% idle" reference. This handles scheduler
    // behavior changes (e.g., WiFi going quiet, tasks completing).
    if (count0 > idleBaseline0) idleBaseline0 = count0;
    if (count1 > idleBaseline1) idleBaseline1 = count1;

    // Usage = (1 - idle_fraction) * 100%
    // Clamped to 0% minimum (can't have negative usage).
    if (idleBaseline0 > 0) {
        cpuUsage0 = (1.0f - (float)count0 / (float)idleBaseline0) * 100.0f;
        if (cpuUsage0 < 0.0f) cpuUsage0 = 0.0f;
    }
    if (idleBaseline1 > 0) {
        cpuUsage1 = (1.0f - (float)count1 / (float)idleBaseline1) * 100.0f;
        if (cpuUsage1 < 0.0f) cpuUsage1 = 0.0f;
    }
}

// Accessors called by WebPortal to include CPU data in SSE events
float getCpuUsage0() { return cpuUsage0; }
float getCpuUsage1() { return cpuUsage1; }

// ==========================================================================
// BNO085 IMU — 9-DOF Inertial Measurement Unit
// ==========================================================================
//
// The BNO085 runs sensor fusion internally (accelerometer + gyroscope +
// magnetometer) and outputs a rotation vector as a unit quaternion.
// This gives us absolute orientation without needing to do any math
// on the ESP32 — the BNO085's ARM Cortex-M0+ handles it at 100Hz.
//
// Communication: I2C at 400kHz for init (conservative), bumped to
// 1MHz for runtime (the BNO085 supports it and we need the bandwidth).
//
// The quaternion format is [w, x, y, z] where w is the scalar part.
// A quaternion of [1, 0, 0, 0] means "no rotation" (identity).
// ==========================================================================

#define IMU_SDA 8   // ESP32-S3 GPIO8 for I2C data
#define IMU_SCL 9   // ESP32-S3 GPIO9 for I2C clock
BNO08x imu;
static bool imuReady = false;
static float imuQuat[4] = {1.0f, 0.0f, 0.0f, 0.0f}; // w, x, y, z (identity)
static float imuAccuracy = 0.0f; // radians — how confident the BNO085 is

// Accessors for WebPortal's SSE streaming
bool isImuReady() { return imuReady; }
void getImuQuat(float* q) { memcpy(q, imuQuat, sizeof(imuQuat)); }
float getImuAccuracy() { return imuAccuracy; }

// Scans the I2C bus for devices, then tries to connect to the BNO085
// at its two possible addresses (0x4A default, 0x4B alternate).
// Enables the rotation vector report at 100Hz (10ms interval).
void initIMU() {
    Serial.println("[IMU] Initializing I2C on SDA=8, SCL=9...");
    Wire.begin(IMU_SDA, IMU_SCL);
    Wire.setClock(400000); // 400kHz — safe speed for initialization

    // I2C bus scan: helps debug wiring issues.
    // Every I2C device responds to its address with an ACK.
    // If nothing ACKs, we know the bus is dead before wasting
    // time trying to talk to the BNO085.
    Serial.println("[IMU] Scanning I2C bus...");
    int found = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.printf("[IMU]   Found device at 0x%02X\n", addr);
            found++;
        }
    }
    if (found == 0) {
        Serial.println("[IMU]   *** No I2C devices found! Check wiring. ***");
        return;
    }

    // Try both possible BNO085 I2C addresses.
    // Address depends on the state of the DI pin on the breakout board.
    Serial.println("[IMU] Connecting to BNO085...");
    if (imu.begin(0x4A, Wire)) {
        Serial.println("[IMU] BNO085 found at 0x4A");
    } else if (imu.begin(0x4B, Wire)) {
        Serial.println("[IMU] BNO085 found at 0x4B");
    } else {
        Serial.println("[IMU] *** BNO085 not found at 0x4A or 0x4B! ***");
        return;
    }

    // Enable the rotation vector report.
    // The BNO085 supports many report types (accelerometer, gyro, etc.)
    // but rotation vector gives us the fused quaternion directly.
    // 10ms interval = 100Hz output rate.
    if (imu.enableRotationVector(10)) {
        Serial.println("[IMU] Rotation vector enabled at 100Hz");
    } else {
        Serial.println("[IMU] *** Failed to enable rotation vector ***");
        return;
    }

    // Now that init is done, crank up I2C speed for runtime.
    // 1MHz (Fast Mode Plus) gives us enough bandwidth for 100Hz
    // quaternion reads without blocking loop() for too long.
    Wire.setClock(1000000);
    imuReady = true;
    Serial.println("[IMU] Ready");
}

// Called every loop() iteration to read the latest quaternion.
// Non-blocking: getSensorEvent() returns false if no new data is ready.
// The BNO085 can spontaneously reset (power glitch, I2C error) —
// wasReset() detects this and re-enables our report.
void readIMU() {
    if (!imuReady) return;
    if (imu.wasReset()) {
        Serial.println("[IMU] Sensor was reset, re-enabling reports...");
        imu.enableRotationVector(10);
    }
    if (imu.getSensorEvent()) {
        if (imu.getSensorEventID() == SENSOR_REPORTID_ROTATION_VECTOR) {
            imuQuat[0] = imu.getQuatReal();
            imuQuat[1] = imu.getQuatI();
            imuQuat[2] = imu.getQuatJ();
            imuQuat[3] = imu.getQuatK();
            imuAccuracy = imu.getQuatRadianAccuracy();
        }
    }
}

// ==========================================================================
// WiFi Event Handler
// ==========================================================================
// Arduino-ESP32 fires these events asynchronously on WiFi state changes.
// We log them all for serial debugging — invaluable when troubleshooting
// connection issues in the field.
void onWiFiEvent(WiFiEvent_t event) {
    switch (event) {
        case ARDUINO_EVENT_WIFI_AP_STACONNECTED:
            Serial.println("[WiFi-Event] Client CONNECTED to AP");
            Serial.printf("[WiFi-Event] Total AP clients: %d\n", WiFi.softAPgetStationNum());
            break;
        case ARDUINO_EVENT_WIFI_AP_STADISCONNECTED:
            Serial.println("[WiFi-Event] Client DISCONNECTED from AP");
            Serial.printf("[WiFi-Event] Total AP clients: %d\n", WiFi.softAPgetStationNum());
            break;
        case ARDUINO_EVENT_WIFI_AP_STAIPASSIGNED:
            Serial.println("[WiFi-Event] AP assigned IP to client");
            break;
        case ARDUINO_EVENT_WIFI_STA_CONNECTED:
            Serial.println("[WiFi-Event] STA connected to AP");
            break;
        case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
            Serial.println("[WiFi-Event] STA disconnected from AP");
            break;
        case ARDUINO_EVENT_WIFI_STA_GOT_IP:
            Serial.printf("[WiFi-Event] STA got IP: %s\n", WiFi.localIP().toString().c_str());
            break;
        default:
            Serial.printf("[WiFi-Event] Event ID: %d\n", event);
            break;
    }
}

// ==========================================================================
// setup() — Runs once at boot
// ==========================================================================
// Initialization order matters:
//   1. Serial (so we can log everything that follows)
//   2. IMU (I2C init before WiFi, which can interfere with GPIO)
//   3. CPU idle counters (start counting ASAP for accurate baselines)
//   4. WiFi event handler (before any WiFi operations)
//   5. LittleFS check (verify web UI files exist)
//   6. ConfigStore (load saved WiFi credentials from NVS)
//   7. WiFi connect or AP fallback
//   8. WebPortal (HTTP server + captive portal if in AP mode)
void setup() {
    Serial.begin(115200);
    delay(1000); // Wait for serial monitor to connect
    Serial.println("\n\n========================================");
    Serial.println("       3DScanner Booting");
    Serial.println("========================================");
    Serial.printf("[Boot] Chip: %s  Rev: %d\n", ESP.getChipModel(), ESP.getChipRevision());
    Serial.printf("[Boot] CPU Freq: %d MHz\n", ESP.getCpuFreqMHz());
    Serial.printf("[Boot] Free Heap: %d bytes\n", ESP.getFreeHeap());
    Serial.printf("[Boot] Flash Size: %d bytes\n", ESP.getFlashChipSize());
    Serial.printf("[Boot] SDK: %s\n", ESP.getSdkVersion());

    // Initialize IMU early — I2C is sensitive to GPIO state changes
    // that WiFi initialization can cause
    initIMU();

    // Start idle counter tasks on both cores at priority 0 (lowest).
    // 1024 bytes of stack is plenty — these tasks just increment a counter.
    // Pinned to specific cores so we measure each core independently.
    xTaskCreatePinnedToCore(idleCounterTask0, "idleCnt0", 1024, NULL, 0, NULL, 0);
    xTaskCreatePinnedToCore(idleCounterTask1, "idleCnt1", 1024, NULL, 0, NULL, 1);
    Serial.println("[Boot] CPU idle counter tasks started (self-calibrating)");

    // Register WiFi event handler before any WiFi operations
    WiFi.onEvent(onWiFiEvent);
    Serial.println("[Boot] WiFi event handler registered");

    // Quick LittleFS sanity check: mount, list files, unmount.
    // WebPortal will re-mount it when it starts.
    // This early check catches "forgot to upload filesystem" errors
    // before they become mysterious 404s.
    Serial.println("[Boot] Mounting LittleFS...");
    if (LittleFS.begin(true)) {
        Serial.println("[Boot] LittleFS mounted OK");
        File root = LittleFS.open("/");
        if (root && root.isDirectory()) {
            Serial.println("[Boot] LittleFS files:");
            File file = root.openNextFile();
            int fileCount = 0;
            while (file) {
                Serial.printf("[Boot]   /%s  (%d bytes)\n", file.name(), file.size());
                fileCount++;
                file = root.openNextFile();
            }
            if (fileCount == 0) {
                Serial.println("[Boot]   *** NO FILES FOUND! Did you upload filesystem? ***");
                Serial.println("[Boot]   Run: pio run -t uploadfs");
            }
        } else {
            Serial.println("[Boot]   *** Could not open root directory! ***");
        }
        LittleFS.end(); // WebPortal will re-mount it
    } else {
        Serial.println("[Boot] *** LittleFS mount FAILED! ***");
        Serial.println("[Boot] *** Web UI will not work! Run: pio run -t uploadfs ***");
    }

    // Initialize NVS-backed credential storage
    Serial.println("[Boot] Initializing ConfigStore...");
    configStore.begin();

    // WiFi manager generates a unique AP name from the chip's MAC address
    Serial.println("[Boot] Initializing WiFiManager...");
    wifiManager.begin(&configStore);

    // If we have saved WiFi credentials, try to connect as a station (STA).
    // If that fails (wrong password, network gone, etc.), fall through
    // to AP mode so the user can reconfigure.
    bool connected = false;
    if (configStore.hasCredentials()) {
        WiFiConfig config = configStore.load();
        Serial.printf("[Boot] Found saved network: '%s'\n", config.ssid.c_str());
        connected = wifiManager.connectSTA(config);
    } else {
        Serial.println("[Boot] No saved credentials found");
    }

    // AP mode: the device becomes its own WiFi network ("3DScanner-XXXX")
    // so users can connect and configure it via the captive portal.
    if (!connected) {
        Serial.println("[Boot] Starting AP mode...");
        wifiManager.startAP();
    }

    // Start the web server. In AP mode, it also starts a DNS server
    // that redirects ALL domain lookups to 192.168.4.1, which is how
    // captive portals work — the phone/laptop thinks it needs to sign in.
    Serial.println("[Boot] Starting WebPortal...");
    webPortal.begin(&wifiManager, &configStore);

    Serial.println("========================================");
    Serial.println("       3DScanner Ready");
    Serial.println("========================================");
    Serial.printf("[Boot] Mode: %s\n",
                  (wifiManager.getState() == WiFiState::AP_MODE) ? "AP" : "STA");
    Serial.printf("[Boot] IP: %s\n", wifiManager.getIP().c_str());
    if (wifiManager.getState() == WiFiState::AP_MODE) {
        Serial.printf("[Boot] AP SSID: %s\n", wifiManager.getAPSSID().c_str());
        Serial.println("[Boot] Connect to this WiFi network, then browse to 192.168.4.1");
    }
    Serial.printf("[Boot] Free Heap: %d bytes\n", ESP.getFreeHeap());

    Serial.println("========================================\n");
}

// ==========================================================================
// loop() — Main loop, runs on core 1
// ==========================================================================
// This runs continuously after setup(). It handles:
//   - DNS processing for captive portal (must be called frequently)
//   - IMU quaternion reads (as fast as available, ~100Hz)
//   - SSE push of IMU data at 20Hz (every 50ms)
//   - SSE push of device metrics at 1Hz (every 1000ms)
//   - WiFi reconnection check every 10 seconds
//   - Serial heartbeat every 30 seconds
//
// The delay(1) at the end is CRITICAL: without it, loop() runs at
// priority 1 and starves the idle counter task (priority 0) on core 1,
// making CPU measurement show 100% usage even when the core is idle.
void loop() {
    // Process DNS requests for captive portal (no-op if DNS isn't running)
    webPortal.loop();

    // Poll the BNO085 for new quaternion data.
    // Returns immediately if no new data is available.
    readIMU();

    uint32_t now = millis();

    // Push IMU quaternion to all connected SSE clients at ~20Hz.
    // 50ms interval matches a good balance between responsiveness
    // and bandwidth. The raw sensor runs at 100Hz but 20Hz is
    // smooth enough for the 3D visualization and saves bandwidth.
    if (now - lastImuPush >= 50) {
        lastImuPush = now;
        webPortal.sendIMU();
    }

    // Sample CPU usage and push device metrics at 1Hz.
    // CPU must be sampled BEFORE sending — otherwise we'd send
    // stale data from the previous second.
    if (now - lastDevicePush >= 1000) {
        lastDevicePush = now;
        sampleCpuUsage();
        webPortal.sendDevice();
    }

    // WiFi auto-reconnect: only in STA mode, checks every 10 seconds.
    // If WiFi drops, we try WiFi.reconnect() with a 15-second timeout.
    // After 3 consecutive failures, we give up and fall back to AP mode
    // so the user can reconfigure (maybe the router moved/changed).
    if (wifiManager.getState() == WiFiState::STA_CONNECTED && now - lastReconnectCheck >= 10000) {
        lastReconnectCheck = now;
        if (WiFi.status() != WL_CONNECTED) {
            Serial.println("[Reconnect] WiFi connection lost, attempting reconnect...");
            WiFi.reconnect();

            // Blocking wait — acceptable here since we've already lost WiFi
            // and can't serve web clients anyway.
            uint32_t start = millis();
            while (WiFi.status() != WL_CONNECTED && (millis() - start) < 15000) {
                delay(500);
                Serial.print(".");
            }
            Serial.println();

            if (WiFi.status() == WL_CONNECTED) {
                Serial.printf("[Reconnect] Reconnected! IP: %s\n", WiFi.localIP().toString().c_str());
                reconnectFailures = 0;
            } else {
                reconnectFailures++;
                Serial.printf("[Reconnect] Failed (attempt %d/3)\n", reconnectFailures);
                if (reconnectFailures >= 3) {
                    Serial.println("[Reconnect] 3 failures, falling back to AP mode...");
                    wifiManager.startAP();
                    webPortal.restartCaptivePortal();
                    reconnectFailures = 0;
                }
            }
        } else {
            reconnectFailures = 0; // reset counter on healthy connection
        }
    }

    // Heartbeat: serial diagnostic output every 30 seconds.
    // Useful for monitoring device health over serial without
    // needing the web dashboard.
    if (now - lastHeartbeat >= 30000) {
        lastHeartbeat = now;
        Serial.printf("[Heartbeat] Uptime: %lus | Heap: %d | CPU0: %.1f%% CPU1: %.1f%% | IMU: %s | WiFi: %s",
                      now / 1000, ESP.getFreeHeap(), cpuUsage0, cpuUsage1,
                      imuReady ? "OK" : "N/A",
                      (wifiManager.getState() == WiFiState::AP_MODE) ? "AP" :
                      (wifiManager.getState() == WiFiState::STA_CONNECTED) ? "STA" : "IDLE");
        if (wifiManager.getState() == WiFiState::AP_MODE) {
            Serial.printf(" | AP Clients: %d", WiFi.softAPgetStationNum());
        }
        Serial.println();
    }

    // Yield core 1 so the idle counter task (priority 0) can run.
    // Without this, loop() at priority 1 monopolizes core 1 and
    // the idle counter never increments, showing 100% CPU usage.
    delay(1);
}
