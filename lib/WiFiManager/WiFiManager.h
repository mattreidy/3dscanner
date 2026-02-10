// ==========================================================================
// WiFiManager.h — WiFi State Machine
// ==========================================================================
//
// Manages the ESP32's WiFi in two modes:
//
// 1. AP Mode (Access Point): The ESP32 creates its own WiFi network
//    named "3DScanner-XXXX" (where XXXX comes from the chip's MAC).
//    Users connect to this network to configure WiFi credentials
//    via the captive portal. Uses WIFI_AP_STA mode internally so
//    the STA radio stays active for network scanning.
//
// 2. STA Mode (Station): The ESP32 connects to an existing WiFi
//    network as a client, just like a phone or laptop would.
//    Supports both DHCP and static IP configuration.
//
// WiFi scanning is asynchronous (non-blocking) to avoid starving
// the async web server. The scan uses a 3-state machine:
//   IDLE → SCANNING → DONE → (results consumed) → IDLE
//
// Key gotcha: WiFi.mode(WIFI_AP) disables the STA radio entirely,
// making WiFi.scanNetworks() return 0 or -1. That's why we use
// WIFI_AP_STA instead — both radios stay active.
// ==========================================================================

#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <vector>
#include <functional>
#include <map>
#include "ConfigStore.h"

// WiFi connection state machine
enum class WiFiState {
    IDLE,            // Not connected, not in AP mode
    AP_MODE,         // Running as access point (captive portal)
    STA_CONNECTING,  // Attempting to connect to a network
    STA_CONNECTED    // Connected to a network as a station
};

// Info about a discovered WiFi network (used in scan results)
struct NetworkInfo {
    String ssid;    // Network name
    int32_t rssi;   // Signal strength in dBm (closer to 0 = stronger)
    bool secure;    // true if network requires a password
};

// Async scan state machine
enum class ScanState {
    IDLE,      // No scan in progress, ready to start one
    SCANNING,  // WiFi.scanNetworks(async=true) is running in background
    DONE       // Results are ready to read via getScanResults()
};

class ScannerWiFiManager {
public:
    void begin(ConfigStore* store);
    bool connectSTA(const WiFiConfig& config, uint32_t timeoutMs = 10000);
    void startAP();
    void stopAP();

    // Async WiFi scan workflow:
    //   1. Call startScan() to kick off a background scan
    //   2. Poll getScanState() until it returns DONE
    //   3. Read results from getScanResults()
    //   4. State resets to IDLE when you call startScan() again
    bool startScan();
    ScanState getScanState();
    std::vector<NetworkInfo>& getScanResults();

    // Getters for current WiFi state and connection info
    WiFiState getState() const;
    String getAPSSID() const;
    String getIP() const;
    String getConnectedSSID() const;
    int32_t getRSSI() const;

private:
    ConfigStore* _store = nullptr;
    WiFiState _state = WiFiState::IDLE;
    String _apSSID;
    void generateAPSSID();  // Creates "3DScanner-XXXX" from MAC address

    ScanState _scanState = ScanState::IDLE;
    std::vector<NetworkInfo> _scanResults;
    void collectScanResults();  // Deduplicates and sorts raw scan results
};
