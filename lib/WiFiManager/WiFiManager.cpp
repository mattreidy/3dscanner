// ==========================================================================
// WiFiManager.cpp — WiFi State Machine Implementation
// ==========================================================================
//
// Handles AP mode, STA mode, and async WiFi scanning.
//
// Important ESP32 WiFi gotchas handled here:
//   - WIFI_AP disables STA radio → use WIFI_AP_STA for scanning in AP mode
//   - WiFi.scanNetworks(false) blocks and starves async_tcp → use async scan
//   - Switching AP→STA with open TCP sockets crashes lwIP → reboot instead
//   - Duplicate SSIDs from multi-AP networks → deduplicate by strongest RSSI
// ==========================================================================

#include "WiFiManager.h"
#include <WiFi.h>
#include <vector>
#include <algorithm>

// Initialize the WiFi manager: set hostname, generate unique AP name.
// Called once during setup().
void ScannerWiFiManager::begin(ConfigStore* store) {
    _store = store;
    generateAPSSID();
    WiFi.setHostname("3dscanner");  // Shows up in router's client list
    Serial.printf("[WiFi] Hostname set to '3dscanner'\n");
    Serial.printf("[WiFi] Generated AP SSID: '%s'\n", _apSSID.c_str());
    Serial.printf("[WiFi] MAC: %s\n", WiFi.macAddress().c_str());
}

// Generates a unique AP name like "3DScanner-A1B2" from the chip's
// factory-programmed MAC address (stored in eFuse, unique per chip).
// We use the upper 16 bits of the 48-bit MAC for a short hex suffix.
void ScannerWiFiManager::generateAPSSID() {
    uint64_t mac = ESP.getEfuseMac();
    uint16_t id = (uint16_t)(mac >> 32);  // Upper 16 bits of MAC
    char buf[20];
    snprintf(buf, sizeof(buf), "3DScanner-%04X", id);
    _apSSID = String(buf);
}

// Connect to a WiFi network as a station (client).
// Supports both DHCP and static IP configuration.
// Blocks for up to `timeoutMs` milliseconds waiting for connection.
// Returns true if connected, false if timed out.
bool ScannerWiFiManager::connectSTA(const WiFiConfig& config, uint32_t timeoutMs) {
    _state = WiFiState::STA_CONNECTING;
    Serial.printf("[WiFi] Connecting to '%s'...\n", config.ssid.c_str());
    Serial.printf("[WiFi] Password length: %d\n", config.password.length());

    // STA-only mode: we're connecting to an existing network,
    // don't need AP radio. (AP mode uses AP_STA instead.)
    WiFi.mode(WIFI_STA);
    Serial.println("[WiFi] Mode set to STA");

    // Apply static IP configuration if DHCP is disabled.
    // All four addresses must parse successfully from strings.
    // WiFi.config() must be called BEFORE WiFi.begin().
    if (!config.useDHCP && config.staticIP.length() > 0) {
        IPAddress ip, gateway, subnet, dns;
        bool ipOk = ip.fromString(config.staticIP);
        bool gwOk = gateway.fromString(config.staticGateway);
        bool snOk = subnet.fromString(config.staticSubnet);
        bool dnsOk = dns.fromString(config.staticDNS);
        Serial.printf("[WiFi] Static config parse - IP:%s GW:%s SN:%s DNS:%s\n",
                      ipOk ? "ok" : "FAIL", gwOk ? "ok" : "FAIL",
                      snOk ? "ok" : "FAIL", dnsOk ? "ok" : "FAIL");
        WiFi.config(ip, gateway, subnet, dns);
        Serial.printf("[WiFi] Static IP configured: %s\n", config.staticIP.c_str());
    } else {
        Serial.println("[WiFi] Using DHCP");
    }

    // Start the connection attempt. This is non-blocking — it kicks
    // off the WPA handshake in the background. We poll WiFi.status()
    // below to wait for it to complete.
    WiFi.begin(config.ssid.c_str(), config.password.c_str());
    Serial.println("[WiFi] WiFi.begin() called, waiting for connection...");

    // Poll connection status every 250ms until connected or timeout.
    // We log the status code at each step so connection issues are
    // visible in the serial monitor (e.g., NO_SSID = wrong network name,
    // CONNECT_FAILED = wrong password).
    uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - start) < timeoutMs) {
        delay(250);
        wl_status_t status = (wl_status_t)WiFi.status();
        const char* statusStr;
        switch (status) {
            case WL_IDLE_STATUS:     statusStr = "IDLE"; break;
            case WL_NO_SSID_AVAIL:   statusStr = "NO_SSID"; break;
            case WL_SCAN_COMPLETED:  statusStr = "SCAN_DONE"; break;
            case WL_CONNECTED:       statusStr = "CONNECTED"; break;
            case WL_CONNECT_FAILED:  statusStr = "FAILED"; break;
            case WL_CONNECTION_LOST: statusStr = "LOST"; break;
            case WL_DISCONNECTED:    statusStr = "DISCONNECTED"; break;
            default:                 statusStr = "UNKNOWN"; break;
        }
        Serial.printf(".[%s]", statusStr);
    }
    Serial.println();

    if (WiFi.status() == WL_CONNECTED) {
        _state = WiFiState::STA_CONNECTED;
        Serial.printf("[WiFi] Connected! IP: %s\n", WiFi.localIP().toString().c_str());
        Serial.printf("[WiFi] Gateway: %s\n", WiFi.gatewayIP().toString().c_str());
        Serial.printf("[WiFi] DNS: %s\n", WiFi.dnsIP().toString().c_str());
        Serial.printf("[WiFi] RSSI: %d dBm\n", WiFi.RSSI());
        Serial.printf("[WiFi] Channel: %d\n", WiFi.channel());
        return true;
    }

    // Connection failed — clean up and return to idle state
    Serial.printf("[WiFi] Connection failed! Final status: %d\n", WiFi.status());
    WiFi.disconnect(true);  // true = also erase WiFi credentials from RAM
    _state = WiFiState::IDLE;
    return false;
}

// Start the ESP32 as a WiFi access point for captive portal provisioning.
// Uses WIFI_AP_STA (not WIFI_AP) so the STA radio stays active —
// this is required for WiFi.scanNetworks() to work while in AP mode.
// Without STA, scan returns 0 networks because the radio is off.
void ScannerWiFiManager::startAP() {
    WiFi.mode(WIFI_AP_STA);
    Serial.println("[WiFi] Mode set to AP_STA (AP + STA for scanning)");

    bool apStarted = WiFi.softAP(_apSSID.c_str());
    delay(100);  // Brief delay for AP to fully initialize
    _state = WiFiState::AP_MODE;

    if (apStarted) {
        Serial.printf("[WiFi] AP started: SSID='%s'  IP=%s\n",
                      _apSSID.c_str(), WiFi.softAPIP().toString().c_str());
    } else {
        Serial.println("[WiFi] *** AP failed to start! ***");
    }
}

// Shut down the access point. Called when transitioning to STA mode.
// The `true` parameter disconnects all connected clients first.
void ScannerWiFiManager::stopAP() {
    Serial.println("[WiFi] Stopping AP...");
    WiFi.softAPdisconnect(true);
}

// Kick off an asynchronous WiFi scan.
// WiFi.scanNetworks(true) returns immediately — the scan runs in the
// background on the WiFi task. This is critical because a synchronous
// scan blocks for 1-3 seconds and starves the async web server,
// causing TCP watchdog timeouts and dropped connections.
//
// The 300ms parameter is the per-channel dwell time. Lower = faster
// scan but might miss networks with low beacon rates.
bool ScannerWiFiManager::startScan() {
    if (_scanState == ScanState::SCANNING) {
        Serial.println("[WiFi] Scan already in progress");
        return false;
    }

    Serial.println("[WiFi] Starting async scan...");
    Serial.printf("[WiFi] Current WiFi mode: %d (1=STA, 2=AP, 3=AP_STA)\n", WiFi.getMode());

    _scanResults.clear();
    _scanState = ScanState::SCANNING;

    // async=true: returns immediately, scan runs in background
    int rc = WiFi.scanNetworks(true, false, false, 300);
    Serial.printf("[WiFi] WiFi.scanNetworks(async=true) returned %d\n", rc);
    return true;
}

// Check if the async scan has finished.
// WiFi.scanComplete() returns:
//   WIFI_SCAN_RUNNING (-1): still scanning
//   WIFI_SCAN_FAILED (-2): scan failed
//   >= 0: number of networks found
// When done, we collect and process the results.
ScanState ScannerWiFiManager::getScanState() {
    if (_scanState == ScanState::SCANNING) {
        int16_t result = WiFi.scanComplete();
        if (result == WIFI_SCAN_RUNNING) {
            return ScanState::SCANNING;
        }
        // Scan finished (result >= 0) or failed (result < 0 but not RUNNING)
        Serial.printf("[WiFi] Scan complete, result code: %d\n", result);
        collectScanResults();
        _scanState = ScanState::DONE;
    }
    return _scanState;
}

// Process raw scan results: deduplicate by SSID, keep strongest signal.
//
// Why deduplicate? Enterprise and mesh networks often have multiple
// access points broadcasting the same SSID. Without dedup, "HomeWiFi"
// might appear 3-4 times in the list, confusing the user. We keep
// only the strongest signal for each SSID since that's the AP the
// ESP32 will most likely connect to.
//
// Results are sorted by RSSI (strongest first) for a better UX.
void ScannerWiFiManager::collectScanResults() {
    int n = WiFi.scanComplete();
    _scanResults.clear();

    if (n > 0) {
        Serial.println("[WiFi] Raw scan results:");
        for (int i = 0; i < n; i++) {
            Serial.printf("[WiFi]   [%d] '%s' RSSI:%d Ch:%d %s\n",
                          i, WiFi.SSID(i).c_str(), WiFi.RSSI(i), WiFi.channel(i),
                          WiFi.encryptionType(i) == WIFI_AUTH_OPEN ? "open" : "secured");
        }

        // Deduplicate: use a map keyed by SSID, keeping the entry
        // with the strongest (highest/least negative) RSSI.
        std::map<String, NetworkInfo> seen;
        for (int i = 0; i < n; i++) {
            String ssid = WiFi.SSID(i);
            if (ssid.length() == 0) continue;  // Skip hidden networks
            auto it = seen.find(ssid);
            if (it == seen.end() || WiFi.RSSI(i) > it->second.rssi) {
                seen[ssid] = {ssid, WiFi.RSSI(i), WiFi.encryptionType(i) != WIFI_AUTH_OPEN};
            }
        }

        // Copy to vector and sort by signal strength (strongest first)
        for (auto& pair : seen) {
            _scanResults.push_back(pair.second);
        }
        std::sort(_scanResults.begin(), _scanResults.end(),
                  [](const NetworkInfo& a, const NetworkInfo& b) { return a.rssi > b.rssi; });
    } else if (n == 0) {
        Serial.println("[WiFi] No networks found (0 results)");
    } else {
        Serial.printf("[WiFi] *** Scan error code: %d ***\n", n);
    }

    // Free the scan results memory held by the WiFi driver.
    // Without this, repeated scans leak memory.
    WiFi.scanDelete();
    Serial.printf("[WiFi] Returning %d deduplicated networks\n", _scanResults.size());
}

// Returns a reference to the scan results vector.
// Only valid after getScanState() returns DONE.
std::vector<NetworkInfo>& ScannerWiFiManager::getScanResults() {
    return _scanResults;
}

// --- Simple getters for current WiFi state ---

WiFiState ScannerWiFiManager::getState() const {
    return _state;
}

String ScannerWiFiManager::getAPSSID() const {
    return _apSSID;
}

// Returns the device's current IP address.
// In STA mode: the IP assigned by the router (or static IP).
// In AP mode: always 192.168.4.1 (ESP32's default AP IP).
String ScannerWiFiManager::getIP() const {
    if (_state == WiFiState::STA_CONNECTED) {
        return WiFi.localIP().toString();
    } else if (_state == WiFiState::AP_MODE) {
        return WiFi.softAPIP().toString();
    }
    return "0.0.0.0";
}

String ScannerWiFiManager::getConnectedSSID() const {
    if (_state == WiFiState::STA_CONNECTED) {
        return WiFi.SSID();
    }
    return "";
}

// RSSI (Received Signal Strength Indicator) in dBm.
// Typical values: -30 = excellent, -50 = good, -70 = fair, -90 = unusable.
// Returns 0 when not connected (no signal to measure).
int32_t ScannerWiFiManager::getRSSI() const {
    if (_state == WiFiState::STA_CONNECTED) {
        return WiFi.RSSI();
    }
    return 0;
}
