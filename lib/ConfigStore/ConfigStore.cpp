// ==========================================================================
// ConfigStore.cpp — WiFi Credential Persistence Implementation
// ==========================================================================
//
// Uses the ESP32 Preferences API (a friendly wrapper around NVS).
// Each field gets its own key in the "wifi" namespace:
//   "ssid", "pass", "dhcp", "ip", "gw", "sn", "dns"
//
// NVS keys are limited to 15 characters, hence the short names.
// String values can be up to 4000 bytes (plenty for an SSID + password).
// ==========================================================================

#include "ConfigStore.h"

// Opens the "wifi" namespace in NVS. The `false` parameter means
// read-write mode (as opposed to read-only). Must be called once
// during setup() before any load/save/clear operations.
void ConfigStore::begin() {
    bool ok = _prefs.begin(NAMESPACE, false);
    Serial.printf("[Config] NVS namespace '%s' opened: %s\n", NAMESPACE, ok ? "OK" : "FAILED");
}

// Reads all stored WiFi configuration from NVS.
// Each getString/getBool call specifies a default value that's
// returned if the key doesn't exist (e.g., first boot).
// Default for DHCP is true (most networks use it).
// Default subnet is 255.255.255.0 (standard /24 network).
WiFiConfig ConfigStore::load() {
    WiFiConfig config;
    config.ssid = _prefs.getString("ssid", "");
    config.password = _prefs.getString("pass", "");
    config.useDHCP = _prefs.getBool("dhcp", true);
    config.staticIP = _prefs.getString("ip", "");
    config.staticGateway = _prefs.getString("gw", "");
    config.staticSubnet = _prefs.getString("sn", "255.255.255.0");
    config.staticDNS = _prefs.getString("dns", "");

    // Log what we loaded (password length only, not the actual password)
    Serial.printf("[Config] Loaded: ssid='%s' pass=%d_chars dhcp=%s ip='%s'\n",
                  config.ssid.c_str(), config.password.length(),
                  config.useDHCP ? "yes" : "no", config.staticIP.c_str());
    return config;
}

// Writes WiFi configuration to NVS. Each putString/putBool call
// writes to flash immediately — no explicit "commit" needed.
// NVS handles wear leveling internally so we don't need to worry
// about flash write endurance for infrequent config saves.
void ConfigStore::save(const WiFiConfig& config) {
    Serial.printf("[Config] Saving: ssid='%s' pass=%d_chars dhcp=%s\n",
                  config.ssid.c_str(), config.password.length(),
                  config.useDHCP ? "yes" : "no");

    _prefs.putString("ssid", config.ssid);
    _prefs.putString("pass", config.password);
    _prefs.putBool("dhcp", config.useDHCP);
    _prefs.putString("ip", config.staticIP);
    _prefs.putString("gw", config.staticGateway);
    _prefs.putString("sn", config.staticSubnet);
    _prefs.putString("dns", config.staticDNS);

    Serial.println("[Config] Save complete");
}

// Erases all keys in the "wifi" namespace.
// Called when the user hits "Forget Network" in the web UI.
// After clearing, the device will boot into AP mode on next restart
// since hasCredentials() will return false.
void ConfigStore::clear() {
    Serial.println("[Config] Clearing all stored credentials...");
    _prefs.clear();
    Serial.println("[Config] Cleared");
}

// Quick check: is there a saved SSID? An empty SSID means
// no credentials have been saved (or they were cleared).
// Used during boot to decide: connect to saved network vs. start AP.
bool ConfigStore::hasCredentials() {
    String ssid = _prefs.getString("ssid", "");
    bool has = ssid.length() > 0;
    Serial.printf("[Config] hasCredentials: %s (ssid='%s')\n", has ? "YES" : "NO", ssid.c_str());
    return has;
}
