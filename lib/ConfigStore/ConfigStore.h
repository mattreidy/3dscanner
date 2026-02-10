// ==========================================================================
// ConfigStore.h — WiFi Credential Persistence via NVS
// ==========================================================================
//
// Stores and retrieves WiFi configuration in the ESP32's Non-Volatile
// Storage (NVS). NVS is a key-value store built into the ESP32's flash
// that survives reboots and power cycles — like EEPROM but wear-leveled
// and organized by namespace.
//
// This module stores: SSID, password, DHCP flag, and optional static
// IP configuration (IP, gateway, subnet, DNS). The "wifi" namespace
// keeps our keys separate from other libraries that might also use NVS.
// ==========================================================================

#pragma once
#include <Arduino.h>
#include <Preferences.h>  // ESP32 NVS wrapper — simpler than raw nvs_* API

// Holds everything needed to connect to a WiFi network.
// Supports both DHCP (automatic IP) and static IP configuration.
struct WiFiConfig {
    String ssid;           // Network name to connect to
    String password;       // WPA2 password (empty for open networks)
    bool useDHCP;          // true = get IP from router, false = use static fields below
    String staticIP;       // e.g., "192.168.1.100"
    String staticGateway;  // e.g., "192.168.1.1"
    String staticSubnet;   // e.g., "255.255.255.0"
    String staticDNS;      // e.g., "8.8.8.8"
};

class ConfigStore {
public:
    void begin();                          // Open the NVS namespace
    void save(const WiFiConfig& config);   // Write all fields to NVS
    WiFiConfig load();                     // Read all fields from NVS
    void clear();                          // Erase all stored credentials
    bool hasCredentials();                 // Check if an SSID is saved

private:
    Preferences _prefs;  // ESP32 Preferences API handle (wraps NVS)

    // NVS namespace — all our keys live under "wifi" to avoid
    // collisions with other libraries using NVS.
    static constexpr const char* NAMESPACE = "wifi";
};
