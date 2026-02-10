// ==========================================================================
// WebPortal.h — Async Web Server + SSE Streaming + Captive Portal
// ==========================================================================
//
// This module runs the entire web-facing side of the 3DScanner:
//
// 1. Static file serving — serves index.html, style.css, app.js from
//    LittleFS (the ESP32's flash filesystem for web assets).
//
// 2. REST API — endpoints for WiFi scanning, status, device info,
//    connecting, and disconnecting (see setupRoutes()).
//
// 3. Server-Sent Events (SSE) — a persistent one-way HTTP stream
//    at /api/events that pushes IMU quaternions at 20Hz and device
//    metrics at 1Hz. SSE is simpler than WebSockets and works great
//    for server→client data flow.
//
// 4. Captive Portal — in AP mode, a DNS server intercepts ALL domain
//    lookups and redirects them to 192.168.4.1. Combined with OS-specific
//    detection endpoints (/generate_204 for Android, /hotspot-detect.html
//    for iOS, etc.), this auto-opens the config page on most devices.
//
// Uses ESPAsyncWebServer for non-blocking HTTP handling. The web server
// runs on core 0 (via async_tcp) while the main loop runs on core 1.
// ==========================================================================

#pragma once

#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <DNSServer.h>
#include "WiFiManager.h"
#include "ConfigStore.h"

// These functions are defined in main.cpp and give us access to
// CPU usage and IMU data for the SSE streams and device info API.
extern float getCpuUsage0();
extern float getCpuUsage1();
extern bool isImuReady();
extern void getImuQuat(float* q);
extern float getImuAccuracy();

class WebPortal {
public:
    void begin(ScannerWiFiManager* wifi, ConfigStore* store);
    void loop(); // Must be called in main loop() for DNS processing
    void restartCaptivePortal(); // Called from main on reconnect failure → AP fallback
    void sendIMU();    // Push IMU quaternion via SSE at ~20Hz
    void sendDevice(); // Push device/system info via SSE at ~1Hz

private:
    AsyncWebServer _server{80};          // HTTP server on port 80
    AsyncEventSource _events{"/api/events"}; // SSE endpoint
    DNSServer _dns;                       // DNS server for captive portal
    ScannerWiFiManager* _wifi = nullptr;
    ConfigStore* _store = nullptr;
    bool _dnsStarted = false;             // Track if DNS is running

    void setupRoutes();
    void startCaptivePortal();
    void stopCaptivePortal();

    // API route handlers
    void handleScan(AsyncWebServerRequest* request);
    void handleStatus(AsyncWebServerRequest* request);
    void handleDevice(AsyncWebServerRequest* request);
    void handleConnect(AsyncWebServerRequest* request);
    void handleDisconnect(AsyncWebServerRequest* request);
    String buildDeviceJson(); // Shared between HTTP GET and SSE push

    // Captive portal redirect URL (built from AP IP, avoids hardcoding)
    String _captiveRedirectUrl;
    void buildCaptiveRedirectUrl();

    // POST body accumulation buffer for /api/connect.
    // ESPAsyncWebServer may deliver the body in multiple chunks —
    // we accumulate them here and process in the request handler.
    String _connectBody;
};
