// ==========================================================================
// WebPortal.cpp — Web Server, REST API, SSE Streaming, Captive Portal
// ==========================================================================
//
// The web server is fully asynchronous (ESPAsyncWebServer) — it handles
// multiple concurrent HTTP requests without blocking the main loop.
// This is critical on the ESP32 where WiFi, I2C, and the main loop
// all share CPU time.
//
// Architecture:
//   - Static files (HTML/CSS/JS) served from LittleFS flash filesystem
//   - REST API for WiFi provisioning (scan, connect, disconnect)
//   - SSE stream pushes real-time data (no polling = lower latency)
//   - Captive portal DNS for automatic config page on phones/laptops
// ==========================================================================

#include "WebPortal.h"
#include <ArduinoJson.h>
#include <LittleFS.h>

// Initialize the web portal: mount filesystem, set up routes, start server.
// If we're in AP mode, also starts the captive portal DNS.
void WebPortal::begin(ScannerWiFiManager* wifi, ConfigStore* store) {
    _wifi = wifi;
    _store = store;

    // Mount LittleFS — this is where index.html, style.css, and app.js live.
    // The `true` parameter formats the filesystem if mount fails (first boot).
    Serial.println("[WebPortal] Mounting LittleFS...");
    if (!LittleFS.begin(true)) {
        Serial.println("[WebPortal] *** LittleFS mount FAILED! ***");
        Serial.println("[WebPortal] *** Web UI will not load. Run: pio run -t uploadfs ***");
    } else {
        Serial.println("[WebPortal] LittleFS mounted OK");
        // Verify the three required web files exist
        const char* requiredFiles[] = {"/index.html", "/style.css", "/app.js"};
        for (auto& f : requiredFiles) {
            if (LittleFS.exists(f)) {
                File file = LittleFS.open(f);
                Serial.printf("[WebPortal]   %s (%d bytes) OK\n", f, file.size());
                file.close();
            } else {
                Serial.printf("[WebPortal]   *** %s MISSING! ***\n", f);
            }
        }
    }

    // Build the captive portal redirect URL from the AP's IP.
    // Must be done before setupRoutes() since captive portal handlers reference it.
    buildCaptiveRedirectUrl();

    setupRoutes();

    // Register the SSE event source as a handler.
    // Clients connect to /api/events and receive a persistent stream.
    _server.addHandler(&_events);
    _server.begin();
    Serial.println("[WebPortal] Web server started on port 80");

    // Start captive portal DNS only in AP mode
    if (_wifi->getState() == WiFiState::AP_MODE) {
        startCaptivePortal();
    }
}

// Must be called in the main loop() — processes DNS requests.
// The DNSServer library doesn't use interrupts, so it needs
// periodic polling. Each call handles one queued DNS request.
void WebPortal::loop() {
    if (_dnsStarted) {
        _dns.processNextRequest();
    }
}

// Build the captive portal redirect URL from the AP's actual IP address.
// Called once when the captive portal starts, avoids hardcoding "192.168.4.1"
// so the portal still works if the AP IP is ever changed.
void WebPortal::buildCaptiveRedirectUrl() {
    _captiveRedirectUrl = "http://" + WiFi.softAPIP().toString() + "/";
    Serial.printf("[WebPortal] Captive redirect URL: %s\n", _captiveRedirectUrl.c_str());
}

// Start the captive portal DNS server.
// Binds to port 53 (standard DNS) and responds to ALL domain queries
// with the AP's IP address. This is the trick that makes phones/laptops
// auto-open the config page — they think any domain resolves to our device,
// so their connectivity check fails and they show the "Sign in to network" prompt.
void WebPortal::startCaptivePortal() {
    IPAddress apIP = WiFi.softAPIP();
    buildCaptiveRedirectUrl();
    Serial.printf("[WebPortal] Starting captive portal DNS -> %s\n", apIP.toString().c_str());
    _dns.start(53, "*", apIP);  // "*" = respond to ALL domain queries
    _dnsStarted = true;
    Serial.println("[WebPortal] Captive portal DNS started (all domains -> AP IP)");
}

void WebPortal::stopCaptivePortal() {
    if (_dnsStarted) {
        _dns.stop();
        _dnsStarted = false;
        Serial.println("[WebPortal] Captive portal DNS stopped");
    }
}

// Called from main.cpp when WiFi reconnection fails and we fall back to AP.
// Restarts the DNS server so the captive portal works again.
void WebPortal::restartCaptivePortal() {
    startCaptivePortal();
}

// ==========================================================================
// HTTP Route Setup
// ==========================================================================
// Sets up all routes in one place:
//   - Static files from LittleFS (/, /index.html, /style.css, /app.js)
//   - REST API (/api/scan, /api/status, /api/device, /api/connect, /api/disconnect)
//   - Captive portal detection endpoints (Android, iOS, Windows, Firefox)
//   - Catch-all 404 handler (redirects to / in AP mode)
void WebPortal::setupRoutes() {
    Serial.println("[WebPortal] Setting up HTTP routes...");

    // Serve static files from LittleFS with no caching.
    // No-cache headers ensure browsers always get fresh JS/CSS after
    // a firmware update. Without this, browsers cache aggressively and
    // users see stale UI after uploading new web files.
    _server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html")
        .setCacheControl("no-cache, no-store, must-revalidate");
    Serial.println("[WebPortal]   Static files: / -> LittleFS (default: index.html, no-cache)");

    // --- REST API Endpoints ---

    // GET /api/scan — Async WiFi network scan.
    // Returns either {scanning:true} (scan in progress) or an array
    // of {ssid, rssi, secure} objects. The web UI polls this endpoint
    // every 500ms until it gets results.
    _server.on("/api/scan", HTTP_GET, [this](AsyncWebServerRequest* request) {
        Serial.printf("[HTTP] %s %s from %s\n", "GET", request->url().c_str(),
                      request->client()->remoteIP().toString().c_str());
        handleScan(request);
    });
    Serial.println("[WebPortal]   Route: GET /api/scan");

    // GET /api/status — WiFi mode, IP, SSID, signal strength.
    // Used once on page load to determine if we're in AP or STA mode.
    _server.on("/api/status", HTTP_GET, [this](AsyncWebServerRequest* request) {
        Serial.printf("[HTTP] %s %s from %s\n", "GET", request->url().c_str(),
                      request->client()->remoteIP().toString().c_str());
        handleStatus(request);
    });
    Serial.println("[WebPortal]   Route: GET /api/status");

    // GET /api/device — Full device info (chip, CPU, memory, network, IMU).
    // Used for initial page load; ongoing updates come via SSE.
    _server.on("/api/device", HTTP_GET, [this](AsyncWebServerRequest* request) {
        Serial.printf("[HTTP] %s %s from %s\n", "GET", request->url().c_str(),
                      request->client()->remoteIP().toString().c_str());
        handleDevice(request);
    });
    Serial.println("[WebPortal]   Route: GET /api/device");

    // POST /api/connect — Save WiFi credentials and reboot.
    // Uses a "body handler" (3rd callback) because ESPAsyncWebServer
    // delivers POST body data separately from the request. The body may
    // arrive in multiple chunks, so we accumulate in _connectBody and
    // process in the request handler (1st callback) which fires after
    // all body data has been received.
    _server.on("/api/connect", HTTP_POST,
        [this](AsyncWebServerRequest* request) {
            // Request handler fires after all body chunks have been received.
            Serial.printf("[HTTP] POST /api/connect from %s (%d bytes)\n",
                          request->client()->remoteIP().toString().c_str(), _connectBody.length());
            handleConnect(request);
        },
        nullptr,  // upload handler (not used)
        [this](AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total) {
            // Body handler: accumulate chunks into _connectBody.
            // index=0 means first chunk — clear the buffer.
            if (index == 0) _connectBody = "";
            _connectBody += String((char*)data, len);
        }
    );
    Serial.println("[WebPortal]   Route: POST /api/connect");

    // POST /api/disconnect — Clear saved credentials and reboot to AP.
    _server.on("/api/disconnect", HTTP_POST, [this](AsyncWebServerRequest* request) {
        Serial.printf("[HTTP] POST /api/disconnect from %s\n",
                      request->client()->remoteIP().toString().c_str());
        handleDisconnect(request);
    });
    Serial.println("[WebPortal]   Route: POST /api/disconnect");

    // --- Captive Portal Detection Endpoints ---
    // Each OS/browser checks a specific URL to test internet connectivity.
    // If the response isn't what's expected, the OS shows a "Sign in"
    // prompt. We redirect all of these to our config page.

    // Android: expects HTTP 204 No Content from /generate_204
    _server.on("/generate_204", HTTP_GET, [this](AsyncWebServerRequest* request) {
        Serial.printf("[Captive] /generate_204 from %s -> redirect\n",
                      request->client()->remoteIP().toString().c_str());
        request->redirect(_captiveRedirectUrl);
    });
    // iOS/macOS: expects "Success" from /hotspot-detect.html
    _server.on("/hotspot-detect.html", HTTP_GET, [this](AsyncWebServerRequest* request) {
        Serial.printf("[Captive] /hotspot-detect.html from %s -> redirect\n",
                      request->client()->remoteIP().toString().c_str());
        request->redirect(_captiveRedirectUrl);
    });
    // Windows: connectivity check via NCSI (Network Connectivity Status Indicator)
    _server.on("/connecttest.txt", HTTP_GET, [this](AsyncWebServerRequest* request) {
        Serial.printf("[Captive] /connecttest.txt from %s -> redirect\n",
                      request->client()->remoteIP().toString().c_str());
        request->redirect(_captiveRedirectUrl);
    });
    _server.on("/redirect", HTTP_GET, [this](AsyncWebServerRequest* request) {
        Serial.printf("[Captive] /redirect from %s -> redirect\n",
                      request->client()->remoteIP().toString().c_str());
        request->redirect(_captiveRedirectUrl);
    });
    // Windows 10/11 NCSI probe
    _server.on("/ncsi.txt", HTTP_GET, [this](AsyncWebServerRequest* request) {
        Serial.printf("[Captive] /ncsi.txt from %s -> redirect\n",
                      request->client()->remoteIP().toString().c_str());
        request->redirect(_captiveRedirectUrl);
    });
    // Firefox: uses its own captive portal detection
    _server.on("/canonical.html", HTTP_GET, [this](AsyncWebServerRequest* request) {
        Serial.printf("[Captive] /canonical.html from %s -> redirect\n",
                      request->client()->remoteIP().toString().c_str());
        request->redirect(_captiveRedirectUrl);
    });
    _server.on("/success.txt", HTTP_GET, [this](AsyncWebServerRequest* request) {
        Serial.printf("[Captive] /success.txt from %s -> redirect\n",
                      request->client()->remoteIP().toString().c_str());
        request->redirect(_captiveRedirectUrl);
    });
    Serial.println("[WebPortal]   Routes: captive portal detection endpoints");

    // Catch-all: any URL not matched above.
    // In AP mode: redirect to config page (helps captive portal work
    // even with OS-specific URLs we haven't explicitly handled).
    // In STA mode: return 404 (normal web server behavior).
    _server.onNotFound([this](AsyncWebServerRequest* request) {
        Serial.printf("[HTTP] 404 %s %s from %s",
                      request->methodToString(), request->url().c_str(),
                      request->client()->remoteIP().toString().c_str());
        if (_wifi->getState() == WiFiState::AP_MODE) {
            Serial.println(" -> redirect to /");
            request->redirect(_captiveRedirectUrl);
        } else {
            Serial.println(" -> 404");
            request->send(404, "text/plain", "Not found");
        }
    });
    Serial.println("[WebPortal]   Route: catch-all 404 handler");
    Serial.println("[WebPortal] All routes configured");
}

// ==========================================================================
// API Handler: /api/scan
// ==========================================================================
// Implements a polling-based async scan workflow:
//   1. First call (IDLE): starts a background scan, returns {scanning:true}
//   2. Subsequent calls (SCANNING): scan still running, returns {scanning:true}
//   3. Final call (DONE): returns JSON array of networks, resets to IDLE
void WebPortal::handleScan(AsyncWebServerRequest* request) {
    ScanState state = _wifi->getScanState();

    if (state == ScanState::DONE) {
        // Results ready — serialize to JSON and send
        auto& networks = _wifi->getScanResults();
        Serial.printf("[Scan] Returning %d networks\n", networks.size());

        JsonDocument doc;
        JsonArray arr = doc.to<JsonArray>();
        for (auto& net : networks) {
            JsonObject obj = arr.add<JsonObject>();
            obj["ssid"] = net.ssid;
            obj["rssi"] = net.rssi;
            obj["secure"] = net.secure;
            Serial.printf("[Scan]   '%s' RSSI:%d %s\n", net.ssid.c_str(), net.rssi,
                          net.secure ? "secured" : "open");
        }
        String json;
        serializeJson(doc, json);
        request->send(200, "application/json", json);
        return;
    }

    if (state == ScanState::SCANNING) {
        // Scan in progress — tell client to poll again
        Serial.println("[Scan] Scan in progress, telling client to wait...");
        request->send(200, "application/json", "{\"scanning\":true}");
        return;
    }

    // IDLE — kick off a new async scan
    Serial.println("[Scan] Starting new async scan...");
    _wifi->startScan();
    request->send(200, "application/json", "{\"scanning\":true}");
}

// ==========================================================================
// API Handler: /api/status
// ==========================================================================
// Returns current WiFi mode, IP, SSID, RSSI, and saved IP configuration.
// The web UI calls this once on page load to determine which view to show
// (AP provisioning form vs. STA device dashboard).
void WebPortal::handleStatus(AsyncWebServerRequest* request) {
    JsonDocument doc;
    WiFiState state = _wifi->getState();

    const char* mode = (state == WiFiState::AP_MODE) ? "AP" :
                       (state == WiFiState::STA_CONNECTED) ? "STA" : "DISCONNECTED";
    doc["mode"] = mode;
    doc["ip"] = _wifi->getIP();
    doc["ssid"] = (state == WiFiState::AP_MODE) ? _wifi->getAPSSID() : _wifi->getConnectedSSID();
    doc["rssi"] = _wifi->getRSSI();

    // Include saved IP config so the form can show current settings
    WiFiConfig config = _store->load();
    doc["dhcp"] = config.useDHCP;
    doc["staticIP"] = config.staticIP;
    doc["staticGateway"] = config.staticGateway;
    doc["staticSubnet"] = config.staticSubnet;
    doc["staticDNS"] = config.staticDNS;

    String json;
    serializeJson(doc, json);
    Serial.printf("[Status] %s -> %s\n", mode, json.c_str());
    request->send(200, "application/json", json);
}

// ==========================================================================
// Build Device Info JSON
// ==========================================================================
// Assembles a comprehensive device info payload. Shared between the
// GET /api/device endpoint (one-time fetch) and SSE "device" events
// (pushed every second). Includes chip info, CPU per-core, memory,
// storage, network, IMU status, and uptime.
String WebPortal::buildDeviceJson() {
    JsonDocument doc;

    // Chip info
    doc["chip"] = ESP.getChipModel();
    doc["chipRev"] = ESP.getChipRevision();
    doc["cpuFreq"] = ESP.getCpuFreqMHz();
    doc["cores"] = ESP.getChipCores();
    doc["sdk"] = ESP.getSdkVersion();

    // CPU usage as pre-formatted strings (e.g., "30.4") to avoid
    // floating-point serialization quirks in ArduinoJson.
    // The `serialized()` wrapper tells ArduinoJson to embed the
    // string value directly without quoting it (raw JSON number).
    doc["cpuUsage0"] = serialized(String(getCpuUsage0(), 1));
    doc["cpuUsage1"] = serialized(String(getCpuUsage1(), 1));

    // Memory — heap is the ESP32's dynamic RAM allocation pool.
    // freeHeap: currently available bytes.
    // minFreeHeap: lowest free heap ever seen (watermark for detecting leaks).
    doc["freeHeap"] = ESP.getFreeHeap();
    doc["totalHeap"] = ESP.getHeapSize();
    doc["minFreeHeap"] = ESP.getMinFreeHeap();

    // PSRAM — external SPI RAM (this board variant doesn't have it,
    // but we report it anyway for compatibility with boards that do).
    doc["psramSize"] = ESP.getPsramSize();
    doc["freePsram"] = ESP.getFreePsram();

    // Flash / storage
    doc["flashSize"] = ESP.getFlashChipSize();
    doc["flashSpeed"] = ESP.getFlashChipSpeed();

    // LittleFS usage — how much of the flash filesystem is used by web UI files
    doc["fsTotal"] = LittleFS.totalBytes();
    doc["fsUsed"] = LittleFS.usedBytes();

    // Network
    doc["mac"] = WiFi.macAddress();
    doc["hostname"] = WiFi.getHostname();
    doc["ip"] = _wifi->getIP();
    doc["gateway"] = WiFi.gatewayIP().toString();
    doc["dns"] = WiFi.dnsIP().toString();
    doc["subnet"] = WiFi.subnetMask().toString();
    doc["rssi"] = _wifi->getRSSI();
    doc["channel"] = WiFi.channel();
    doc["ssid"] = _wifi->getConnectedSSID();

    // IMU
    doc["imuReady"] = isImuReady();

    // Uptime in milliseconds (rolls over after ~49 days)
    doc["uptimeMs"] = millis();

    String json;
    serializeJson(doc, json);
    return json;
}

// GET /api/device — returns full device info JSON
void WebPortal::handleDevice(AsyncWebServerRequest* request) {
    String json = buildDeviceJson();
    Serial.printf("[Device] %s\n", json.c_str());
    request->send(200, "application/json", json);
}

// ==========================================================================
// API Handler: POST /api/connect
// ==========================================================================
// Receives WiFi credentials as JSON, saves them to NVS, then reboots.
//
// Why reboot instead of switching modes live?
// Switching from AP to STA mode while the async web server has open TCP
// sockets triggers a lwIP assertion crash (`tcp_update_rcv_ann_wnd`).
// Rebooting is clean — setup() will find the saved credentials in NVS
// and connect in under a second.
void WebPortal::handleConnect(AsyncWebServerRequest* request) {
    Serial.println("[Connect] Received connection request");

    // Log raw body for debugging (helpful when JSON parsing fails)
    Serial.printf("[Connect] Body: %s\n", _connectBody.c_str());

    // Parse the accumulated JSON body
    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, _connectBody);

    if (err) {
        Serial.printf("[Connect] *** JSON parse error: %s ***\n", err.c_str());
        request->send(400, "application/json", "{\"error\":\"Invalid JSON\"}");
        return;
    }

    // Extract fields with sensible defaults.
    // The `| ""` syntax is ArduinoJson's way of providing a default
    // value if the key is missing from the JSON.
    WiFiConfig config;
    config.ssid = doc["ssid"] | "";
    config.password = doc["password"] | "";
    config.useDHCP = doc["dhcp"] | true;
    config.staticIP = doc["ip"] | "";
    config.staticGateway = doc["gateway"] | "";
    config.staticSubnet = doc["subnet"] | "255.255.255.0";
    config.staticDNS = doc["dns"] | "";

    Serial.printf("[Connect] SSID: '%s'\n", config.ssid.c_str());
    Serial.printf("[Connect] Password: '%s' (%d chars)\n",
                  config.password.length() > 0 ? "****" : "(empty)", config.password.length());
    Serial.printf("[Connect] DHCP: %s\n", config.useDHCP ? "yes" : "no");
    if (!config.useDHCP) {
        Serial.printf("[Connect] Static IP: %s  GW: %s  SN: %s  DNS: %s\n",
                      config.staticIP.c_str(), config.staticGateway.c_str(),
                      config.staticSubnet.c_str(), config.staticDNS.c_str());
    }

    // Validate: at minimum we need an SSID
    if (config.ssid.length() == 0) {
        Serial.println("[Connect] *** SSID is empty! Rejecting. ***");
        request->send(400, "application/json", "{\"error\":\"SSID required\"}");
        return;
    }

    // Save to NVS so credentials survive the reboot
    _store->save(config);
    Serial.printf("[Connect] Config saved for '%s'\n", config.ssid.c_str());

    // Send response first, then reboot. The 500ms delay gives the
    // HTTP response time to be sent before the TCP stack shuts down.
    request->send(200, "application/json", "{\"status\":\"connecting\"}");
    Serial.println("[Connect] Response sent, rebooting to connect...");
    delay(500);
    ESP.restart();
}

// ==========================================================================
// API Handler: POST /api/disconnect
// ==========================================================================
// Clears all saved WiFi credentials from NVS and reboots.
// On restart, setup() will find no credentials and start AP mode.
void WebPortal::handleDisconnect(AsyncWebServerRequest* request) {
    Serial.println("[Disconnect] Clearing credentials and rebooting...");
    _store->clear();
    request->send(200, "application/json", "{\"status\":\"disconnecting\"}");
    Serial.println("[Disconnect] Response sent, rebooting in 500ms...");
    delay(500);
    ESP.restart();
}

// ==========================================================================
// SSE Push: IMU Data (20Hz)
// ==========================================================================
// Sends the current quaternion (w, x, y, z) and accuracy to all
// connected SSE clients. The "imu" event name lets the client-side
// JavaScript distinguish IMU events from device events on the same stream.
//
// Early-out if no clients are connected — avoids wasting CPU on
// JSON serialization when nobody's listening.
void WebPortal::sendIMU() {
    if (_events.count() == 0) return;  // No SSE clients connected
    if (!isImuReady()) return;         // IMU not initialized

    float q[4];
    getImuQuat(q);
    float acc = getImuAccuracy();

    // Manual JSON construction with snprintf — faster than ArduinoJson
    // for this small fixed-format payload. Called 20 times per second,
    // so every microsecond counts.
    char buf[96];
    snprintf(buf, sizeof(buf),
        "{\"w\":%.4f,\"x\":%.4f,\"y\":%.4f,\"z\":%.4f,\"a\":%.3f}",
        q[0], q[1], q[2], q[3], acc);

    // Send to all connected SSE clients with event name "imu"
    // and millis() as the event ID (helps clients detect missed events)
    _events.send(buf, "imu", millis());
}

// ==========================================================================
// SSE Push: Device Metrics (1Hz)
// ==========================================================================
// Sends comprehensive device info to all connected SSE clients.
// Reuses buildDeviceJson() so the SSE data matches the REST API response.
void WebPortal::sendDevice() {
    if (_events.count() == 0) return;  // No SSE clients connected

    String json = buildDeviceJson();
    _events.send(json.c_str(), "device", millis());
}
