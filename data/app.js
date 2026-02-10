// ==========================================================================
// app.js — 3DScanner Web Dashboard Client
// ==========================================================================
//
// This script powers the entire browser-side experience:
//
// 1. SSE (Server-Sent Events) — single persistent HTTP connection
//    receives IMU quaternions at 20Hz and device metrics at 1Hz.
//    No polling, no WebSockets — SSE is ideal for one-way server→client
//    real-time data.
//
// 2. IMU visualization — 3D GY-BNO085 PCB board model rendered on a
//    Canvas 2D context with orthographic projection, backface culling,
//    Lambert shading, and painter's algorithm depth sorting. The IC chip
//    renders in black, the PCB body in green. XYZ axis arrows overlay
//    the model, matching the sensor's physical axis markings.
//    Rotates in real-time with the BNO085 quaternion data.
//
// 3. Rolling charts — canvas-based time-series graphs for CPU usage
//    per core, heap memory, and filesystem usage.
//
// 4. WiFi provisioning UI — network scanning, password entry, DHCP/static
//    IP configuration (only shown in AP mode).
//
// No external libraries — everything is vanilla JS + Canvas 2D API.
// ==========================================================================

// --- Global State ---

let selectedSSID = '';        // SSID the user tapped in the scan results
let selectedSecure = false;   // Whether the selected network needs a password
let useDHCP = true;           // IP config mode toggle (DHCP vs. static)
let refreshMs = 5000;         // Chart update interval in ms (user-selectable)
let eventSource = null;       // SSE EventSource connection (null when disconnected)
let lastChartUpdate = 0;      // Timestamp of the last chart data point
let latestDevice = null;      // Most recent device metrics from SSE

// IMU smoothing using Exponential Moving Average (EMA).
// Alpha controls how much weight new samples get:
//   0 = completely frozen (ignores new data)
//   1 = no smoothing (raw data, jittery)
//   0.3 = a good balance between responsiveness and smooth animation
const IMU_ALPHA = 0.3;
let smoothIMU = null; // {w, x, y, z, a} — null until first IMU event

// Chart data: circular buffers (arrays that shift when full).
// MAX_POINTS controls how many data points are visible on screen.
const chartData = { cpu0: [], cpu1: [], heap: [] };
const MAX_POINTS = 60;

// 3D IMU visualization state
let imuVizCanvas = null;
let imuVizCtx = null;

// Canvas dimension cache — avoids expensive getBoundingClientRect() + canvas
// resize on every frame. The IMU viz renders at 20Hz; resizing the canvas
// clears it and forces a GPU buffer reallocation, which is wasteful when the
// size hasn't changed (99% of frames).
let cachedVizW = 0;
let cachedVizH = 0;

// ToF heatmap state
let tofCanvas = null;
let tofCtx = null;
let cachedTofW = 0;
let cachedTofH = 0;

// --- Initialization ---

document.addEventListener('DOMContentLoaded', () => {
    // Fetch WiFi status to determine which view to show (AP or STA)
    fetchStatus();

    // Grab the 3D visualization canvas for later rendering
    imuVizCanvas = document.getElementById('imu-viz-canvas');
    if (imuVizCanvas) imuVizCtx = imuVizCanvas.getContext('2d');

    // Grab the ToF heatmap canvas
    tofCanvas = document.getElementById('tof-heatmap-0');
    if (tofCanvas) tofCtx = tofCanvas.getContext('2d');

    // Wire up the refresh rate dropdown
    var sel = document.getElementById('refresh-rate');
    if (sel) {
        sel.addEventListener('change', function() {
            refreshMs = parseInt(this.value);
        });
    }
});

// ==========================================================================
// Server-Sent Events (SSE)
// ==========================================================================
//
// SSE is an HTML5 API where the browser opens a persistent HTTP connection
// and the server pushes text events down it. Unlike WebSockets, SSE is:
//   - Unidirectional (server → client only, which is all we need)
//   - Auto-reconnects on disconnect (built into the browser)
//   - Works over regular HTTP (no upgrade handshake)
//
// Our ESP32 sends two event types on /api/events:
//   "imu"    — quaternion + accuracy at 20Hz
//   "device" — system metrics at 1Hz

function startEventStream() {
    if (eventSource) return; // Already connected
    eventSource = new EventSource('/api/events');

    // Listen for IMU quaternion events (20Hz from ESP32)
    eventSource.addEventListener('imu', function(e) {
        try {
            var d = JSON.parse(e.data);

            // Apply EMA smoothing to reduce jitter in the 3D visualization.
            // Each component is smoothed independently:
            //   smoothed = smoothed + alpha * (raw - smoothed)
            // This is equivalent to: smoothed = alpha*raw + (1-alpha)*smoothed
            if (!smoothIMU) {
                smoothIMU = { w: d.w, x: d.x, y: d.y, z: d.z, a: d.a };
            } else {
                // q and -q are the same rotation. If the new sample is in the
                // opposite hemisphere, negate it to avoid spinning the long way.
                var dot = d.w*smoothIMU.w + d.x*smoothIMU.x + d.y*smoothIMU.y + d.z*smoothIMU.z;
                var nw = d.w, nx = d.x, ny = d.y, nz = d.z;
                if (dot < 0) { nw = -nw; nx = -nx; ny = -ny; nz = -nz; }
                smoothIMU.w += IMU_ALPHA * (nw - smoothIMU.w);
                smoothIMU.x += IMU_ALPHA * (nx - smoothIMU.x);
                smoothIMU.y += IMU_ALPHA * (ny - smoothIMU.y);
                smoothIMU.z += IMU_ALPHA * (nz - smoothIMU.z);
                smoothIMU.a += IMU_ALPHA * (d.a - smoothIMU.a);
                // Re-normalize — EMA doesn't preserve unit length
                var len = Math.sqrt(smoothIMU.w*smoothIMU.w + smoothIMU.x*smoothIMU.x + smoothIMU.y*smoothIMU.y + smoothIMU.z*smoothIMU.z);
                if (len > 0) { smoothIMU.w /= len; smoothIMU.x /= len; smoothIMU.y /= len; smoothIMU.z /= len; }
            }
            updateIMUDisplay(smoothIMU.w, smoothIMU.x, smoothIMU.y, smoothIMU.z, smoothIMU.a);
        } catch (err) {
            console.error('IMU parse error:', err);
        }
    });

    // Listen for ToF distance data events (10Hz from ESP32)
    eventSource.addEventListener('tof', function(e) {
        try {
            var d = JSON.parse(e.data);
            if (d.sensors && d.sensors.length > 0) {
                renderToFHeatmap(d.sensors[0]);
            }
        } catch (err) {
            console.error('ToF parse error:', err);
        }
    });

    // Listen for device metrics events (1Hz from ESP32)
    eventSource.addEventListener('device', function(e) {
        try {
            var d = JSON.parse(e.data);
            latestDevice = d;
            updateDeviceDisplay(d);

            // Only add a chart data point at the user-selected refresh rate.
            // The ESP32 sends device events every second, but the user might
            // want charts to update every 5 seconds for a less busy display.
            var now = Date.now();
            if (now - lastChartUpdate >= refreshMs) {
                lastChartUpdate = now;
                updateCharts(d);
            }
        } catch (err) {
            console.error('Device parse error:', err);
        }
    });

    // SSE auto-reconnects on error — no custom handling needed.
    // The browser will retry with exponential backoff.
    eventSource.onerror = function() {
        // EventSource auto-reconnects
    };
}

// Close the SSE connection (e.g., when switching to AP view)
function stopEventStream() {
    if (eventSource) {
        eventSource.close();
        eventSource = null;
    }
    smoothIMU = null; // Reset smoothing state for next connection
}

// ==========================================================================
// Status Check (determines AP vs STA view)
// ==========================================================================
// Called once on page load. Fetches /api/status to determine if the ESP32
// is in AP mode (show WiFi provisioning form) or STA mode (show device
// dashboard with live charts and IMU visualization).

async function fetchStatus() {
    try {
        const res = await fetch('/api/status');
        const data = await res.json();
        updateStatusBar(data);

        if (data.mode === 'STA') {
            // Connected to a network — show the device dashboard
            document.getElementById('device-info').classList.remove('hidden');
            document.getElementById('networks-section').classList.add('hidden');
            document.getElementById('connect-section').classList.add('hidden');
            document.getElementById('disconnect-section').classList.remove('hidden');
            startEventStream(); // Begin receiving real-time data
        } else {
            // AP mode — show WiFi provisioning form
            document.getElementById('device-info').classList.add('hidden');
            document.getElementById('networks-section').classList.remove('hidden');
            document.getElementById('disconnect-section').classList.add('hidden');
            stopEventStream();
        }
    } catch (e) {
        console.error('Status fetch failed:', e);
    }
}

// ==========================================================================
// Display Updaters — populate DOM elements with live data
// ==========================================================================

// Updates IMU info card and triggers 3D visualization redraw.
// Converts quaternion to Euler angles (yaw/pitch/roll) for human readability,
// but the 3D visualization uses the quaternion directly (Euler angles have
// gimbal lock issues that quaternions avoid).
function updateIMUDisplay(w, x, y, z, acc) {
    document.getElementById('info-imu-status').textContent = 'Connected';
    document.getElementById('info-imu-status').className = 'info-value good';
    document.getElementById('info-imu-quat').textContent =
        'W:' + w.toFixed(3) + ' X:' + x.toFixed(3) + ' Y:' + y.toFixed(3) + ' Z:' + z.toFixed(3);

    // Remap from BNO085's Z-up (ENU) frame to Y-up visualization frame.
    // Sensor axes: X=short edge, Y=long edge, Z=up (perpendicular to board)
    // Model axes:  X=long edge,  Y=up,        Z=-short edge
    // Quaternion remap: [w, x, y, z] → [w, x, z, -y]
    var rw = w, rx = x, ry = z, rz = -y;

    // Euler angles (Tait-Bryan ZYX decomposition in Y-up frame).
    // Gimbal lock at ±90° yaw is handled by clamping.
    var sp = 2 * (rw * rx + ry * rz);
    var cp = 1 - 2 * (rx * rx + ry * ry);
    var pitch = Math.atan2(sp, cp) * 180 / Math.PI;
    var sy = 2 * (rw * ry - rz * rx);
    var yaw = (Math.abs(sy) >= 1 ? Math.sign(sy) * 90 : Math.asin(sy) * 180 / Math.PI);
    var sr = 2 * (rw * rz + rx * ry);
    var cr = 1 - 2 * (ry * ry + rz * rz);
    var roll = Math.atan2(sr, cr) * 180 / Math.PI;

    document.getElementById('info-imu-yaw').textContent = yaw.toFixed(1) + '\u00B0';
    document.getElementById('info-imu-pitch').textContent = pitch.toFixed(1) + '\u00B0';
    document.getElementById('info-imu-roll').textContent = roll.toFixed(1) + '\u00B0';

    // Accuracy is in radians from the BNO085 — convert to degrees for display
    var accDeg = (acc * 180 / Math.PI).toFixed(1);
    document.getElementById('info-imu-accuracy').textContent = '\u00B1' + accDeg + '\u00B0';

    // Redraw the 3D board model with the new orientation
    renderIMUViz(w, x, y, z);
}

// Updates the device info cards (network, system, IMU status)
function updateDeviceDisplay(d) {
    // Network info
    document.getElementById('info-ssid').textContent = d.ssid;
    document.getElementById('info-ip').textContent = d.ip;
    document.getElementById('info-gateway').textContent = d.gateway;
    document.getElementById('info-dns').textContent = d.dns;
    document.getElementById('info-mac').textContent = d.mac;
    document.getElementById('info-channel').textContent = d.channel;

    // RSSI → human-readable signal strength with dBm value
    var rssi = d.rssi;
    var strength = rssi >= -50 ? 'Excellent' : rssi >= -60 ? 'Good' : rssi >= -70 ? 'Fair' : 'Weak';
    document.getElementById('info-rssi').textContent = strength + ' (' + rssi + ' dBm)';

    // System info
    document.getElementById('info-chip').textContent = d.chip + ' rev ' + d.chipRev;
    document.getElementById('info-cpu').textContent = d.cores + ' cores @ ' + d.cpuFreq + ' MHz';
    document.getElementById('info-heap').textContent = formatBytes(d.freeHeap) + ' free / ' + formatBytes(d.totalHeap);
    document.getElementById('info-flash').textContent = formatBytes(d.flashSize);
    document.getElementById('info-fs').textContent = formatBytes(d.fsUsed) + ' / ' + formatBytes(d.fsTotal);
    document.getElementById('info-uptime').textContent = formatUptime(d.uptimeMs);
    document.getElementById('info-sdk').textContent = d.sdk;

    // PSRAM: show "None" if the board doesn't have external RAM
    if (d.psramSize > 0) {
        document.getElementById('info-psram').textContent = formatBytes(d.freePsram) + ' free / ' + formatBytes(d.psramSize);
    } else {
        document.getElementById('info-psram').textContent = 'None';
    }

    // IMU connection status indicator
    if (d.imuReady) {
        document.getElementById('info-imu-status').textContent = 'Connected';
        document.getElementById('info-imu-status').className = 'info-value good';
    } else {
        document.getElementById('info-imu-status').textContent = 'Not detected';
        document.getElementById('info-imu-status').className = 'info-value';
    }
}

// Adds new data points to the rolling charts and redraws.
// Each chart is a fixed-length array that shifts (drops oldest point)
// when it exceeds MAX_POINTS, creating a scrolling time-series effect.
// The filesystem chart is a donut (pie) since storage is essentially
// static at runtime — no need for a time series.
function updateCharts(d) {
    var cpu0 = parseFloat(d.cpuUsage0) || 0;
    var cpu1 = parseFloat(d.cpuUsage1) || 0;
    chartData.cpu0.push(cpu0);
    chartData.cpu1.push(cpu1);
    if (chartData.cpu0.length > MAX_POINTS) chartData.cpu0.shift();
    if (chartData.cpu1.length > MAX_POINTS) chartData.cpu1.shift();

    chartData.heap.push(d.freeHeap);
    if (chartData.heap.length > MAX_POINTS) chartData.heap.shift();

    // Update the numeric value labels above each chart
    document.getElementById('chart-cpu0-val').textContent = cpu0.toFixed(1) + '%';
    document.getElementById('chart-cpu1-val').textContent = cpu1.toFixed(1) + '%';
    document.getElementById('chart-heap-val').textContent = formatBytes(d.freeHeap);
    document.getElementById('chart-fs-val').textContent =
        formatBytes(d.fsUsed) + ' / ' + formatBytes(d.fsTotal);

    // Redraw rolling time-series charts
    drawChart('chart-cpu0', chartData.cpu0, '#4ecca3', '%', 0, 100);
    drawChart('chart-cpu1', chartData.cpu1, '#4ecca3', '%', 0, 100);
    drawChart('chart-heap', chartData.heap, '#4ecca3', 'B', 0, d.totalHeap);

    // Filesystem as a donut chart (used vs total)
    drawDonut('chart-fs', d.fsUsed, d.fsTotal);
}

// ==========================================================================
// Utility Functions
// ==========================================================================

// Formats a byte count into a human-readable string with appropriate units
function formatBytes(bytes) {
    if (bytes >= 1048576) return (bytes / 1048576).toFixed(1) + ' MB';
    if (bytes >= 1024) return (bytes / 1024).toFixed(0) + ' KB';
    return bytes + ' B';
}

// Converts milliseconds uptime into "Xd Yh Zm" or "Yh Zm Ws" format
function formatUptime(ms) {
    var s = Math.floor(ms / 1000);
    var d = Math.floor(s / 86400);
    var h = Math.floor((s % 86400) / 3600);
    var m = Math.floor((s % 3600) / 60);
    var sec = s % 60;
    if (d > 0) return d + 'd ' + h + 'h ' + m + 'm';
    if (h > 0) return h + 'h ' + m + 'm ' + sec + 's';
    return m + 'm ' + sec + 's';
}

// ==========================================================================
// Canvas Charts — Retina-Aware Rolling Time Series
// ==========================================================================
//
// Each chart is drawn on an HTML <canvas> element. We handle:
//   - Retina/HiDPI displays: canvas pixel buffer is scaled by devicePixelRatio
//     so lines appear crisp on iPhones and MacBooks
//   - Fixed Y-axis range: CPU is always 0-100%, memory uses total as max
//   - Grid lines + Y-axis labels for readability
//   - Semi-transparent fill under the line for visual weight
//   - Right-aligned data: new points appear on the right edge

function drawChart(canvasId, data, color, unit, fixedMin, fixedMax) {
    var canvas = document.getElementById(canvasId);
    if (!canvas) return;

    // Handle Retina/HiDPI displays: the canvas element has a CSS size
    // (e.g., 300x100) but we set its pixel buffer to 2x that on retina
    // screens so lines aren't blurry.
    var dpr = window.devicePixelRatio || 1;
    var rect = canvas.getBoundingClientRect();
    canvas.width = rect.width * dpr;
    canvas.height = rect.height * dpr;

    var ctx = canvas.getContext('2d');
    ctx.scale(dpr, dpr); // Scale drawing ops so we work in CSS pixels

    var W = rect.width;
    var H = rect.height;
    var pad = { top: 4, bottom: 14, left: 0, right: 0 };
    var plotW = W - pad.left - pad.right;
    var plotH = H - pad.top - pad.bottom;

    ctx.clearRect(0, 0, W, H);

    if (data.length < 2) return; // Need at least 2 points to draw a line

    var min = fixedMin;
    var max = fixedMax;

    // Draw horizontal grid lines (5 lines including top and bottom)
    ctx.strokeStyle = '#2a2a4a';
    ctx.lineWidth = 1;
    for (var i = 0; i <= 4; i++) {
        var gy = pad.top + (plotH / 4) * i;
        ctx.beginPath();
        ctx.moveTo(pad.left, gy);
        ctx.lineTo(pad.left + plotW, gy);
        ctx.stroke();
    }

    // Y-axis labels on the right edge
    ctx.fillStyle = '#666';
    ctx.font = '9px sans-serif';
    ctx.textAlign = 'right';
    for (var i = 0; i <= 4; i++) {
        var gy = pad.top + (plotH / 4) * i;
        var val = max - (max - min) * (i / 4);
        var label;
        if (unit === '%') {
            label = val.toFixed(0) + '%';
        } else {
            label = formatBytes(val);
        }
        ctx.fillText(label, W - 2, gy + 3);
    }

    // Draw the data line (right-aligned: newest data on the right edge)
    ctx.strokeStyle = color;
    ctx.lineWidth = 2;
    ctx.lineJoin = 'round'; // Smooth corners between line segments

    ctx.beginPath();

    // Calculate horizontal spacing so MAX_POINTS fills the full width.
    // If we have fewer points, they cluster toward the right edge.
    var step = plotW / (MAX_POINTS - 1);
    var startX = pad.left + plotW - (data.length - 1) * step;

    for (var i = 0; i < data.length; i++) {
        var x = startX + i * step;
        // Map data value to Y position (0=bottom, 1=top)
        var ratio = max > min ? (data[i] - min) / (max - min) : 0;
        if (ratio > 1) ratio = 1;
        if (ratio < 0) ratio = 0;
        var y = pad.top + plotH - ratio * plotH;
        if (i === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
    }
    ctx.stroke();

    // Semi-transparent fill under the line for visual weight.
    // Continues the path down to the bottom edge and back.
    ctx.lineTo(startX + (data.length - 1) * step, pad.top + plotH);
    ctx.lineTo(startX, pad.top + plotH);
    ctx.closePath();
    ctx.globalAlpha = 0.1;
    ctx.fillStyle = color;
    ctx.fill();
    ctx.globalAlpha = 1.0;
}

// ==========================================================================
// Donut Chart — used for filesystem (static data, not a time series)
// ==========================================================================
// Draws a ring chart with a "used" segment and a "free" segment,
// with the usage percentage displayed in the center.

function drawDonut(canvasId, used, total) {
    var canvas = document.getElementById(canvasId);
    if (!canvas || total <= 0) return;

    var dpr = window.devicePixelRatio || 1;
    var rect = canvas.getBoundingClientRect();
    canvas.width = rect.width * dpr;
    canvas.height = rect.height * dpr;

    var ctx = canvas.getContext('2d');
    ctx.scale(dpr, dpr);

    var W = rect.width, H = rect.height;
    ctx.clearRect(0, 0, W, H);

    var cx = W / 2, cy = H / 2;
    var outerR = Math.min(W, H) / 2 - 4;
    var innerR = outerR * 0.62;

    var frac = Math.min(used / total, 1);
    var usedAngle = frac * 2 * Math.PI;
    var start = -Math.PI / 2; // 12 o'clock

    // Free segment (full ring as background)
    ctx.beginPath();
    ctx.arc(cx, cy, outerR, 0, 2 * Math.PI);
    ctx.arc(cx, cy, innerR, 2 * Math.PI, 0, true);
    ctx.fillStyle = '#2a2a4a';
    ctx.fill();

    // Used segment (drawn on top, clockwise from 12 o'clock)
    if (frac > 0) {
        ctx.beginPath();
        ctx.arc(cx, cy, outerR, start, start + usedAngle);
        ctx.arc(cx, cy, innerR, start + usedAngle, start, true);
        ctx.fillStyle = '#4ecca3';
        ctx.fill();
    }

    // Center label: usage percentage
    var pct = (frac * 100).toFixed(0) + '%';
    ctx.fillStyle = '#e0e0e0';
    ctx.font = 'bold ' + Math.round(innerR * 0.48) + 'px sans-serif';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(pct, cx, cy);
}

// ==========================================================================
// Status Bar — shows current WiFi mode and connection info
// ==========================================================================

function updateStatusBar(data) {
    var modeEl = document.getElementById('status-mode');
    var infoEl = document.getElementById('status-info');

    // Set badge color based on mode (AP=red, STA=green, offline=gray)
    modeEl.className = 'badge ' + data.mode.toLowerCase();

    if (data.mode === 'AP') {
        modeEl.textContent = 'AP Mode';
        infoEl.textContent = data.ssid + ' \u00B7 ' + data.ip; // middle dot separator
    } else if (data.mode === 'STA') {
        modeEl.textContent = 'Connected';
        infoEl.textContent = data.ssid + ' \u00B7 ' + data.ip;
    } else {
        modeEl.textContent = 'Offline';
        infoEl.textContent = '';
    }
}

// ==========================================================================
// WiFi Provisioning (AP Mode Only)
// ==========================================================================
// These functions handle the network scan → select → connect workflow
// that runs when the device is in AP mode.

// Scan for available WiFi networks.
// Uses polling because the ESP32's scan is asynchronous: first call
// starts the scan and returns {scanning:true}, subsequent calls check
// for results every 500ms until the scan completes or times out.
async function scanNetworks() {
    var btn = document.getElementById('btn-scan');
    var list = document.getElementById('network-list');

    btn.disabled = true;
    btn.textContent = 'Scanning...';
    list.innerHTML = '<p class="muted">Scanning...</p>';

    try {
        var res = await fetch('/api/scan');
        var data = await res.json();

        // Poll every 500ms while scan is in progress (max 20 attempts = 10s)
        var attempts = 0;
        while (data.scanning && attempts < 20) {
            await new Promise(r => setTimeout(r, 500));
            res = await fetch('/api/scan');
            data = await res.json();
            attempts++;
        }

        if (data.scanning) {
            list.innerHTML = '<p class="muted">Scan timed out, try again</p>';
        } else if (Array.isArray(data) && data.length === 0) {
            list.innerHTML = '<p class="muted">No networks found</p>';
        } else if (Array.isArray(data)) {
            // Build the network list UI
            list.innerHTML = '';
            data.forEach(function(net) {
                var item = document.createElement('div');
                item.className = 'network-item';
                item.onclick = function() { selectNetwork(net.ssid, net.secure, item); };

                var bars = signalBars(net.rssi);
                var lock = net.secure ? '<span class="network-lock">&#128274;</span>' : '';

                item.innerHTML = bars +
                    '<span class="network-ssid">' + escapeHtml(net.ssid) + '</span>' + lock;
                list.appendChild(item);
            });
        }
    } catch (e) {
        list.innerHTML = '<p class="muted">Scan failed</p>';
        console.error('Scan error:', e);
    }

    btn.disabled = false;
    btn.textContent = 'Scan';
}

// Generates a 4-bar WiFi signal strength indicator using CSS-styled divs.
// RSSI thresholds: -50 = 4 bars, -60 = 3, -70 = 2, worse = 1.
function signalBars(rssi) {
    var strength = rssi >= -50 ? 4 : rssi >= -60 ? 3 : rssi >= -70 ? 2 : 1;
    var html = '<div class="signal-bars">';
    for (var i = 1; i <= 4; i++) {
        html += '<div class="bar' + (i <= strength ? ' active' : '') + '"></div>';
    }
    html += '</div>';
    return html;
}

// Called when the user taps a network in the scan results.
// Highlights the selected network and shows the connection form.
function selectNetwork(ssid, secure, element) {
    selectedSSID = ssid;
    selectedSecure = secure;

    // Remove highlight from all items, add to the tapped one
    document.querySelectorAll('.network-item').forEach(function(el) { el.classList.remove('selected'); });
    element.classList.add('selected');

    // Show the connection form with the selected SSID
    document.getElementById('selected-ssid').textContent = ssid;
    document.getElementById('connect-section').classList.remove('hidden');

    // Show password field only for secured networks
    var pwField = document.getElementById('password-field');
    if (secure) {
        pwField.classList.remove('hidden');
        document.getElementById('password').value = '';
        document.getElementById('password').focus();
    } else {
        pwField.classList.add('hidden');
    }
}

// Toggle password visibility (Show/Hide button next to password input)
function togglePassword() {
    var pw = document.getElementById('password');
    var btn = pw.nextElementSibling;
    if (pw.type === 'password') {
        pw.type = 'text';
        btn.textContent = 'Hide';
    } else {
        pw.type = 'password';
        btn.textContent = 'Show';
    }
}

// Toggle between DHCP (automatic IP) and Static IP configuration
function setDHCP(isDHCP) {
    useDHCP = isDHCP;
    document.getElementById('btn-dhcp').classList.toggle('active', isDHCP);
    document.getElementById('btn-static').classList.toggle('active', !isDHCP);
    document.getElementById('static-fields').classList.toggle('hidden', isDHCP);
}

// Submit the WiFi connection form.
// Sends credentials to /api/connect, which saves them to NVS and reboots.
// The connection will drop (we were connected to the AP, which is shutting
// down), so we show a helpful message and retry status after 8 seconds.
async function connectToNetwork(event) {
    event.preventDefault();

    var payload = {
        ssid: selectedSSID,
        password: document.getElementById('password').value,
        dhcp: useDHCP
    };

    // Include static IP fields only when DHCP is disabled
    if (!useDHCP) {
        payload.ip = document.getElementById('static-ip').value;
        payload.subnet = document.getElementById('static-subnet').value;
        payload.gateway = document.getElementById('static-gateway').value;
        payload.dns = document.getElementById('static-dns').value;
    }

    document.getElementById('connect-section').classList.add('hidden');
    document.getElementById('spinner').classList.remove('hidden');

    try {
        await fetch('/api/connect', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(payload)
        });
    } catch (e) {
        // Expected — the ESP32 reboots after saving credentials,
        // so this request will fail. That's normal.
    }

    // Show a user-friendly message while the device reboots
    document.getElementById('spinner').innerHTML =
        '<div class="spin"></div>' +
        '<p>Connecting to <strong>' + escapeHtml(selectedSSID) + '</strong>...</p>' +
        '<p class="muted" style="margin-top:12px">If you lose connection, reconnect to your WiFi and browse to the device\'s new IP (check your router or serial monitor).</p>';

    // After 8 seconds, try to fetch status — if the ESP32 connected
    // to the same network as the browser, this will succeed.
    setTimeout(async function() {
        try {
            var res = await fetch('/api/status');
            var data = await res.json();
            if (data.mode === 'STA') {
                document.getElementById('spinner').classList.add('hidden');
                fetchStatus();
                document.getElementById('disconnect-section').classList.remove('hidden');
            }
        } catch (e) {
            // Still disconnected — the device is on a different network now
        }
    }, 8000);
}

// "Forget Network & Reset" button handler.
// Confirms with the user, then sends POST /api/disconnect which
// clears NVS credentials and reboots into AP mode.
async function disconnectNetwork() {
    if (!confirm('This will forget the saved network and reboot into AP mode. Continue?')) return;
    stopEventStream();

    try {
        await fetch('/api/disconnect', { method: 'POST' });
    } catch (e) {
        // Device is rebooting — request will fail, that's expected
    }

    document.getElementById('status-mode').textContent = 'Rebooting...';
    document.getElementById('status-info').textContent = '';
}

// Safely escape HTML special characters to prevent XSS when
// inserting user-controlled text (like SSIDs) into innerHTML.
function escapeHtml(text) {
    var div = document.createElement('div');
    div.appendChild(document.createTextNode(text));
    return div.innerHTML;
}

// ==========================================================================
// ToF Heatmap — 8x8 Distance Grid Visualization
// ==========================================================================
//
// Renders an 8x8 grid of colored cells representing the VL53L5CX distance
// data. Each cell is colored using a 5-stop ramp:
//   Blue (close, 20mm) → Cyan → Green → Yellow → Red (far, 4000mm)
//   Gray for invalid zones (status != 5).
//
// Lens flip: the real VL53L5CX has inverted optics, so zone i maps to
// display position row=7-floor(i/8), col=7-(i%8). This matches what the
// sensor physically "sees" when looking at the scene.

function tofDistanceColor(dist, status) {
    if (status !== 5) return '#333'; // Invalid zone

    // Normalize to 0-1 range across the sensor's distance range
    var t = Math.max(0, Math.min(1, (dist - 20) / 3980));

    // 5-stop color ramp matching the CSS legend gradient
    var stops = [
        [74, 158, 255],   // blue   (t=0, close)
        [0, 229, 255],    // cyan   (t=0.25)
        [78, 204, 163],   // green  (t=0.5)
        [255, 230, 109],  // yellow (t=0.75)
        [233, 69, 96]     // red    (t=1.0, far)
    ];

    // Interpolate between the two surrounding stops
    var idx = t * 4;
    var i = Math.min(3, Math.floor(idx));
    var f = idx - i;
    var c0 = stops[i], c1 = stops[i + 1];
    return 'rgb(' +
        Math.round(c0[0] + (c1[0] - c0[0]) * f) + ',' +
        Math.round(c0[1] + (c1[1] - c0[1]) * f) + ',' +
        Math.round(c0[2] + (c1[2] - c0[2]) * f) + ')';
}

function renderToFHeatmap(sensor) {
    if (!tofCtx) return;

    var d = sensor.d;
    var s = sensor.s;
    if (!d || d.length !== 64) return;

    // Retina-aware canvas sizing (same pattern as IMU viz)
    var dpr = window.devicePixelRatio || 1;
    var rect = tofCanvas.getBoundingClientRect();
    var needsResize = (rect.width !== cachedTofW || rect.height !== cachedTofH);
    if (needsResize) {
        cachedTofW = rect.width;
        cachedTofH = rect.height;
        tofCanvas.width = cachedTofW * dpr;
        tofCanvas.height = cachedTofH * dpr;
    }
    var ctx = tofCtx;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);

    var W = cachedTofW, H = cachedTofH;
    ctx.clearRect(0, 0, W, H);

    // Calculate cell size to fit 8x8 grid with 1px gaps, keep cells square
    var gap = 1;
    var gridSize = 8;
    var cellW = (W - gap * (gridSize + 1)) / gridSize;
    var cellH = (H - gap * (gridSize + 1)) / gridSize;
    var cell = Math.min(cellW, cellH);

    // Center the grid in the canvas
    var totalW = cell * gridSize + gap * (gridSize + 1);
    var totalH = cell * gridSize + gap * (gridSize + 1);
    var offsetX = (W - totalW) / 2;
    var offsetY = (H - totalH) / 2;

    // Set up text style once (reused per cell)
    var showLabels = cell >= 28;
    if (showLabels) {
        ctx.font = Math.max(9, Math.min(12, cell * 0.35)) + 'px sans-serif';
        ctx.textAlign = 'center';
        ctx.textBaseline = 'middle';
    }

    for (var i = 0; i < 64; i++) {
        // Lens flip: invert rows and columns to match sensor optics
        var dispRow = 7 - Math.floor(i / 8);
        var dispCol = 7 - (i % 8);

        var x = offsetX + gap + dispCol * (cell + gap);
        var y = offsetY + gap + dispRow * (cell + gap);

        ctx.fillStyle = tofDistanceColor(d[i], s[i]);
        ctx.fillRect(x, y, cell, cell);

        // Distance labels when cells are large enough to read
        if (showLabels && s[i] === 5) {
            // Dark text on bright backgrounds (mid-range), light text on dark ones
            var t = (d[i] - 20) / 3980;
            ctx.fillStyle = (t > 0.15 && t < 0.8) ? 'rgba(0,0,0,0.8)' : 'rgba(255,255,255,0.9)';
            var label = d[i] >= 1000 ? (d[i] / 1000).toFixed(1) + 'm' : d[i] + '';
            ctx.fillText(label, x + cell / 2, y + cell / 2);
        }
    }
}

// ==========================================================================
// 3D IMU Visualization — Board Model + XYZ Axis Gizmo
// ==========================================================================
//
// Renders the GY-BNO085 PCB mesh (loaded from board.js) and three
// colored axis arrows, all rotating in real-time with the IMU quaternion.
//
// Axes match the sensor's physical board markings:
//   X = red   (long edge)   Y = green (short edge)   Z = blue (up through IC)
//
// Render pipeline:
//   1. Build rotation matrix from IMU quaternion (Z-up → Y-up remap)
//   2. Apply a fixed ~20° camera pitch for a 3/4 overhead view
//   3. Rotate all board vertices + axis endpoints by the combined matrix
//   4. Orthographic-project to 2D screen coords (no perspective distortion)
//   5. Board mesh: backface cull, Lambert shade, depth-sort (painter's algo), draw
//      - IC chip faces (vertex Y > 4) render in dark grey/black
//      - PCB body faces render in shaded green
//   6. Axis lines drawn on top with arrows and labels, depth-sorted

var AXIS_LEN = 100;
var AXIS_COLORS = ['#e94560', '#4ecca3', '#4a9eff'];
var AXIS_LABELS = ['X', 'Y', 'Z'];

// Sensor axis endpoints in Y-up model frame (after Z-up → Y-up remap)
var AXIS_VECTORS = [
    [AXIS_LEN, 0, 0],
    [0, 0, -AXIS_LEN],
    [0, AXIS_LEN, 0]
];

// Directional light vector for Lambert shading on the board mesh.
// Points from upper-right-front (x=0.2, y=0.7, z=-0.5), normalized to unit length.
// The dot product of each face normal with this vector determines shading intensity.
var BOARD_LIGHT = (function() {
    var x = 0.2, y = 0.7, z = -0.5;
    var l = Math.sqrt(x*x + y*y + z*z);
    return [x/l, y/l, z/l];
})();

// Convert a unit quaternion [w, x, y, z] to a 3×3 rotation matrix.
// Used to transform 3D vertices by the IMU orientation.
// The matrix is stored as an array of 3 row-arrays: [[r00,r01,r02], [r10,...], [r20,...]].
function quatToRotMatrix(w, x, y, z) {
    var xx=x*x, yy=y*y, zz=z*z, xy=x*y, xz=x*z, yz=y*z, wx=w*x, wy=w*y, wz=w*z;
    return [
        [1-2*(yy+zz), 2*(xy-wz),   2*(xz+wy)  ],
        [2*(xy+wz),   1-2*(xx+zz), 2*(yz-wx)  ],
        [2*(xz-wy),   2*(yz+wx),   1-2*(xx+yy)]
    ];
}

// Multiply a 3×3 rotation matrix by a 3D vector: result = M × v.
function matVecMul(m, v) {
    return [
        m[0][0]*v[0]+m[0][1]*v[1]+m[0][2]*v[2],
        m[1][0]*v[0]+m[1][1]*v[1]+m[1][2]*v[2],
        m[2][0]*v[0]+m[2][1]*v[1]+m[2][2]*v[2]
    ];
}

// ==========================================================================
// Main 3D Render — called 20x/sec with each IMU update
// ==========================================================================
function renderIMUViz(w, x, y, z) {
    if (!imuVizCtx) return;

    // Retina/HiDPI canvas setup: scale the pixel buffer by devicePixelRatio
    // so lines appear crisp on high-DPI screens (iPhones, MacBooks, etc.)
    // Only resize the canvas buffer when the CSS dimensions actually change.
    // Setting canvas.width/height clears the buffer and forces a GPU
    // reallocation — expensive at 20Hz when the size is usually constant.
    var dpr = window.devicePixelRatio || 1;
    var rect = imuVizCanvas.getBoundingClientRect();
    var needsResize = (rect.width !== cachedVizW || rect.height !== cachedVizH);
    if (needsResize) {
        cachedVizW = rect.width;
        cachedVizH = rect.height;
        imuVizCanvas.width = cachedVizW * dpr;
        imuVizCanvas.height = cachedVizH * dpr;
    }
    var ctx = imuVizCtx;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);

    var W = cachedVizW, H = cachedVizH;
    ctx.clearRect(0, 0, W, H);

    // Remap quaternion from BNO085's Z-up (ENU) to Y-up model frame.
    // Sensor: X=short edge, Y=long edge, Z=up.
    // Model:  X=long edge,  Y=up,        Z=-short edge.
    // Quaternion component mapping: [w, x, y, z] → [w, x, z, -y]
    var Rimu = quatToRotMatrix(w, x, z, -y);

    // Camera pitch: compose a ~20° downward tilt (0.35 rad) with the IMU
    // rotation. This gives a 3/4 overhead view so the board's top surface
    // is visible when the sensor points straight up. The tilt is a rotation
    // around the X axis applied after the IMU rotation: R_view = R_pitch × R_imu.
    var cp = Math.cos(0.35), sp = Math.sin(0.35);
    var R = [
        [Rimu[0][0], Rimu[0][1], Rimu[0][2]],
        [cp*Rimu[1][0]-sp*Rimu[2][0], cp*Rimu[1][1]-sp*Rimu[2][1], cp*Rimu[1][2]-sp*Rimu[2][2]],
        [sp*Rimu[1][0]+cp*Rimu[2][0], sp*Rimu[1][1]+cp*Rimu[2][1], sp*Rimu[1][2]+cp*Rimu[2][2]]
    ];

    // Orthographic projection: no perspective distortion, so the rectangular
    // board always looks rectangular regardless of rotation angle. The scale
    // factor maps model units to screen pixels based on the smaller canvas dimension.
    var midX = W / 2, midY = H / 2;
    var sc = Math.min(W, H) / 210;

    function project(p) {
        return { x: midX + p[0] * sc, y: midY - p[1] * sc };
    }

    // --- Board mesh (drawn first, behind axis lines) ---
    // BOARD_V and BOARD_F are loaded from board.js (generated by tools/stl_to_js.py).
    // BOARD_V is a flat array of vertex coordinates [x0,y0,z0, x1,y1,z1, ...].
    // BOARD_F is a flat array of face indices [a0,b0,c0, a1,b1,c1, ...].
    if (typeof BOARD_V !== 'undefined' && typeof BOARD_F !== 'undefined') {
        // Transform all vertices: rotate by combined view matrix, then project to 2D.
        var nv = BOARD_V.length / 3;
        var rv = new Array(nv);  // rotated 3D vertices
        var pv = new Array(nv);  // projected 2D screen coords
        for (var i = 0; i < nv; i++) {
            rv[i] = matVecMul(R, [BOARD_V[i*3], BOARD_V[i*3+1], BOARD_V[i*3+2]]);
            pv[i] = project(rv[i]);
        }

        // Process each triangle face: backface cull, compute shading, collect for sorting.
        var drawFaces = [];
        var nf = BOARD_F.length / 3;
        for (var i = 0; i < nf; i++) {
            var a = BOARD_F[i*3], b = BOARD_F[i*3+1], c = BOARD_F[i*3+2];
            var p0 = rv[a], p1 = rv[b], p2 = rv[c];

            // Compute face normal via cross product of two edge vectors.
            var ex = p1[0]-p0[0], ey = p1[1]-p0[1], ez = p1[2]-p0[2];
            var fx = p2[0]-p0[0], fy = p2[1]-p0[1], fz = p2[2]-p0[2];
            var nx = ey*fz - ez*fy, ny = ez*fx - ex*fz, nz = ex*fy - ey*fx;

            // Backface culling: skip faces pointing away from the camera.
            // In our coordinate system, camera looks along -Z, so faces
            // with nz >= 0 (normal pointing toward or away from camera) are back-facing.
            if (nz >= 0) continue;

            var len = Math.sqrt(nx*nx + ny*ny + nz*nz);
            if (len > 0) { nx /= len; ny /= len; nz /= len; }

            // Lambert shading: intensity = ambient + diffuse * max(0, N·L).
            // t ranges from 0.3 (fully shadowed) to 1.0 (fully lit).
            var dot = nx*BOARD_LIGHT[0] + ny*BOARD_LIGHT[1] + nz*BOARD_LIGHT[2];
            var t = 0.3 + 0.7 * Math.max(0, dot);

            // Depth key for painter's algorithm (draw far faces first).
            var avgZ = (p0[2] + p1[2] + p2[2]) / 3;

            // Classify face as IC chip or PCB body. The board mesh has 3 Y levels:
            //   -8.5 (PCB bottom), 0.9 (PCB top / IC base), 8.5 (IC top).
            // Any face touching the IC top level (Y > 4) is part of the IC package.
            var ic = BOARD_V[a*3+1] > 4 || BOARD_V[b*3+1] > 4 || BOARD_V[c*3+1] > 4;
            drawFaces.push({a: a, b: b, c: c, t: t, z: avgZ, ic: ic});
        }

        // Painter's algorithm: sort by depth (furthest first) so closer faces
        // overdraw farther ones, producing correct occlusion without a Z-buffer.
        drawFaces.sort(function(a, b) { return a.z - b.z; });

        for (var i = 0; i < drawFaces.length; i++) {
            var f = drawFaces[i];
            ctx.beginPath();
            ctx.moveTo(pv[f.a].x, pv[f.a].y);
            ctx.lineTo(pv[f.b].x, pv[f.b].y);
            ctx.lineTo(pv[f.c].x, pv[f.c].y);
            ctx.closePath();
            if (f.ic) {
                // IC chip: dark grey/black
                var c = (40*f.t|0);
                ctx.fillStyle = 'rgb(' + c + ',' + c + ',' + c + ')';
            } else {
                // PCB green: base rgb(30, 100, 55)
                ctx.fillStyle = 'rgb(' + (30*f.t|0) + ',' + (100*f.t|0) + ',' + (55*f.t|0) + ')';
            }
            ctx.fill();
        }
    }

    // --- Axis lines (drawn on top of board mesh) ---
    // Three colored arrows from the origin to each axis tip, with labels.
    // Depth-sorted so axes closer to the camera draw on top of farther ones.
    var originP = project([0, 0, 0]);
    var axes = [];
    for (var i = 0; i < 3; i++) {
        var tip3d = matVecMul(R, AXIS_VECTORS[i]);
        axes.push({ idx: i, tipP: project(tip3d), depth: tip3d[2] });
    }
    axes.sort(function(a, b) { return a.depth - b.depth; });

    for (var i = 0; i < axes.length; i++) {
        var a = axes[i];
        var color = AXIS_COLORS[a.idx];

        // Unit vector from origin to tip in screen space (for arrowhead direction)
        var dx = a.tipP.x - originP.x;
        var dy = a.tipP.y - originP.y;
        var len = Math.sqrt(dx * dx + dy * dy);
        if (len < 1) continue;
        var ux = dx / len, uy = dy / len;

        // Axis line
        ctx.beginPath();
        ctx.moveTo(originP.x, originP.y);
        ctx.lineTo(a.tipP.x, a.tipP.y);
        ctx.strokeStyle = color;
        ctx.lineWidth = 3;
        ctx.lineCap = 'round';
        ctx.stroke();

        // Arrowhead (filled triangle at the tip)
        var al = 10, aw = 5;
        ctx.beginPath();
        ctx.moveTo(a.tipP.x, a.tipP.y);
        ctx.lineTo(a.tipP.x - ux*al + uy*aw, a.tipP.y - uy*al - ux*aw);
        ctx.lineTo(a.tipP.x - ux*al - uy*aw, a.tipP.y - uy*al + ux*aw);
        ctx.closePath();
        ctx.fillStyle = color;
        ctx.fill();

        // Axis label (X/Y/Z) offset past the arrowhead
        ctx.fillStyle = color;
        ctx.font = 'bold 14px sans-serif';
        ctx.textAlign = 'center';
        ctx.textBaseline = 'middle';
        ctx.fillText(AXIS_LABELS[a.idx], a.tipP.x + ux * 14, a.tipP.y + uy * 14);
    }

    // Origin dot (small circle at the center of rotation)
    ctx.beginPath();
    ctx.arc(originP.x, originP.y, 3, 0, 2 * Math.PI);
    ctx.fillStyle = '#e0e0e0';
    ctx.fill();
}
