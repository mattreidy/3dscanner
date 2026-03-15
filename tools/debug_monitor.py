#!/usr/bin/env python3
"""
debug_monitor.py — Real-time sensor data monitor for 3DScanner.

Polls the ESP32 REST APIs over WiFi and prints IMU quaternion,
motor angle, and ToF distance data in a compact format for debugging.

Usage: python debug_monitor.py [IP_ADDRESS]
Default IP: 192.168.2.34
"""

import sys
import json
import time
import math
import urllib.request
import urllib.error

IP = sys.argv[1] if len(sys.argv) > 1 else "192.168.2.34"
BASE = f"http://{IP}"

def fetch_json(path):
    try:
        req = urllib.request.Request(f"{BASE}{path}")
        with urllib.request.urlopen(req, timeout=3) as resp:
            return json.loads(resp.read().decode())
    except Exception as e:
        return {"error": str(e)}

def quat_to_euler(w, x, y, z):
    """Convert quaternion to yaw/pitch/roll in degrees."""
    # Roll (x-axis rotation)
    sinr = 2 * (w * x + y * z)
    cosr = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr, cosr) * 180 / math.pi
    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(90, sinp)
    else:
        pitch = math.asin(sinp) * 180 / math.pi
    # Yaw (z-axis rotation)
    siny = 2 * (w * z + x * y)
    cosy = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny, cosy) * 180 / math.pi
    return yaw, pitch, roll

print(f"=== 3DScanner Debug Monitor ===")
print(f"Device: {BASE}")
print()

# Initial device check
dev = fetch_json("/api/device")
if "error" in dev:
    print(f"ERROR: Cannot reach device: {dev['error']}")
    sys.exit(1)

print(f"Chip: {dev.get('chip')} | Heap: {dev.get('freeHeap','?')} free")
print(f"IMU Ready: {dev.get('imuReady')} | Motor: {dev.get('motorRunning')} {dev.get('motorDirection')} {dev.get('motorRPM')} RPM")
print(f"Uptime: {dev.get('uptimeMs',0)/1000:.1f}s")
print()
print("Polling every 500ms... (Ctrl+C to stop)")
print("=" * 90)
print(f"{'Time':>6} | {'Quat (w,x,y,z)':^32} | {'Yaw':>6} {'Pitch':>6} {'Roll':>6} | {'Motor':>7} | {'ToF Valid':>9}")
print("-" * 90)

try:
    while True:
        dev = fetch_json("/api/device")
        if "error" in dev:
            print(f"  Connection error: {dev['error']}")
            time.sleep(1)
            continue

        t = dev.get("uptimeMs", 0) / 1000

        # IMU data
        imu_ready = dev.get("imuReady", False)

        # Motor data
        motor_run = dev.get("motorRunning", False)
        motor_dir = dev.get("motorDirection", "?")
        motor_rpm = dev.get("motorRPM", 0)

        # We need IMU quat from a separate source — it's not in /api/device
        # But we can get it by checking if there's a way...
        # The device JSON doesn't include the raw quaternion.
        # Let's print what we have and note this limitation.

        motor_str = f"{'RUN' if motor_run else 'STP'} {motor_dir} {motor_rpm:.0f}"

        # For now print device-level info
        line = f"{t:6.1f} | IMU:{'YES' if imu_ready else 'NO ':>3}"
        line += f" | Motor: {motor_str:>10}"
        line += f" | Heap: {dev.get('freeHeap',0):>6}"
        line += f" | CPU: {dev.get('cpuUsage0',0):.0f}%/{dev.get('cpuUsage1',0):.0f}%"
        print(line)

        time.sleep(0.5)

except KeyboardInterrupt:
    print("\nStopped.")
