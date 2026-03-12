// ==========================================================================
// RealVL53L5CX.cpp — Real VL53L5CX ToF Sensor Driver Implementation
// ==========================================================================

#include "RealVL53L5CX.h"

RealVL53L5CX::RealVL53L5CX(TwoWire& wire, uint8_t address)
    : _wire(wire), _address(address) {
    memset(&_frame, 0, sizeof(_frame));
    memset(&_rawData, 0, sizeof(_rawData));
}

bool RealVL53L5CX::init() {
    Serial.printf("[ToF] Initializing VL53L5CX at 0x%02X...\n", _address);
    Serial.println("[ToF] Uploading firmware (~2s)...");

    if (!_sensor.begin(_address, _wire)) {
        Serial.println("[ToF] *** VL53L5CX not found! Check wiring. ***");
        return false;
    }
    Serial.println("[ToF] Firmware loaded OK");

    // 8x8 resolution = 64 zones
    _sensor.setResolution(8 * 8);
    int resolution = _sensor.getResolution();
    Serial.printf("[ToF] Resolution: %dx%d (%d zones)\n",
                  resolution == 64 ? 8 : 4,
                  resolution == 64 ? 8 : 4,
                  resolution);

    // 15Hz is the max for 8x8 mode
    _sensor.setRangingFrequency(15);
    Serial.println("[ToF] Ranging frequency: 15Hz");

    _sensor.startRanging();
    Serial.println("[ToF] Ranging started");

    _ready = true;
    return true;
}

void RealVL53L5CX::update() {
    if (!_ready) return;

    if (_sensor.isDataReady()) {
        if (_sensor.getRangingData(&_rawData)) {
            // Copy raw data into our ToFFrame format
            for (uint8_t i = 0; i < TOF_ZONES; i++) {
                _frame.distance_mm[i] = _rawData.distance_mm[i];
                _frame.status[i] = _rawData.target_status[i];
            }
            _frame.timestamp = millis();
            _newData = true;
        }
    }
}

bool RealVL53L5CX::isReady() const {
    return _ready;
}

bool RealVL53L5CX::hasNewData() const {
    return _newData;
}

const ToFFrame& RealVL53L5CX::getFrame() const {
    const_cast<RealVL53L5CX*>(this)->_newData = false;
    return _frame;
}
