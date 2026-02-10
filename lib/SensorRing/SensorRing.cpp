// ==========================================================================
// SensorRing.cpp — ToF Sensor Ring Manager Implementation
// ==========================================================================

#include "SensorRing.h"

int8_t SensorRing::addSensor(ToFSensor* sensor, float angleDeg) {
    if (_count >= MAX_SENSORS) {
        Serial.println("[SensorRing] *** Ring is full (max 10 sensors) ***");
        return -1;
    }

    _slots[_count].sensor = sensor;
    _slots[_count].angleDeg = angleDeg;
    _slots[_count].active = true;
    Serial.printf("[SensorRing] Added sensor %d at %.1f degrees\n", _count, angleDeg);
    return _count++;
}

uint8_t SensorRing::initAll() {
    uint8_t successCount = 0;
    for (uint8_t i = 0; i < _count; i++) {
        if (_slots[i].active && _slots[i].sensor) {
            if (_slots[i].sensor->init()) {
                successCount++;
                Serial.printf("[SensorRing] Sensor %d initialized OK\n", i);
            } else {
                _slots[i].active = false;
                Serial.printf("[SensorRing] *** Sensor %d init FAILED ***\n", i);
            }
        }
    }
    return successCount;
}

void SensorRing::update() {
    for (uint8_t i = 0; i < _count; i++) {
        if (_slots[i].active && _slots[i].sensor) {
            _slots[i].sensor->update();
        }
    }
}

uint8_t SensorRing::getCount() const {
    return _count;
}

const SensorSlot& SensorRing::getSlot(uint8_t index) const {
    return _slots[index];
}
