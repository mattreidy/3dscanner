// ==========================================================================
// SensorRing.h — ToF Sensor Ring Manager
// ==========================================================================
//
// Manages an array of up to 10 ToF sensors arranged in a 360-degree ring
// for room scanning. Each sensor occupies a slot with a known angular
// position (degrees around the ring). The ring abstracts over mock and
// real sensors via the ToFSensor interface.
//
// For now we use 1 mock sensor. When real hardware arrives, main.cpp
// will add 10 RealVL53L5CX instances at 36-degree intervals. Nothing
// in SensorRing or the rest of the pipeline needs to change.
//
// Memory: fixed array, no heap allocation. SensorRing holds pointers
// to sensor objects owned by main.cpp — it does not manage their lifetime.
// ==========================================================================

#pragma once

#include "ToFSensor.h"

static const uint8_t MAX_SENSORS = 10;

// One slot in the sensor ring: a ToF sensor at a known angular position.
struct SensorSlot {
    ToFSensor* sensor = nullptr;
    float angleDeg = 0.0f;  // Angular position around the ring (0-360)
    bool active = false;
};

class SensorRing {
public:
    // Add a sensor at a given angular position.
    // Returns slot index (0-9) or -1 if the ring is full.
    int8_t addSensor(ToFSensor* sensor, float angleDeg);

    // Initialize all added sensors. Returns count of successfully initialized sensors.
    uint8_t initAll();

    // Non-blocking update of all sensors. Call every loop() iteration.
    void update();

    // Number of active sensor slots
    uint8_t getCount() const;

    // Get a slot by index (0 to getCount()-1)
    const SensorSlot& getSlot(uint8_t index) const;

private:
    SensorSlot _slots[MAX_SENSORS];
    uint8_t _count = 0;
};
