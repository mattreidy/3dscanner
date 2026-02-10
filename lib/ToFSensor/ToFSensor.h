// ==========================================================================
// ToFSensor.h — Abstract Time-of-Flight Sensor Interface
// ==========================================================================
//
// Defines the data format and interface for any VL53L5CX-compatible ToF
// sensor (mock or real hardware). The 3DScanner's sensor ring manages
// an array of ToFSensor pointers, so the rest of the system doesn't care
// whether it's talking to a mock pattern generator or a real sensor
// behind a TCA9548A I2C multiplexer.
//
// Data types (int16_t distances, uint8_t status) match the SparkFun
// VL53L5CX library's VL53L5CX_ResultsData exactly, so the real driver
// can copy sensor data straight into ToFFrame without conversion.
//
// The VL53L5CX outputs an 8x8 grid (64 zones) of perpendicular distance
// measurements in millimeters, plus per-zone target status codes.
// Valid range: 20–4000 mm. Status 5 = valid measurement.
// ==========================================================================

#pragma once

#include <Arduino.h>

// Sensor constants
static const uint8_t TOF_ZONES = 64;         // 8x8 zone grid
static const uint8_t TOF_GRID_SIZE = 8;      // Zones per row/column
static const int16_t TOF_MIN_RANGE_MM = 20;  // Minimum valid distance
static const int16_t TOF_MAX_RANGE_MM = 4000; // Maximum valid distance
static const uint8_t TOF_STATUS_VALID = 5;   // Status code for a valid measurement

// Data from a single ToF sensor frame.
// One frame = one complete 8x8 ranging measurement.
struct ToFFrame {
    int16_t distance_mm[TOF_ZONES];  // Distance per zone in mm
    uint8_t status[TOF_ZONES];       // Per-zone target status
    uint32_t timestamp;              // millis() when this frame was captured
};

// Abstract interface for any ToF sensor implementation.
// Mock sensors generate synthetic data; real sensors wrap the SparkFun driver.
class ToFSensor {
public:
    virtual ~ToFSensor() = default;

    // Initialize the sensor. Returns true on success.
    virtual bool init() = 0;

    // Non-blocking poll for new data. Call every loop() iteration.
    // Returns immediately if no new data is available.
    virtual void update() = 0;

    // True if the sensor initialized successfully and is producing data.
    virtual bool isReady() const = 0;

    // True if update() captured a new frame since the last call to getFrame().
    virtual bool hasNewData() const = 0;

    // Get the most recent frame. Only valid after isReady() returns true.
    virtual const ToFFrame& getFrame() const = 0;
};
