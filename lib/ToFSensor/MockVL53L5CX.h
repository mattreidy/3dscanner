// ==========================================================================
// MockVL53L5CX.h — Simulated VL53L5CX ToF Sensor
// ==========================================================================
//
// Generates synthetic 8x8 distance data at ~15Hz for development and
// testing before real hardware arrives. Implements the ToFSensor interface
// so it plugs directly into the SensorRing and SSE pipeline.
//
// Five test patterns exercise different aspects of the visualization:
//   FLAT_WALL    — uniform distance + noise (baseline rendering)
//   SPHERE       — parabolic depth bowl (gradient rendering)
//   MOVING_BLOB  — orbiting gaussian hot spot (animation/temporal)
//   ANGLED_PLANE — linear gradient across rows (tilt visualization)
//   RANDOM_NOISE — random distances, some invalid zones (robustness)
//
// Pattern is selectable at construction and changeable at runtime.
// ==========================================================================

#pragma once

#include "ToFSensor.h"

enum class MockPattern : uint8_t {
    FLAT_WALL,
    SPHERE,
    MOVING_BLOB,
    ANGLED_PLANE,
    RANDOM_NOISE
};

class MockVL53L5CX : public ToFSensor {
public:
    explicit MockVL53L5CX(MockPattern pattern = MockPattern::FLAT_WALL);

    bool init() override;
    void update() override;
    bool isReady() const override;
    bool hasNewData() const override;
    const ToFFrame& getFrame() const override;

    void setPattern(MockPattern pattern);
    MockPattern getPattern() const;

private:
    MockPattern _pattern;
    ToFFrame _frame;
    bool _ready = false;
    bool _newData = false;
    uint32_t _lastUpdate = 0;
    uint32_t _frameCount = 0;

    // ~15Hz update rate, matching the real VL53L5CX default
    static const uint32_t UPDATE_INTERVAL_MS = 66;

    void generateFlatWall();
    void generateSphere();
    void generateMovingBlob();
    void generateAngledPlane();
    void generateRandomNoise();
};
