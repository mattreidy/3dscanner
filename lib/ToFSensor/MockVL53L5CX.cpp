// ==========================================================================
// MockVL53L5CX.cpp — Simulated VL53L5CX ToF Sensor Implementation
// ==========================================================================

#include "MockVL53L5CX.h"
#include <math.h>

MockVL53L5CX::MockVL53L5CX(MockPattern pattern) : _pattern(pattern) {
    memset(&_frame, 0, sizeof(_frame));
}

bool MockVL53L5CX::init() {
    _ready = true;
    _lastUpdate = millis();
    _frameCount = 0;
    Serial.printf("[MockToF] Initialized with pattern %d\n", (int)_pattern);
    return true;
}

void MockVL53L5CX::update() {
    if (!_ready) return;

    uint32_t now = millis();
    if (now - _lastUpdate < UPDATE_INTERVAL_MS) return;
    _lastUpdate = now;
    _frameCount++;

    switch (_pattern) {
        case MockPattern::FLAT_WALL:    generateFlatWall();    break;
        case MockPattern::SPHERE:       generateSphere();      break;
        case MockPattern::MOVING_BLOB:  generateMovingBlob();  break;
        case MockPattern::ANGLED_PLANE: generateAngledPlane(); break;
        case MockPattern::RANDOM_NOISE: generateRandomNoise(); break;
    }

    _frame.timestamp = now;
    _newData = true;
}

bool MockVL53L5CX::isReady() const { return _ready; }

bool MockVL53L5CX::hasNewData() const { return _newData; }

const ToFFrame& MockVL53L5CX::getFrame() const {
    // Clear the new-data flag on read (const_cast needed since getFrame is const)
    const_cast<MockVL53L5CX*>(this)->_newData = false;
    return _frame;
}

void MockVL53L5CX::setPattern(MockPattern pattern) { _pattern = pattern; }
MockPattern MockVL53L5CX::getPattern() const { return _pattern; }

// ---------- Test Pattern Generators ----------

// FLAT_WALL: uniform distance with small random noise.
// Simulates the sensor pointing at a flat surface 1.5m away.
void MockVL53L5CX::generateFlatWall() {
    for (int i = 0; i < TOF_ZONES; i++) {
        _frame.distance_mm[i] = 1500 + (int16_t)(random(-30, 31));
        _frame.status[i] = TOF_STATUS_VALID;
    }
}

// SPHERE: parabolic depth bowl — closer in center, farther at edges.
// Simulates a convex object (like a ball) centered in the field of view.
void MockVL53L5CX::generateSphere() {
    for (int i = 0; i < TOF_ZONES; i++) {
        int row = i / TOF_GRID_SIZE;
        int col = i % TOF_GRID_SIZE;
        float dx = col - 3.5f;
        float dy = row - 3.5f;
        float distSq = dx * dx + dy * dy;
        float maxDistSq = 3.5f * 3.5f + 3.5f * 3.5f; // corner distance squared
        int16_t dist = 800 + (int16_t)(1700.0f * distSq / maxDistSq);
        _frame.distance_mm[i] = dist + (int16_t)(random(-15, 16));
        _frame.status[i] = TOF_STATUS_VALID;
    }
}

// MOVING_BLOB: gaussian hot spot that orbits around the grid.
// Background at 2000mm, blob peak at 500mm. Tests animation smoothness.
void MockVL53L5CX::generateMovingBlob() {
    float angle = _frameCount * 0.05f; // slow orbit
    float cx = 3.5f + 2.0f * cosf(angle);
    float cy = 3.5f + 2.0f * sinf(angle);

    for (int i = 0; i < TOF_ZONES; i++) {
        int row = i / TOF_GRID_SIZE;
        int col = i % TOF_GRID_SIZE;
        float dx = col - cx;
        float dy = row - cy;
        float distSq = dx * dx + dy * dy;
        // Gaussian falloff: sigma ~1.5 zones
        float gaussian = expf(-distSq / 4.5f);
        int16_t dist = 2000 - (int16_t)(1500.0f * gaussian);
        _frame.distance_mm[i] = dist + (int16_t)(random(-10, 11));
        _frame.status[i] = TOF_STATUS_VALID;
    }
}

// ANGLED_PLANE: linear distance gradient across rows.
// Simulates the sensor viewing a tilted surface.
void MockVL53L5CX::generateAngledPlane() {
    for (int i = 0; i < TOF_ZONES; i++) {
        int row = i / TOF_GRID_SIZE;
        int16_t dist = 500 + (int16_t)(row * 2500.0f / 7.0f);
        _frame.distance_mm[i] = dist + (int16_t)(random(-20, 21));
        _frame.status[i] = TOF_STATUS_VALID;
    }
}

// RANDOM_NOISE: random distances across the full sensor range.
// ~10% of zones are marked invalid to test error handling in the UI.
void MockVL53L5CX::generateRandomNoise() {
    for (int i = 0; i < TOF_ZONES; i++) {
        _frame.distance_mm[i] = (int16_t)random(TOF_MIN_RANGE_MM, TOF_MAX_RANGE_MM + 1);
        _frame.status[i] = (random(0, 10) == 0) ? 0 : TOF_STATUS_VALID;
    }
}
