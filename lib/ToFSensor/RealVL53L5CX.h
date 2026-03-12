// ==========================================================================
// RealVL53L5CX.h — Real VL53L5CX ToF Sensor Driver
// ==========================================================================
//
// Wraps the SparkFun VL53L5CX Arduino Library to implement the ToFSensor
// interface. Handles initialization (firmware upload takes ~2s), 8x8
// resolution config, and non-blocking data polling.
//
// The VL53L5CX requires ~90KB firmware loaded over I2C at every power-on.
// init() blocks during this upload. After that, update() is non-blocking.
//
// I2C address is 0x29 (7-bit) by default. For multi-sensor setups with
// a TCA9548A mux, each sensor appears on its own mux channel so they
// can all use the default address.
// ==========================================================================

#pragma once

#include "ToFSensor.h"
#include <Wire.h>
#include <SparkFun_VL53L5CX_Library.h>

class RealVL53L5CX : public ToFSensor {
public:
    explicit RealVL53L5CX(TwoWire& wire = Wire, uint8_t address = 0x29);

    bool init() override;
    void update() override;
    bool isReady() const override;
    bool hasNewData() const override;
    const ToFFrame& getFrame() const override;

private:
    SparkFun_VL53L5CX _sensor;
    TwoWire& _wire;
    uint8_t _address;
    ToFFrame _frame;
    VL53L5CX_ResultsData _rawData;
    bool _ready = false;
    bool _newData = false;
};
