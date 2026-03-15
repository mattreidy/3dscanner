// ==========================================================================
// StepperMotor.h — 28BYJ-48 Stepper Motor Driver (ULN2003)
// ==========================================================================
//
// Half-step driver for the 28BYJ-48 unipolar stepper motor via ULN2003
// Darlington array. 8-phase half-step sequence gives 4096 steps per
// revolution (0.088 degrees/step) for smooth, quiet operation.
//
// Stepping runs in a FreeRTOS task on core 0 so it never blocks the
// main loop or web server. Speed, direction, and start/stop are
// controlled via thread-safe volatile state variables.
//
// When stopped, all coils are de-energized to prevent overheating.
// ==========================================================================

#pragma once

#include <Arduino.h>

class StepperMotor {
public:
    void begin(uint8_t in1, uint8_t in2, uint8_t in3, uint8_t in4);
    void start();
    void stop();
    void setDirection(bool clockwise);
    void setSpeed(float rpm);

    bool isRunning() const { return _running; }
    bool getDirection() const { return _clockwise; }
    float getSpeedRPM() const { return _rpm; }
    int32_t getStepCount() const { return _stepCount; }
    float getAngleDeg() const { return (_stepCount % STEPS_PER_REV) * 360.0f / STEPS_PER_REV; }
    void resetStepCount() { _stepCount = 0; }

private:
    uint8_t _pins[4];
    volatile bool _running = false;
    volatile bool _clockwise = true;
    volatile float _rpm = 6.0f;
    volatile uint32_t _stepDelayUs = 1464;
    volatile int32_t _stepCount = 0;
    uint8_t _stepIndex = 0;
    TaskHandle_t _taskHandle = nullptr;

    static const uint8_t HALF_STEP[8][4];
    static const uint16_t STEPS_PER_REV = 4096;
    static constexpr float MIN_RPM = 1.0f;
    static constexpr float MAX_RPM = 15.0f;

    static void stepTask(void* param);
    void writeStep(uint8_t index);
    void computeStepDelay();
};
