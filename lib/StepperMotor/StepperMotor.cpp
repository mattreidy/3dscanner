// ==========================================================================
// StepperMotor.cpp — 28BYJ-48 Half-Step Driver Implementation
// ==========================================================================

#include "StepperMotor.h"

// Half-step sequence: 8 phases for smooth unipolar stepping.
// Each row is {IN1, IN2, IN3, IN4} — HIGH or LOW.
const uint8_t StepperMotor::HALF_STEP[8][4] = {
    {1, 0, 0, 0},
    {1, 1, 0, 0},
    {0, 1, 0, 0},
    {0, 1, 1, 0},
    {0, 0, 1, 0},
    {0, 0, 1, 1},
    {0, 0, 0, 1},
    {1, 0, 0, 1}
};

void StepperMotor::begin(uint8_t in1, uint8_t in2, uint8_t in3, uint8_t in4) {
    _pins[0] = in1;
    _pins[1] = in2;
    _pins[2] = in3;
    _pins[3] = in4;

    for (int i = 0; i < 4; i++) {
        pinMode(_pins[i], OUTPUT);
        digitalWrite(_pins[i], LOW);
    }

    computeStepDelay();

    xTaskCreatePinnedToCore(stepTask, "stepper", 2048, this, 1, &_taskHandle, 0);
    Serial.printf("[Stepper] Initialized on GPIO %d,%d,%d,%d\n", in1, in2, in3, in4);
}

void StepperMotor::stepTask(void* param) {
    StepperMotor* self = static_cast<StepperMotor*>(param);
    for (;;) {
        if (self->_running) {
            if (self->_clockwise) {
                self->_stepIndex = (self->_stepIndex + 1) & 7;
            } else {
                self->_stepIndex = (self->_stepIndex + 7) & 7;
            }
            self->writeStep(self->_stepIndex);
            delayMicroseconds(self->_stepDelayUs);
        } else {
            // De-energize all coils when stopped
            for (int i = 0; i < 4; i++) {
                digitalWrite(self->_pins[i], LOW);
            }
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
}

void StepperMotor::writeStep(uint8_t index) {
    for (int i = 0; i < 4; i++) {
        digitalWrite(_pins[i], HALF_STEP[index][i]);
    }
}

void StepperMotor::computeStepDelay() {
    _stepDelayUs = (uint32_t)(60000000UL / (_rpm * STEPS_PER_REV));
}

void StepperMotor::start() {
    _running = true;
    Serial.printf("[Stepper] Start %s at %.1f RPM\n", _clockwise ? "CW" : "CCW", _rpm);
}

void StepperMotor::stop() {
    _running = false;
    Serial.println("[Stepper] Stop");
}

void StepperMotor::setDirection(bool clockwise) {
    _clockwise = clockwise;
    Serial.printf("[Stepper] Direction: %s\n", clockwise ? "CW" : "CCW");
}

void StepperMotor::setSpeed(float rpm) {
    _rpm = constrain(rpm, MIN_RPM, MAX_RPM);
    computeStepDelay();
    Serial.printf("[Stepper] Speed: %.1f RPM (%lu us/step)\n", _rpm, _stepDelayUs);
}
