#pragma once

#include <Encoder.h>

#include "Motor.h"

constexpr double MOTOR_UPDATE_INTERVAL_MS = 20;

class MotorController {
    private:
        Motor& motor;
        Encoder& encoder;

        double targetVel = 0;

        int32_t prevPos = 0;
        double integral = 0;

    public:
        MotorController(Motor& motor, Encoder& encoder);

        void setTarget(double velTicks);
        void update();
};
