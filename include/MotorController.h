#pragma once

#include <Encoder.h>

#include "Motor.h"

class MotorController {
    private:
        Motor& motor;
        Encoder& encoder;

        double targetVel = 0;

        int32_t prevPos = 0;
        double prevTs = 0;
        double integral = 0;

    public:
        MotorController(Motor& motor, Encoder& encoder);
        void setTarget(double velTicks);
        int32_t readEncoder();
        void update();
};
