#pragma once

#include <Arduino.h>
#include <Encoder.h>

class Motor {
    private:
        uint8_t ph, en;

    public:
        Motor(uint8_t ph, uint8_t en);
        
        void setThrottle(double throttle);
};
