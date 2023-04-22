#include <Arduino.h>
#include <Encoder.h>

#include <Motor.h>

// some rough testing suggests min 5k based on electrical time constant
#define FREQ 10000
#define RES 15

Motor::Motor(uint8_t ph, uint8_t en): ph(ph), en(en) {
    pinMode(en, OUTPUT);
    pinMode(ph, OUTPUT);
    
    analogWriteFrequency(en, FREQ);
    analogWriteResolution(RES);
}

void Motor::setThrottle(double throttle) {
    throttle = min(max(throttle, -1), 1);
    int16_t pwm_duty = abs(throttle) * 0x7fffp0;
    if (throttle > 0) {
        digitalWrite(ph, HIGH);
    } else {
        digitalWrite(ph, LOW);
    }
    analogWrite(en, pwm_duty);
}