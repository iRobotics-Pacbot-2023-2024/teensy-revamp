#include <Arduino.h>
#include <Encoder.h>
#include <TaskScheduler.h>

#include "imu.h"
#include "motors.h"

Scheduler ts{};

void setup() {
    Serial.begin(115200);

    imuInit();
    motorsInit(ts);
}

void loop() {
    ts.execute();

    if (imuUpdateReadings()) {
        double yawDeg = imuGetYaw() * 180 / M_PI;
        double angVelDeg = imuGetAngVel() * 180 / M_PI;
        Serial.printf("yaw: %f, ang vel: %f\n", yawDeg, angVelDeg);
    }
    delay(1);
}