#include "MotorController.h"

constexpr double kP = 4;
constexpr double kI = 0;

constexpr double kV = 2.7304 / 0x7fffp0;
constexpr double kS = 3603. / 0x7fffp0;

constexpr double MAX_INTEGRAL = 0.15 / kV;

MotorController::MotorController(Motor& motor, Encoder& encoder):
    motor(motor), encoder(encoder) {
}

template<typename T>
T sign(T a) {
    return a == 0 ? 0 : a > 0 ? 1 : -1;
}

void MotorController::update() {
    int32_t currPos = encoder.read();
    int32_t dPos = currPos - prevPos;
    prevPos = currPos;

    double currentTs = micros() / 1e6;
    double dt = currentTs - prevTs;
    prevTs = currentTs;

    double velocity = dPos / dt;
    double error = targetVel - velocity;

    integral += kI * error * dt;
    integral = min(max(integral, -MAX_INTEGRAL), MAX_INTEGRAL);

    double outVel = kP * error + integral + targetVel;
    double outThrottle = kV * outVel + kS * sign(targetVel);

    motor.setThrottle(outThrottle);
}

void MotorController::setTarget(double velTicks) {
    targetVel = velTicks;
    update();
}