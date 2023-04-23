#include <Arduino.h>
#include <Encoder.h>

#include "imu.h"
#include "motors.h"
#include "tofs.h"

enum class MovementDirection {
    NORTH,
    EAST,
    SOUTH,
    WEST,
    NONE
};

MovementDirection movementDirection = MovementDirection::NONE;

double targetYaw = 0;

void setup() {
    Serial.begin(115200);

    imuInit();
    tofInit();

    delay(5000);
    while (!imuUpdateReadings()) {
        Serial.println("Waiting for IMU");
        delay(100);
    }
    targetYaw = imuGetYaw();
}

constexpr double kP = 0.5;
constexpr double kD = 0.1;

constexpr double movementSpeed = 16;

uint32_t lastDirectionChangeTime = 0;

void loop() {
    while (Serial1.available()) {
        lastDirectionChangeTime = millis();

        char c = Serial1.read();
        Serial1.flush();
        switch (c) {
            case 'n':
                movementDirection = MovementDirection::NORTH;
                break;
            case 'e':
                movementDirection = MovementDirection::EAST;
                break;
            case 's':
                movementDirection = MovementDirection::SOUTH;
                break;
            case 'w':
                movementDirection = MovementDirection::WEST;
                break;
            case 'r':
                targetYaw = imuGetYaw();
                movementDirection = MovementDirection::NONE; 
                break;
            default:
                movementDirection = MovementDirection::NONE;
                break;
        }
    }

    if (millis() - lastDirectionChangeTime > 300) {
        movementDirection = MovementDirection::NONE;
    }

    imuUpdateReadings();
    tofUpdateReadings();

    double yaw = imuGetYaw();
    double angVel = imuGetAngVel();

    double front = tofGetFrontIn();
    double right = tofGetRightIn();
    double rear = tofGetRearIn();
    double left = tofGetLeftIn();

    double fw_vel = 0;
    double lateral_vel = 0;
    double turn_vel = 0;

    switch (movementDirection) {
        case MovementDirection::NORTH:
            fw_vel = movementSpeed;
            break;
        case MovementDirection::EAST:
            lateral_vel = -movementSpeed;
            break;
        case MovementDirection::SOUTH:
            fw_vel = -movementSpeed;
            break;
        case MovementDirection::WEST:
            lateral_vel = movementSpeed;
            break;
        default:
            break;
    }

    if (movementDirection != MovementDirection::NONE) {
        if (front < 1) {
            fw_vel -= 2;
        } else if (right < 1) {
            lateral_vel += 2;
        } else if (left < 1) {
            lateral_vel -= 2;
        } else if (rear < 1) {
            fw_vel += 2;
        }

        turn_vel = kP * (targetYaw - yaw) + kD * -angVel;
    }

    Serial.printf("front: %f, right: %f, rear: %f, left: %f\n", front, right, rear, left);
    Serial.printf("yaw: %f, ang vel: %f\n", yaw * 180 / M_PI, angVel * 180 / M_PI);
    Serial.printf("fw: %f, lat: %f, turn: %f\n", fw_vel, lateral_vel, turn_vel);
    Serial.printf("direction: %d\n", static_cast<int>(movementDirection));

    motorsSetVelocity(fw_vel, lateral_vel, turn_vel);

    delay(20);
}