#include <Arduino.h>
#include <Encoder.h>

#include "imu.h"
#include "motors.h"
#include "tofs.h"

// TODO: actually tune these
constexpr double kPRot = 0.5;
constexpr double kDRot = 0.5;

constexpr double movementSpeed = 16;

constexpr double pTOF = 1;
constexpr double avgDist = 1;
constexpr double thresh_stop = 1;

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

void updateDirectionFromSerial();
bool isCurrDirectionSafe();
void baseMovement(double& fw_vel, double& lateral_vel);
void tofFeedback(double& fw_vel, double& lateral_vel);
void imuFeedback(double& turn_vel);

void loop() {
    updateDirectionFromSerial();

    imuUpdateReadings();
    tofUpdateReadings();

    if (!isCurrDirectionSafe()) {
        movementDirection = MovementDirection::NONE;
    }

    double fw_vel = 0;
    double lateral_vel = 0;
    double turn_vel = 0;

    if (movementDirection != MovementDirection::NONE) {
        baseMovement(fw_vel, lateral_vel);
        tofFeedback(fw_vel, lateral_vel);
        imuFeedback(turn_vel);
    }

    Serial.printf("fw: %f, lat: %f, turn: %f\n", fw_vel, lateral_vel, turn_vel);
    Serial.printf("direction: %d\n", static_cast<int>(movementDirection));

    motorsSetVelocity(fw_vel, lateral_vel, turn_vel);

    delay(20);
}


void updateDirectionFromSerial() {
    static uint32_t lastSerialUpdate = 0;

    while (Serial1.available()) {
        char c = Serial1.read();
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

        lastSerialUpdate = millis();
    }

    if (millis() - lastSerialUpdate > 300) {
        movementDirection = MovementDirection::NONE;
    }
}

bool isCurrDirectionSafe() {
    if (movementDirection == MovementDirection::NONE) {
        return false;
    }

    double front = tofGetFrontIn();
    double right = tofGetRightIn();
    double rear = tofGetRearIn();
    double left = tofGetLeftIn();

    switch (movementDirection) {
        case MovementDirection::NORTH:
            return front > thresh_stop;
        case MovementDirection::EAST:
            return right > thresh_stop;
        case MovementDirection::SOUTH:
            return rear > thresh_stop;
        case MovementDirection::WEST:
            return left > thresh_stop;
        default:
            return false;
    }
}

void baseMovement(double& fw_vel, double& lateral_vel) {
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
}

double tofVelocity_calc(double tof, double avgDist, double p) {
    double dist = min(tof, avgDist);
    return p*dist;
}

void tofFeedback(double& fw_vel, double& lateral_vel) {
    double front = tofGetFrontIn();
    double right = tofGetRightIn();
    double rear = tofGetRearIn();
    double left = tofGetLeftIn();

    Serial.printf("front: %f, right: %f, rear: %f, left: %f\n", front, right, rear, left);

    fw_vel -= tofVelocity_calc(front, avgDist, pTOF);
    fw_vel += tofVelocity_calc(rear, avgDist, pTOF);
    lateral_vel -= tofVelocity_calc(left, avgDist, pTOF);
    lateral_vel += tofVelocity_calc(right, avgDist, pTOF);
}

void imuFeedback(double& turn_vel) {
    double yaw = imuGetYaw();
    double angVel = imuGetAngVel();

    Serial.printf("yaw: %f, ang vel: %f\n", yaw * 180 / M_PI, angVel * 180 / M_PI);

    turn_vel = kPRot * (targetYaw - yaw) + kDRot * -angVel;
}