#include <Arduino.h>
#include <Encoder.h>
#include <Wire.h>

#include "imu.h"
#include "motors.h"
#include "tofs.h"

// TODO: actually tune these
/* for 4 inches/sec 

constexpr double kPRot = 1.50;
constexpr double kDRot = 0.00;

constexpr double movementSpeed = 4;

constexpr double pTOF = -0.6;
constexpr double avgDist = 1.55;
constexpr double thresh_stop = 3;


*/


constexpr double kPRot = 3.00;
constexpr double kDRot = 0.50;

constexpr double movementSpeed = 7;

constexpr double pTOF = -1.5;
constexpr double avgDist = 1.55;
constexpr double thresh_stop = 3;

enum class MovementDirection {
    NORTH,
    EAST,
    SOUTH,
    WEST,
    NONE
};

MovementDirection movementDirection = MovementDirection::NONE;

double targetYaw = 0;
double x, y = 0;
void setup() {
    Serial.begin(115200);
    Serial5.begin(115200);

    imuInit();
    tofInit();

    Wire.setClock(1000000);

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
void resetOdom(double& x, double&y, char dir);
void incOdom(double& x, double& y, double& theta, int flEnc, int frEnc, int blEnc, int brEnc);

void loop() {
    uint32_t start = micros();

    // updateDirectionFromSerial();
   
    movementDirection = MovementDirection::WEST;
    // Serial.printf("1!!! time: %d\n", micros() - start);

    imuUpdateReadings();


    // Serial.printf("2!!! time: %d\n", micros() - start);
    tofUpdateReadings();
    // Serial.printf("3!!! time: %d\n", micros() - start);
    if (!isCurrDirectionSafe())
    {
        movementDirection = MovementDirection::NONE;
    }

    double fw_vel = 0;
    double lateral_vel = 0;
    double turn_vel = 0;

    double yaw = imuGetYaw();
    double angVel = imuGetAngVel();

    int32_t [] encoder = getEncoderValues();
    printf("Encoders: rl: %d rr: %d fl: %d fr: %d", encoder[0], encoder[1], encoder[2], encoder[3]);
    incOdom(x, y, yaw, encoder[0], encoder[1], encoder[2], encoder[3]);

    Serial.printf("yaw: %f, ang vel: %f\n", (yaw) * 180 / M_PI, angVel * 180 / M_PI);

    if (movementDirection != MovementDirection::NONE) {
        baseMovement(fw_vel, lateral_vel);
        tofFeedback(fw_vel, lateral_vel);
        imuFeedback(turn_vel);
    }

    Serial.printf("fw: %f, lat: %f, turn: %f\n", fw_vel, lateral_vel, turn_vel);
    Serial.printf("direction: %d\n", static_cast<int>(movementDirection));

    motorsSetVelocity(fw_vel, lateral_vel, turn_vel);
    // Serial.printf("4!!! time: %d\n", micros() - start);

    motorsUpdate();
    MotorCo
    // Serial.printf("5!!! time: %d\n", micros() - start);

    // delay(20);

}


void updateDirectionFromSerial() {
    static uint32_t lastSerialUpdate = 0;

    while (Serial5.available()) {
        char c = Serial5.read();
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

    if (millis() - lastSerialUpdate > 1000) {
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

    Serial.printf("left: %f, right: %f, rear: %f, left: %f\n", front, right, rear, left);

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

    // Serial.printf("front: %f, right: %f, rear: %f, left: %f\n", front, right, rear, left);

    fw_vel -= tofVelocity_calc(front, avgDist, pTOF);
    fw_vel += tofVelocity_calc(rear, avgDist, pTOF);
    lateral_vel -= tofVelocity_calc(left, avgDist, pTOF);
    lateral_vel += tofVelocity_calc(right, avgDist, pTOF);
}

void imuFeedback(double& turn_vel) {
    double yaw = imuGetYaw();
    double angVel = imuGetAngVel();

    //Serial.printf("yaw: %f, ang vel: %f\n", (yaw - targetYaw) * 180 / M_PI, angVel * 180 / M_PI);

    turn_vel = kPRot * (targetYaw - yaw) + kDRot * -angVel;
}

void resetOdom(double& x, double&y, char dir){
    switch(dir){
        case 'n': 
            y = 0.0;
            break;
        case 's': 
            y = 0.0;
            break;
        case 'e': 
            x = 0.0;
            break;
        case 'w': 
            x = 0.0;
            break;

        case 'r':
            x = 0.0;
            y = 0.0;
            break;

        default:
            break;
    }
}
const double sn = 1/(pow(2,.5));
void incOdom(double& x, double& y, double& theta, int32_t blEnc, int32_t brEnc, int32_t flEnc, int32_t frEnc){
    
    double x_new = 0, y_new = 0;
    x_new += sn*(flEnc + frEnc - brEnc - blEnc);
    y_new += sn*(flEnc - frEnc - brEnc + blEnc);

    x += x_new*cos(theta) - y_new*sin(theta);
    y += x_new*sin(theta) + y_new*cos(theta); 
    Serial.printf("x: %f, y: %f\n", x, y );
}