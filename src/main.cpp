#include <Arduino.h>
#include <Encoder.h>
#include <Wire.h>
#include <cstdint>

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



constexpr double kPRot = 12;

constexpr double movementSpeed = 16;

constexpr double pTOF = 4;
constexpr double avgDist = 1.55;
constexpr double maxDist = 3;
constexpr double thresh_stop = 3;

bool hasReset = false;
double x_in_cell = 0;
double y_in_cell = 0;
const double maxDistUpdate = 3.5;

const double minCentered = 2;
const double maxCentered = 4;

enum class MovementDirection {
    NORTH,
    EAST,
    SOUTH,
    WEST,
    NONE,
    STOP
};

MovementDirection movementDirection = MovementDirection::STOP;

double targetYaw = 0;
double x, y = 0;
double curr_encoders[4];
double prev_encoders[4];
double d_encoders[4];
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

    for(int i = 0; i < 4; i++){
        curr_encoders[i] = 0;
        prev_encoders[i] = 0;
        d_encoders[i] = 0;
    }
    hasReset = false;
    movementDirection = MovementDirection::NORTH;
}

void updateDirectionFromSerial();
bool isCurrDirectionSafe();
void baseMovement(double& fw_vel, double& lateral_vel);
void tofFeedback(double& fw_vel, double& lateral_vel);
void imuFeedback(double& turn_vel);
void resetOdom(double& x, double&y, char dir);
void incOdom(double& x, double& y, double& theta, double flEnc, double frEnc, double blEnc, double brEnc);
bool isCentered(double& x_in_cell, double& y_in_cell);

int count = 0;
void loop() {
    uint32_t start = micros();

    imuUpdateReadings();

    // Serial.printf("2!!! time: %d\n", micros() - start);
    tofUpdateReadings();
    // Serial.printf("3!!! time: %d\n", micros() - start);
    if (!isCurrDirectionSafe() && movementDirection != MovementDirection::STOP) {
        movementDirection = MovementDirection::NONE;
    }

    double fw_vel = 0;
    double lateral_vel = 0;
    double turn_vel = 0;

    double yaw = imuGetYaw();
 //   double angVel = imuGetAngVel();

    getEncodersValues(curr_encoders);
    for(int i = 0; i < 4; i++){
        d_encoders[i] = curr_encoders[i] - prev_encoders[i];
    }
    for(int i = 0; i < 4; i++){
        prev_encoders[i] = curr_encoders[i];
    }

   // printf("Encoders: rl: %d rr: %d fl: %d fr: %d", d_encoders[0], d_encoders[1], d_encoders[2], d_encoders[3]);
    incOdom(x, y, yaw, d_encoders[0], d_encoders[1], d_encoders[2], d_encoders[3]);

    
    if(isCentered(x_in_cell, y_in_cell) && hasReset){
        hasReset = false;
        Serial5.print('&');
        Serial5.print('w');

        count += 1;
        movementDirection = MovementDirection::NONE;
        switch(count){
            case 0:
                movementDirection = MovementDirection::NORTH;
                break;
            case 1:
                movementDirection = MovementDirection::WEST;
                break;
            case 2:
                movementDirection = MovementDirection::SOUTH;
                break;
            case 3:
                movementDirection = MovementDirection::EAST;
                break;

            default:
                movementDirection = MovementDirection::NORTH;
                count = count - 4;
                break;
        }


    
        //updateDirectionFromSerial();

    }
    //Serial.printf("yaw: %f, ang vel: %f\n", (yaw-targetYaw) * 180 / M_PI, angVel * 180 / M_PI);

    //Serial.printf("X: %f, Y: %f\n", x,y);
    if (movementDirection != MovementDirection::STOP) {
        baseMovement(fw_vel, lateral_vel);
        tofFeedback(fw_vel, lateral_vel);
        imuFeedback(turn_vel);
    }

    //Serial.printf("fw: %f, lat: %f, turn: %f\n", fw_vel, lateral_vel, turn_vel);
    //Serial.printf("direction: %d\n", static_cast<int>(movementDirection));

    motorsSetVelocity(fw_vel, lateral_vel, turn_vel);
    // Serial.printf("4!!! time: %d\n", micros() - start);

    motorsUpdate();
    
    // Serial.printf("5!!! time: %d\n", micros() - start);

    delay(10);
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
                movementDirection = MovementDirection::STOP; 
                break;
            default:
                movementDirection = MovementDirection::STOP;
                break;
        }

        lastSerialUpdate = millis();
    }

    if (millis() - lastSerialUpdate > 1000) {
        movementDirection = MovementDirection::STOP;
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

    // Serial.printf("front: %f, right: %f, rear: %f, left: %f\n", front, right, rear, left);
    
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

double tofVelocity_calc(double tof, double avgDist, double maxDist, double p) {
    // double dist = min(tof, avgDist);
    // if(tof < maxDist && tof > avgDist){
    //     dist = tof;
    // }
    double dist = tof;
    if (dist > maxDist) {
        dist = avgDist;
    }

    return p*dist;
}

void tofFeedback(double& fw_vel, double& lateral_vel) {
    double front = tofGetFrontIn();
    double right = tofGetRightIn();
    double rear = tofGetRearIn();
    double left = tofGetLeftIn();


    // Serial.printf("front: %f, right: %f, rear: %f, left: %f\n", front, right, rear, left);



    
    if(movementDirection == MovementDirection::EAST || movementDirection == MovementDirection::WEST){
        fw_vel += tofVelocity_calc(front, avgDist, maxDist, pTOF);
        fw_vel -= tofVelocity_calc(rear, avgDist, maxDist, pTOF);
    }

    if(movementDirection == MovementDirection::NORTH || movementDirection == MovementDirection::SOUTH){
        lateral_vel += tofVelocity_calc(left, avgDist, maxDist, pTOF);
        lateral_vel -= tofVelocity_calc(right, avgDist, maxDist, pTOF);
    }
}

void imuFeedback(double& turn_vel) {
    double yaw = imuGetYaw();

    double yawError = fmod(fmod(targetYaw - yaw, 2 * M_PI) + 3 * M_PI, 2 * M_PI) - M_PI;

    //Serial.printf("yaw: %f, ang vel: %f\n", (yaw - targetYaw) * 180 / M_PI, angVel * 180 / M_PI);
    turn_vel = kPRot * yawError;
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
const double sn = 1/(2*pow(2,.5));

void incOdom(double& x, double& y, double& theta, double blEnc, double brEnc, double flEnc, double frEnc){
    
    double front = tofGetFrontIn();
    double right = tofGetRightIn();
    double rear = tofGetRearIn();
    double left = tofGetLeftIn();

    double threshold = 2;

    double x_new = 0, y_new = 0;
    x_new += -sn*(flEnc + frEnc - brEnc - blEnc);
    y_new += sn*(-flEnc + frEnc + brEnc - blEnc);
 
    //double x_plus = x_new*cos(theta) - y_new*sin(theta);
    //double y_plus = x_new*sin(theta) + y_new*cos(theta);
    double x_plus = x_new;
    double y_plus = y_new;
    
    x += x_plus;
    y += y_plus;

    // x+= x_new;
    // y+= y_new;

    
    switch (movementDirection){
        case MovementDirection::NORTH:
            if (y_in_cell == 0 && (left > maxDistUpdate || right > maxDistUpdate)){
                y_in_cell += y_plus;
            }
            else{
                if(left <= maxDistUpdate && right <= maxDistUpdate){
                    y_in_cell = 0;
                    hasReset = true;
                }
                else
                    y_in_cell += y_plus;
            }
            break;
        case MovementDirection::SOUTH:
            if (y_in_cell == 7 && (left > maxDistUpdate || right > maxDistUpdate)){
                y_in_cell += y_plus;
            }
            else{
                if(left <= maxDistUpdate && right <= maxDistUpdate){
                    y_in_cell = 7;
                    hasReset = true;
                }
                else
                    y_in_cell += y_plus;
            }
            break;
        case MovementDirection::EAST:
            if (x_in_cell == 0 && (front > maxDistUpdate || rear > maxDistUpdate)){
                x_in_cell += x_plus;
            }
            else{
                if(front <= maxDistUpdate && rear <= maxDistUpdate){
                    x_in_cell = 0;
                    hasReset = true;
                }
                else
                    x_in_cell += x_plus;
            }
            break;
        case MovementDirection::WEST:
            if (x_in_cell == 7 && (front > maxDistUpdate || rear > maxDistUpdate)){
                x_in_cell += x_plus;
            }
            else{
                if(front <= maxDistUpdate && rear <= maxDistUpdate){
                    x_in_cell = 7;
                    hasReset = true;
                }
                else
                    x_in_cell += x_plus;
            }
            break;
        default:
            break;
    }
    if(movementDirection != MovementDirection::NONE){
   // Serial.printf("x: %f, y: %f, moveDir: %f \n", x, y, movementDirection );
    Serial.printf("xincel: %f, yincel: %f\n", x_in_cell, y_in_cell);

    Serial.printf("front %f, right %f, rear %f, left %f\n", front,right,rear,left);
    }
}


bool isCentered(double& xc, double& yc){
    switch(movementDirection){
        case MovementDirection::NORTH:
            return (yc > minCentered && yc < maxCentered);
        case MovementDirection::SOUTH:
            return (yc > minCentered && yc < maxCentered);
        case MovementDirection::EAST:
            return (xc > minCentered && xc < maxCentered);
        case MovementDirection::WEST:
            return (xc > minCentered && xc < maxCentered);

        default:
            return false;
    }

}