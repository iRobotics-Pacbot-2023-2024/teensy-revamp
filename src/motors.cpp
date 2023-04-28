#include "motors.h"

#include "Motor.h"
#include "MotorController.h"

// 0 = rear left, 1 = rear right, 2 = front left, 3 = front right 
Motor motors[] {{6, 5}, {9, 8}, {11, 10}, {23, 12}};
Encoder encoders[] {{0, 1}, {2, 3}, {4, 7}, {22, 17}};
MotorController controllers[] {{motors[0], encoders[0]}, {motors[1], encoders[1]}, {motors[2], encoders[2]}, {motors[3], encoders[3]}};

void motorsUpdate() {
    for (MotorController& controller : controllers) {
        controller.update();
    }
}


constexpr double MAX_TICKS_PER_SEC = 8000;

constexpr double TICKS_PER_REV = 357.7 * 4;
constexpr double WHEEL_DIAMETER = 38 / 25.4;
constexpr double IN_PER_REV = M_PI * WHEEL_DIAMETER;
constexpr double TICKS_PER_IN = TICKS_PER_REV / IN_PER_REV;

constexpr double TRACK_RADIUS = 2.2;

void getEncodersValues(double *encoder_values) { //return int array of encoder values
    int i =0 ;
    Serial.print("Encoder values: ");
    for (MotorController& controller : controllers) {
        encoder_values[i] = controller.readEncoder()/TICKS_PER_IN;
        Serial.printf("%f ", encoder_values[i]);
        i++;
    }
    Serial.println("");
}

void motorsSetVelocity(double fw_vel_in, double lateral_vel_in, double turn_vel_rad) {
    double fw_vel = fw_vel_in * TICKS_PER_IN / M_SQRT2;
    double lateral_vel = lateral_vel_in * TICKS_PER_IN / M_SQRT2;
    double turn_vel = turn_vel_rad * TICKS_PER_IN * TRACK_RADIUS;

    double max_vel = abs(turn_vel) + abs(fw_vel) + abs(lateral_vel);
    if (max_vel > MAX_TICKS_PER_SEC) {
        turn_vel *= MAX_TICKS_PER_SEC / max_vel;
        fw_vel *= MAX_TICKS_PER_SEC / max_vel;
        lateral_vel *= MAX_TICKS_PER_SEC / max_vel;
    }

    double fl_vel = fw_vel - lateral_vel - turn_vel;
    double fr_vel = -fw_vel - lateral_vel - turn_vel;
    double rl_vel = fw_vel + lateral_vel - turn_vel;
    double rr_vel = -fw_vel + lateral_vel - turn_vel;

    controllers[0].setTarget(-rl_vel);
    controllers[1].setTarget(-rr_vel);
    controllers[2].setTarget(-fl_vel);
    controllers[3].setTarget(-fr_vel);
}