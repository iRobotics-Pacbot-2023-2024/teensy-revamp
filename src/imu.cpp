#include <Adafruit_BNO08x.h>
#include <Wire.h>

#include "imu.h"

struct euler_t {
  double yaw;
  double pitch;
  double roll;
};

euler_t quaternionToEuler(double qr, double qi, double qj, double qk) {
    double sqr = sq(qr);
    double sqi = sq(qi);
    double sqj = sq(qj);
    double sqk = sq(qk);

    double yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
    double pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
    double roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

    return {yaw, pitch, roll};
}

Adafruit_BNO08x imu{};

void setReports();

void imuInit() {
    if (!imu.begin_I2C(74, &Wire)) {
        while (true) {
            Serial.println("Failed to start IMU");
            delay(100);
        }
    }

    setReports();
}

void setReports() {
    if (!imu.enableReport(SH2_ROTATION_VECTOR, 1000)) {
        while (true) {
            Serial.println("Could not enable rotation vector");
            delay(100);
        }
    }

    if (!imu.enableReport(SH2_GYROSCOPE_CALIBRATED, 1000)) {
        while (true) {
            Serial.println("Could not enable calibrated gyro");
            delay(109);
        }
    }
}

bool hasRotation = false;
sh2_RotationVectorWAcc rotation;

bool hasAngVel = false;
sh2_Gyroscope_t gyro;

bool imuUpdateReadings() {
    uint32_t start = micros();
    if (imu.wasReset()) {
        setReports();
    }
    // Serial.printf("a%d\n", micros() - start);
    sh2_SensorValue_t reading;
    if (imu.getSensorEvent(&reading)) {
        // Serial.printf("a%d\n", micros() - start);
        // Serial.printf("WACK: %hhd\n", reading.sensorId);
        switch(reading.sensorId) {
            case SH2_ROTATION_VECTOR:
                hasRotation = true;
                rotation = reading.un.rotationVector;
                break;
            case SH2_GYROSCOPE_CALIBRATED:
                hasAngVel = true;
                gyro = reading.un.gyroscope;
                break;
            default:
                break;
        }
    }

    return hasRotation && hasAngVel;
}

double imuGetYaw() {
    float qr = rotation.real;
    float qi = rotation.i;
    float qj = rotation.j;
    float qk = rotation.k;
    euler_t euler = quaternionToEuler(qr, qi, qj, qk);
    return euler.yaw;
}

double imuGetAngVel() {
    return gyro.z;
}