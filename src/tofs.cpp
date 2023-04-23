#include <vl53l4cx_class.h>

VL53L4CX tofs[] {{&Wire, 16}, {&Wire, 15}, {&Wire, 14}, {&Wire, 13}};

void tofInit() {
    for (int i = 0; i < 4; i++) {
        auto& tof = tofs[i];
        int status = tof.begin();
        Serial.printf("tof %d: begin: %d\n", status);
        status = tof.InitSensor(0x12 + 2 * i);
        Serial.printf("tof %d: init: %d\n", status);
        status = tof.VL53L4CX_StartMeasurement();
        Serial.printf("tof %d: start: %d\n", status);
        status = tof.VL53L4CX_ClearInterruptAndStartMeasurement();
        Serial.printf("tof %d: clear: %d\n", status);
    }
}

void tofUpdateReadings() {
    for (int i = 0; i < 4; i++) {
        auto& tof = tofs[i];

        uint8_t dataReady = 0;
        int status = tof.VL53L4CX_GetMeasurementDataReady(&dataReady);
        Serial.printf("tof %d: data ready: %d\n", i, status);

        if (dataReady) {
            VL53L4CX_MultiRangingData_t results;
            status = tof.VL53L4CX_GetMultiRangingData(&results);
            Serial.printf("tof %d: get data: %d\n", i, status);

            Serial.printf("tof %d: n: %d \n", i, results.NumberOfObjectsFound);
        }
    }
}