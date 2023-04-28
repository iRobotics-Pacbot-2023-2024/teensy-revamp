#include <vl53l4cx_class.h>

VL53L4CX tofs[] {{&Wire, 16}, {&Wire, 15}, {&Wire, 14}, {&Wire, 13}};

void tofInit() {
    for (int i = 0; i < 4; i++) {
        auto& tof = tofs[i];
        int status = tof.begin();
        if (status != 0) {
            while (true) {
                Serial.printf("tof %d: begin: %d\n", i, status);
                delay(1000);
            }
        }
        status = tof.InitSensor(0x12 + 2 * i);
        if (status != 0) {
            while (true) {
                Serial.printf("tof %d: init sensor: %d\n", i, status);
                delay(1000);
            }
        }
        if (status != 0) {
            while (true) {
                Serial.printf("tof %d: timing budget: %d\n", i, status);
            }
        }
        status = tof.VL53L4CX_StartMeasurement();
        if (status != 0) {
            while (true) {
                Serial.printf("tof %d: start: %d\n", i, status);
                delay(1000);
            }
        }
        status = tof.VL53L4CX_ClearInterruptAndStartMeasurement();
        if (status != 0) {
            while (true) {
                Serial.printf("tof %d: clear: %d\n", i, status);
                delay(1000);
            }
        }

        // the number of microseconds that the sensor will take for each measurement
        status = tof.VL53L4CX_SetMeasurementTimingBudgetMicroSeconds(8000);
        if(status != 0){
            while (true){
                Serial.printf("tof %d: clear: %d\n", i, status);
                delay(1000);
            }
        }
        
        // the maximum distance that the sensor can read, not sure what units
        status = tof.VL53L4CX_SetTuningParameter(VL53L4CX_TUNINGPARM_RESET_MERGE_THRESHOLD, 5000);
        if(status != 0){
            while (true){
                Serial.printf("tof %d: clear: %d\n", i, status);
                delay(1000);
            }
        }
        
    }
}

uint16_t tofDistance[4] = {0};

void tofUpdateReadings() {
    for (int i = 0; i < 4; i++) {
        auto& tof = tofs[i];

        uint8_t dataReady = 0;
        int status = tof.VL53L4CX_GetMeasurementDataReady(&dataReady);
        if (status != 0) {
            Serial.printf("tof %d: data ready: %d\n", i, status);
            continue;
        }

        if (!dataReady) {
            continue;
        }

        VL53L4CX_MultiRangingData_t results;
        status = tof.VL53L4CX_GetMultiRangingData(&results);
        if (status != 0) {
            Serial.printf("tof %d: get data: %d\n", i, status);
            continue;
        }

        // uint16_t min_distance = 0xffff;
        // for (int j = 0; j < results.NumberOfObjectsFound; j++) {
        //     int16_t range = results.RangeData[j].RangeMilliMeter;
        //     if (range > 0 && range < min_distance) {
        //         min_distance = results.RangeData[j].RangeMilliMeter;
        //     }
        // }
        // tofDistance[i] = min_distance;

        tofDistance[i] = results.RangeData->RangeMinMilliMeter;

        if (status == 0) {
            status = tof.VL53L4CX_ClearInterruptAndStartMeasurement();
            if (status != 0) {
                Serial.printf("tof %d: clear: %d\n", i, status);
            }
        }
    }
}

constexpr double mmPerIn = 25.4;

double tofGetFrontIn() {
    Serial.printf("tofGetFrontIn: %d\n", tofDistance[2]);
    return tofDistance[2] / mmPerIn;
}

double tofGetRightIn() {
    Serial.printf("tofGetRightIn: %d\n", tofDistance[0]);
    return tofDistance[0] / mmPerIn;
}

double tofGetRearIn() {
    Serial.printf("tofGetRearIn: %d\n", tofDistance[1]);
    return tofDistance[1] / mmPerIn;
}

double tofGetLeftIn() {
    Serial.printf("tofGetLeftIn: %d\n", tofDistance[3]);
    return tofDistance[3] / mmPerIn;
}
