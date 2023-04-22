#include <TaskSchedulerDeclarations.h>

#include "Motor.h"
#include "MotorController.h"

void updateMotors();

Task motorUpdateTask(MOTOR_UPDATE_INTERVAL_MS * TASK_MILLISECOND, TASK_FOREVER, &updateMotors);

// 
Motor motors[] {{23, 12}, {11, 10}, };
Encoder enc(22, 17);
MotorController controller(motors[0], enc);

void initMotors(Scheduler& ts) {
    ts.addTask(motorUpdateTask);
    motorUpdateTask.enable();
}

void updateMotors() {
    controller.update();
}