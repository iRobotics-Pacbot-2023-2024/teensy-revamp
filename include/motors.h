#pragma once

#include <TaskSchedulerDeclarations.h>

void motorsInit(Scheduler& ts);
void motorsSetVelocity(double fw_vel_in, double lateral_vel_in, double turn_vel_rad);
