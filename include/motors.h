#pragma once
#include <cstdint>
void motorsUpdate();
void motorsSetVelocity(double fw_vel_in, double lateral_vel_in, double turn_vel_rad);
void getEncodersValues(int32_t *encoder_values);
