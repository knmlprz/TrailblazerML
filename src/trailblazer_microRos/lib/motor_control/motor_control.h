#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>

// Pin Definitions
#define motor_front_right 18
#define motor_front_left 4
#define motor_back_right 19
#define motor_back_left 16

#define motor_front_right_dir 25
#define motor_front_left_dir 27
#define motor_back_right_dir 33
#define motor_back_left_dir 26

// Channels
#define pwm_channel_front_right 0
#define pwm_channel_front_left 1
#define pwm_channel_back_right 2
#define pwm_channel_back_left 3

// Functions
void setMotorSpeeds(int v_l, float v_r);
void motor_setup();
std::pair<int,int> error_motor_drive(int error);

#endif
