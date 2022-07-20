#pragma once

#include <stddef.h>
#include <stdint.h>

// Convert radian command to motor position
int32_t Joint2Mot(double pos, size_t id);

int16_t JointTorque2MotorCurrent(double tau, size_t id);

// Returns an encoder reading in radian
double Mot2JointPosition(int32_t motor, size_t id);

// Returns an encoder velocity reading in radian/s
double Mot2JointVelocity(int32_t motor_velocity, double joint_angle, size_t id);

// Binary conversion from a uint32_t to a float
float convert(uint32_t i);

//convertion from degree to radian
float deg2rad(float deg);