#pragma once

#include <stddef.h>
#include <stdint.h>

// todo These joint index must be handled with care !!!!!!
// 12dof
#define RIGHT_SHOULDER_JOINT_INDEX 0
#define RIGHT_HIP_X_JOINT_INDEX 1
#define RIGHT_HIP_Y_JOINT_INDEX 2
#define RIGHT_LEG_KNEE_JOINT_INDEX 3
#define RIGHT_LEG_ANKLE_Y_JOINT_INDEX 4
#define RIGHT_LEG_ANKLE_X_JOINT_INDEX 5

#define LEFT_SHOULDER_JOINT_INDEX 6
#define LEFT_HIP_X_JOINT_INDEX 7
#define LEFT_HIP_Y_JOINT_INDEX 8
#define LEFT_LEG_KNEE_JOINT_INDEX 9
#define LEFT_LEG_ANKLE_Y_JOINT_INDEX 10
#define LEFT_LEG_ANKLE_X_JOINT_INDEX 11

// 10dof
//#define RIGHT_LEG_KNEE_JOINT_INDEX 3
//#define LEFT_LEG_KNEE_JOINT_INDEX 8
//#define RIGHT_LEG_ANKLE_Y_JOINT_INDEX 4
//#define LEFT_LEG_ANKLE_Y_JOINT_INDEX 9

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