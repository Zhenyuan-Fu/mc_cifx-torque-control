#include "CalculateProg.h"

#include "IOAPPzm.h"
#include "cifXUser.h"

#include <math.h>
#include <string.h>

// todo These joint index must be handled with care !!!!!!
// 12dof
#define RIGHT_LEG_KNEE_JOINT_INDEX 3
#define LEFT_LEG_KNEE_JOINT_INDEX 9
#define RIGHT_LEG_ANKLE_Y_JOINT_INDEX 4
#define LEFT_LEG_ANKLE_Y_JOINT_INDEX 10
// 10dof
//#define RIGHT_LEG_KNEE_JOINT_INDEX 3
//#define LEFT_LEG_KNEE_JOINT_INDEX 8
//#define RIGHT_LEG_ANKLE_Y_JOINT_INDEX 4
//#define LEFT_LEG_ANKLE_Y_JOINT_INDEX 9

#define D2R M_PI/180.0
#define R2D 180.0/M_PI

double r_k = 0.054, S0_k = 0.302, Q_k = 10.0;
double r_a =0.04, S0_a = 0.29, Q_a = 5.0;

int32_t KneeJoint2MotorPosition(double pos)
{
  return round(sqrt(2.0)*sqrt(cos(pos + 3.141592653589793*(1.1E1 / 1.2E1))*-4.077E3 + 1.1765E4)*5.24288E4 - 9.287237632E6);
}

int32_t AnkleYJoint2MotorPosition(double pos)
{
  return round(sqrt(cos(pos + 3.141592653589793*(1.3E1 / 3.6E1))*-2.32E2 + 8.57E2)*5.24288E5 - 1.4443610112E7);
}

int32_t Joint2Mot(double pos, size_t id)
{
  if(id == RIGHT_LEG_KNEE_JOINT_INDEX || id == LEFT_LEG_KNEE_JOINT_INDEX)
  {
    return KneeJoint2MotorPosition(pos);
  }
  if(id == RIGHT_LEG_ANKLE_Y_JOINT_INDEX || id == LEFT_LEG_ANKLE_Y_JOINT_INDEX)
  {
    return AnkleYJoint2MotorPosition(pos);
  }
  if(nDirection_Pos[id] == 1)
  {
    return (int32_t)floor(nGear[id] * nEncoder[id] * pos / (2 * M_PI));
  }
  else
  {
    return (int32_t)ceil(-nGear[id] * nEncoder[id] * pos / (2 * M_PI));
  }
}

//void Tau2MotorCurrent_Athlete(void)
//{
//  float Kt_hip = 0.1182, Kt_knee = 0.1182, Kt_ankle = 0.0893;
//
//  leg_target_joint_current[RIGHT][0] = 100.0 * leg_target_joint_torque[RIGHT][0] / LEG_GEAR_RATE[RIGHT][0] / Kt_hip;  //mA
//  leg_target_joint_current[RIGHT][1] = 100.0 * leg_target_joint_torque[RIGHT][1] / LEG_GEAR_RATE[RIGHT][1] / Kt_hip;  //mA
//  leg_target_joint_current[RIGHT][2] = 100.0 * leg_target_joint_torque[RIGHT][2] / LEG_GEAR_RATE[RIGHT][2] / Kt_hip;  //mA
//  leg_target_joint_current[RIGHT][3] = 100.0 * leg_target_joint_torque[RIGHT][3] / (r_k*6282.7*(S0_k + r_k)*sin(PI + leg_actual_joint_angle[RIGHT][3] - 15.0*D2R)) * Q_k * (sqrt(2.0*S0_k*r_k - 2.0*r_k*r_k*cos(PI + leg_actual_joint_angle[RIGHT][3] - 15.0*D2R) + S0_k*S0_k + 2.0*r_k*r_k - 2.0*S0_k*r_k*cos(PI + leg_actual_joint_angle[RIGHT][3] - 15.0*D2R))) / Kt_knee;
//  leg_target_joint_current[RIGHT][4] = 100.0 * leg_target_joint_torque[RIGHT][4] / (r_a*6282.7*(S0_a + r_a)*sin(leg_actual_joint_angle[RIGHT][4] + 90.0*D2R)) * Q_a * (sqrt(2.0 * S0_a*r_a - 2.0 * r_a*r_a * cos(leg_actual_joint_angle[RIGHT][4] + 90.0*D2R) + S0_a*S0_a + 2 * r_a*r_a - 2.0 * S0_a*r_a*cos(leg_actual_joint_angle[RIGHT][4] + 90.0*D2R))) / Kt_ankle;
//  arm_target_joint_current[RIGHT][0] = 100.0 * arm_target_joint_torque[RIGHT][0] / ARM_GEAR_RATE[RIGHT][0] / Kt_hip;  //mA
//
//  leg_target_joint_current[LEFT][0] = 100.0 * leg_target_joint_torque[LEFT][0] / LEG_GEAR_RATE[LEFT][0] / Kt_hip;  //mA
//  leg_target_joint_current[LEFT][1] = 100.0 * leg_target_joint_torque[LEFT][1] / LEG_GEAR_RATE[LEFT][1] / Kt_hip;  //mA
//  leg_target_joint_current[LEFT][2] = 100.0 * leg_target_joint_torque[LEFT][2] / LEG_GEAR_RATE[LEFT][2] / Kt_hip;  //mA
//  leg_target_joint_current[LEFT][3] = 100.0 * leg_target_joint_torque[LEFT][3] / (r_k*6282.7*(S0_k + r_k)*sin(PI + leg_actual_joint_angle[LEFT][3] - 15.0*D2R)) * Q_k * (sqrt(2.0*S0_k*r_k - 2.0*r_k*r_k*cos(PI + leg_actual_joint_angle[LEFT][3] - 15.0*D2R) + S0_k*S0_k + 2.0*r_k*r_k - 2.0*S0_k*r_k*cos(PI + leg_actual_joint_angle[LEFT][3] - 15.0*D2R))) / Kt_knee;
//  leg_target_joint_current[LEFT][4] = 100.0 * leg_target_joint_torque[LEFT][4] / (r_a*6282.7*(S0_a + r_a)*sin(leg_actual_joint_angle[LEFT][4] + 90.0*D2R)) * Q_a * (sqrt(2.0 * S0_a*r_a - 2.0 * r_a*r_a * cos(leg_actual_joint_angle[LEFT][4] + 90.0*D2R) + S0_a*S0_a + 2 * r_a*r_a - 2.0 * S0_a*r_a*cos(leg_actual_joint_angle[LEFT][4] + 90.0*D2R))) / Kt_ankle;
//  arm_target_joint_current[LEFT][0] = 100.0 * arm_target_joint_torque[LEFT][0] / ARM_GEAR_RATE[LEFT][0] / Kt_hip;  //mA
//}

int16_t KneeJoint2MotorTorque(double tau)
{
  float Kt_knee = 0.1182;
  return (int16_t)(100.0 * tau / (r_k*6282.7*(S0_k + r_k)*sin(M_PI + tau - 15.0*D2R)) * Q_k * (sqrt(2.0*S0_k*r_k - 2.0*r_k*r_k*cos(M_PI + tau - 15.0*D2R) + S0_k*S0_k + 2.0*r_k*r_k - 2.0*S0_k*r_k*cos(M_PI + tau - 15.0*D2R))) / Kt_knee);
}

//int16_t KneeJoint2MotorTorque(double tau)
//{
//  float Kt_knee = 0.1182;
//  return (int16_t)(100.0 * tau / (0.054 * 6282.7 * (0.3 + 0.054) * sin(3.1415 + tau - 15.0 / 180.0 * 3.1415)) * 10
//                   * (sqrt(2.0 * 0.3 * 0.054 - 2.0 * 0.054 * 0.054 * cos(3.1415 + tau - 15.0 / 180.0 * 3.1415)
//                           + 0.3 * 0.3 + 2.0 * 0.054 * 0.054 -
//                           2.0 * 0.3 * 0.054 * cos(3.1415 + tau - 15.0 / 180.0 * 3.1415))) / Kt_knee);
//}
int16_t AnkleYJoint2MotorTorque(double tau)
{
  float Kt_ankle = 0.0893;
  return (int16_t)(100.0 * tau / (r_a*6282.7*(S0_a + r_a)*sin(tau + 90.0*D2R)) * Q_a * (sqrt(2.0 * S0_a*r_a - 2.0 * r_a*r_a * cos(tau + 90.0*D2R) + S0_a*S0_a + 2 * r_a*r_a - 2.0 * S0_a*r_a*cos(tau + 90.0*D2R))) / Kt_ankle);
}
//int16_t AnkleYJoint2MotorTorque(double tau)
//{
//  float Kt_ankle = 0.0893;
//  return (int16_t)(100.0 * tau / (0.04*6282.7*(0.29 + 0.04)*sin(tau + 90.0 / 180.0 * 3.1415)) * 5.0 *
//                       (sqrt(2.0 * 0.29*0.04 - 2.0 * 0.04*0.04 * cos(tau + 90.0 / 180.0 * 3.1415) +
//                             0.29*0.29 + 2.0 * 0.04*0.04 -
//                         2.0 * 0.29*0.04*cos(tau + 90.0 / 180.0 * 3.1415))) / Kt_ankle);
//}
int16_t JointTorque2MotorCurrent(double tau, size_t id)
{
  float Kt_hip = 0.1182;
  // Knee: jointTorque -->> motorCurrent
  if(id == RIGHT_LEG_KNEE_JOINT_INDEX || id == LEFT_LEG_KNEE_JOINT_INDEX){
    return KneeJoint2MotorTorque(tau);
  }
  // Ankle_y: jointTorque -->> motorCurrent
  if(id == RIGHT_LEG_ANKLE_Y_JOINT_INDEX || id == LEFT_LEG_ANKLE_Y_JOINT_INDEX){
    return AnkleYJoint2MotorTorque(tau);
  }
  // Others(arms, hips, ankle_x): jointTorque -->> motorCurrent
  if(nDirection_Pos[id] == 1){
    return (100.0 * tau / nGear[id] / Kt_hip);// mA
  }
}

double Mot2Joint(int32_t motor, size_t id)
{
  if(nDirection_Pos[id] == 1)
  {
    return 2.0 * M_PI * motor / ((double)(nGear[id]) * (double)(nEncoder[id]));
  }
  else
  {
    return -2.0 * M_PI * motor / ((double)(nGear[id]) * (double)(nEncoder[id]));
  }
}

double KneeMot2JointPosition(int32_t motor)
{
  return 3.141592653589793*(-1.1E1 / 1.2E1) + acos((pow(((double)(motor) + 9.287237632E6) / 5.24288E4, 2.0)*0.5 - 1.1765E4) / (-4.077E3));
}

double AnkleYMot2JointPosition(int32_t motor)
{
 return 3.141592653589793*(-1.0 / 2.0) + acos(pow((double)(motor) *1.9073486328125E-5 + 2.7549E2, 2.0)*(-4.310344827586207E-5) + 8.57E2 / 2.32E2) + 25.0 / 180.0*3.141592653589793;
}

double Mot2JointPosition(int32_t motor, size_t id)
{
  if(id == RIGHT_LEG_KNEE_JOINT_INDEX || id == LEFT_LEG_KNEE_JOINT_INDEX)
  {
    return KneeMot2JointPosition(motor);
  }
  if(id == RIGHT_LEG_ANKLE_Y_JOINT_INDEX || id == LEFT_LEG_ANKLE_Y_JOINT_INDEX)
  {
    return AnkleYMot2JointPosition(motor);
  }
  return Mot2Joint(motor, id);
}

double KneeMot2JointVelocity(int32_t motor, double angle)
{
 return ((double)(motor) / pow(2, 18) * 60.0) * 10.0 / 60000.0 / ((0.3 + 0.054) / (sqrt(2.0 * 0.3*0.054 - 2 * 0.054*0.054 * cos(3.1415 + angle - 15.0 / 180.0 * 3.1415) + 0.3*0.3 + 2.0 * 0.054*0.054 - 2.0 * 0.3*0.054*cos(3.1415 + angle - 15.0 / 180.0 * 3.1415)))*sin(3.1415 + angle - 15.0 / 180.0 * 3.1415)) / 0.054;
}

double AnkleYMot2JointVelocity(int32_t motor, double angle)
{
  return ((double)(motor) / pow(2, 18) * 60.0) * 5.0 / 60000.0 / ((0.29 + 0.04) / (sqrt(2.0 * 0.29*0.04 - 2 * 0.04*0.04 * cos(angle + 90.0 / 180.0 * 3.1415) + 0.29*0.29 + 2.0 * 0.04*0.04 - 2.0 * 0.29*0.04*cos(angle + 90.0 / 180.0 * 3.1415)))*sin(angle + 90.0 / 180.0 * 3.1415)) / 0.04;
}

double Mot2JointVelocity(int32_t motor, double angle, size_t id)
{
  if(id == RIGHT_LEG_KNEE_JOINT_INDEX || id == LEFT_LEG_KNEE_JOINT_INDEX)
  {
    return KneeMot2JointVelocity(motor, angle);
  }
  if(id == RIGHT_LEG_ANKLE_Y_JOINT_INDEX || id == LEFT_LEG_ANKLE_Y_JOINT_INDEX)
  {
    return AnkleYMot2JointVelocity(motor, angle);
  }
  return Mot2Joint(motor, id);
}

float convert(uint32_t i)
{
  _Static_assert(sizeof(float) == sizeof(int32_t), "weird platform");
  float out;
  memcpy(&out, &i, sizeof(i));
  return out;
}
