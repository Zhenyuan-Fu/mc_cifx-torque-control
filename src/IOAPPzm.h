#include "cifXUser.h"

#include <stddef.h>

// 电机发送参数
struct Motor_S
{
  uint16_t Control_Word;
  uint16_t Mode; // 01-P/3-V/4-T/08-SP/09-SV/0A-ST
  int16_t Torque;
  int32_t Position;
  int32_t ProfileVelocity;
  int32_t EndVelocity;
  int32_t Acceleration;
  int32_t Deceleration;
  int32_t TargetVelocity;
};

// 电机接收参数
struct Motor_R
{
  uint16_t Status_Word;
  uint16_t ModeDis; // 01-P/3-V/4-T/08-SP/09-SV/0A-ST
  int32_t Position;
  int32_t Velocity;
  int16_t Torque;
  int16_t Current;
};
// 未定义imu相关参数

struct ForceSensor
{
  uint16_t status;
  uint32_t FX;
  uint32_t FY;
  uint32_t FZ;
  uint32_t TX;
  uint32_t TY;
  uint32_t TZ;
};

#define MOT_ID 12 // !! number of motors 4*2+2
#define NO_S_BYTE 30
#define NO_R_BYTE 16

#define IMU_COUNT 1
#define IMU_StartAddress MOT_ID * NO_R_BYTE
#define IMU_BYTE 36

#define ForceSensor_COUNT 2
#define ForceSensorStartAddress (MOT_ID * NO_R_BYTE + IMU_COUNT * IMU_BYTE)
#define ForceSensor_BYTE (6 * 4 + 2)

// We must send 6 bytes to the force sensor (reserved for future use)
#define SendTypeNum MOT_ID * NO_S_BYTE + IMU_COUNT * 2 + ForceSensor_COUNT * 6
#define ReciveTypeNum MOT_ID * NO_R_BYTE + IMU_COUNT * IMU_BYTE + ForceSensor_COUNT * ForceSensor_BYTE

#define ProPositionMode 1
#define ProVelocityMode 3
#define ProTorqueMode 4
#define InterpolatedMode 7
#define SynPositionMode 8
#define SynVelocityMode 9
#define SynTorqueMode 10

#define DisableVoltage 0x00
#define SwithOff 0x06
#define SwithOnVoltageEn 0x07
#define EnOperation 0x0f

/** Output process data ******/
#define CW_SWITCH_ON 0x0001
#define CW_ENABLE_VOLTAGE 0x0002
#define CW_QUICK_STOP 0x0004
#define CW_ENABLE_OPERATION 0x0008
#define CW_RESET_FAULT 0x0080
#define CW_MANDONTARY_MASK 0x008F
#define CW_Relative 0x0040 // 0:�����˶���1������˶�
#define CW_New_Set_PP 0x0010 // new set_point
#define CW_New_Clr_PP 0xFF7f // new set_point clr

/* Device control commands */
#define SHUTDOWN(cw) (CW_QUICK_STOP | CW_ENABLE_VOLTAGE /*| ((cw) | (CW_ENABLE_OPERATION))*/) // 6
#define SWITCH_ON(cw) (CW_QUICK_STOP | CW_ENABLE_VOLTAGE | CW_SWITCH_ON) // 7
#define DIASABLE_VOLTAGE(cw) ((cw) | (CW_ENABLE_OPERATION | CW_QUICK_STOP | CW_SWITCH_ON))
#define QUICK_STOP(cw) (CW_ENABLE_VOLTAGE | ((cw) | (CW_ENABLE_OPERATION | CW_SWITCH_ON)))
#define DISABLE_OPERATIONAL(cw) (CW_QUICK_STOP | CW_ENABLE_VOLTAGE | CW_SWITCH_ON) // 7
#define ENABLE_OPERATIONAL(cw) (CW_ENABLE_OPERATION | CW_QUICK_STOP | CW_ENABLE_VOLTAGE | CW_SWITCH_ON)
#define FAULT_RESET(cw) \
  (CW_RESET_FAULT | ((cw) | (CW_ENABLE_OPERATION | CW_QUICK_STOP | CW_ENABLE_VOLTAGE | CW_SWITCH_ON)))
#define ENABLE_OPERATION_Abs_PP(cw) \
  (CW_ENABLE_OPERATION | CW_QUICK_STOP | CW_ENABLE_VOLTAGE | CW_SWITCH_ON | CW_New_Set_PP)
#define ENABLE_OPERATION_Rel_PP(cw) \
  (CW_ENABLE_OPERATION | CW_QUICK_STOP | CW_ENABLE_VOLTAGE | CW_SWITCH_ON | CW_Relative | CW_New_Set_PP)
#define DisENABLE_OPERATION_Abs_PP(cw) \
  (CW_ENABLE_OPERATION | CW_QUICK_STOP | CW_ENABLE_VOLTAGE | CW_SWITCH_ON & CW_New_Clr_PP)

/** Input process data ***************************************************/
#define SW_READY_TO_SWITCH_ON 0x0001
#define SW_SWITCH_ON 0x0002
#define SW_OPERATION_ENABLE 0x0004
#define SW_FAULT 0x0008
#define SW_VOLTAGE_DISABLED 0x0010
#define SW_QUICK_STOP 0x0020
#define SW_SWITCH_ON_DISABLED 0x0040
#define SW_REMOTE 0x0200
#define SW_TARGET_REACHED 0x0400
#define SW_INTERANL_LIMIT_ACTIVE 0x0800

#define SW_MANDONTARY_MASK 0x0E7F

/* The following macros can be used to indicate the status of the device */
#define NOT_READY_TO_SWITCH_ON(sw) (((sw)&0x004F) == 0x0000)
#define SWITCH_ON_DISABLED(sw) (((sw)&0x004F) == 0x0040)
#define READY_TO_SWITCH_ON(sw) (((sw)&0x006F) == 0x0021)
#define SWITCHED_ON(sw) (((sw)&0x006F) == 0x0023)
#define ONPERATION_ENABLED(sw) (((sw)&0x006F) == 0x0027)
#define QUICK_STOP_ACTIVE(sw) (((sw)&0x006F) == 0x0007)
#define FAULT_REACTION_ACTIVE(sw) (((sw)&0x004F) == 0x000F)
#define FAULT(sw) (((sw)&0x004F) == 0x0008)

#define PositionConSwithOff 0x56
#define PositionConVoltageEn 0x57
#define PositionConEnOperation 0x5f

#define PositionSigSwithOff 0x76
#define PositionSigVoltageEn 0x77
#define PositionSigEnOperation 0x7f

#define set_point_acknoledge 0x00001000

#define set_point_acknoledge 0x00001000
#define PowerOffStatusFlag 0x0001
#define VoltageStatusEnFlag 0x0003
#define PowerOnStatusFlag 0x0007

#define ControlWordRel 0
#define ControlWordNum 2
#define OperationModeRel 2
#define OperationModeNum 1
#define TargetTorqueRel 4
#define TargetTorqueTypeNum 2
#define TargetPositionRel 6
#define TargetPositionTypeNum 4

#define ProfileVelocityRel 10
#define ProfileVelocityTypeNum 4

#define EndVelocityRel 14
#define EndVelocityTypeNum 4
#define AccelerationRel 18
#define AccelerationTypeNum 4
#define DecelerationRel 22
#define DecelerationTypeNum 4
#define TargetVelocityRel 26
#define TargetVelocityTypeNum 4

#define StatusWordRel 0
#define StatusWordNum 2
#define OperationModeDisRel 2
#define OperationModeDisNum 1

#define ActualPositionRel 4
#define ActualPositionTypeNum 4
#define ActualVelocityRel 8
#define ActualVelocityTypeNum 4
#define ActualTorqueRel 12
#define ActualTorqueTypeNum 2
#define ActualCurrentRel 14
#define ActualCurrentTypeNum 2

void Set_Mode(uint32_t S_ID, uint16_t Mode);
void Set_Mode_PP(uint32_t MotorNum);
void Set_Mode_CSP(uint32_t MotorNum);
void Set_Mode_CST(uint32_t MotorNum);

void Set_Position(uint32_t S_ID, int32_t Target);
void Set_TargetVelocity(uint32_t S_ID, int32_t Target);
void Set_ProfileVelocity(uint32_t S_ID, int32_t Target);
void Set_Torque(uint32_t S_ID, int16_t Target);
void Set_Acceleration(uint32_t S_ID, int32_t aTemp);
void Set_Deceleration(uint32_t S_ID, int32_t aTemp);
void Set_EndVelocity(uint32_t S_ID, int32_t aTemp);
void Set_ControlWord(uint32_t S_ID, uint16_t ControlWordTemp);

void Set_Motor(uint32_t S_ID);

int32_t IO_WriteZM(CIFXHANDLE hChannel);
int32_t Get_Mot_Data(CIFXHANDLE hChannel);
void Power_OFF(CIFXHANDLE hChannel, uint32_t S_ID);
void Power_OFF_AllMoter(CIFXHANDLE hChannel, uint32_t MotorNumTemp); // ZM 2020.02.26
void Voltage_En(CIFXHANDLE hChannel, uint32_t S_ID);
void Switch_On(CIFXHANDLE hChannel, uint32_t S_ID);
void Switch_On_All(CIFXHANDLE hChannel, uint32_t Motor_NumTemp);
void Voltage_En(CIFXHANDLE hChannel, uint32_t S_ID);
void Voltage_En_All(CIFXHANDLE hChannel, uint32_t Motor_NumTemp);
void Enable_Operation(CIFXHANDLE hChannel, uint32_t S_ID);
void Servo_On(CIFXHANDLE hChannel, uint32_t S_ID);
void Servo_On_All(CIFXHANDLE hChannel, uint32_t Motor_NumTemp); // ZM 2020.04.2
void init_Motor(CIFXHANDLE hChannel);
void init_ForceSensor(CIFXHANDLE hChannel);

void FaultRes(CIFXHANDLE hChannel, uint32_t MotorNum);
void FaultResAll(CIFXHANDLE hChannel);

void ReadMulMotoMF(CIFXHANDLE hChannel);
void ReadMulMotoEC(CIFXHANDLE hChannel);

void ReadMulMotoEE(CIFXHANDLE hChannel);

long ECatMasterDev_SetSyncMode(CIFXHANDLE hChannel);

extern struct Motor_S MOT_Send[MOT_ID];
extern struct Motor_R MOT_Recive[MOT_ID];
extern struct ForceSensor FS_Recive[ForceSensor_COUNT];
extern struct ForceSensor InitFS_Recive[ForceSensor_COUNT];
extern double nGear[MOT_ID];
extern int32_t nEncoder[MOT_ID];
extern int16_t nDirection_Pos[MOT_ID];
