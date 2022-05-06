#include "IOAPPzm.h"
#include "SDOApp.h"
#include "cifXErrors.h"
#include "cifXUser.h"
#include "rcX_Public.h"
#include "rcX_User.h"

#include "CalculateProg.h"
#include "Display.h"

#include <stdio.h>
#include <string.h>
#include <unistd.h>

_Static_assert(sizeof(struct Motor_R) == NO_R_BYTE, "Wrong definition of NO_R_BYTE");
_Static_assert(offsetof(struct ForceSensor, status) == 0, "Weird padding of ForceSensor");
_Static_assert(sizeof(struct ForceSensor) - offsetof(struct ForceSensor, FX) == ForceSensor_BYTE - 2,
               "Weird padding of ForceSensor");

struct Motor_S MOT_Send[MOT_ID];
// params of motor when receiving data 电机接收参数
struct Motor_R MOT_Recive[MOT_ID];
// Force sensors
struct ForceSensor FS_Recive[ForceSensor_COUNT];
struct ForceSensor InitFS_Recive[ForceSensor_COUNT];

// !! Reduction ratio for each motor
double nGear[MOT_ID] = {17.43, 48.3, 25.0, 1.0, 1.0, 1.0,
                        17.43, 48.3, 25.0, 1.0, 1.0, 1.0};// shoulder, hip1, hip2, knee, ankle_y, ankle_x
// !! Encounter count for each motor
//int32_t nEncoder[MOT_ID] = {262144, 262144, 262144, 262144, 262144, 262144, 262144, 262144, 4096, 4096};
//int32_t nEncoder[MOT_ID] = {4000, 4000,4000, 4000, 262144, 262144, 262144, 262144, 4096, 4096};
int32_t nEncoder[MOT_ID] = {262144,262144,262144, 262144,262144,262144, 262144,262144,262144, 262144,262144,262144};
// !! Direction for each motor
//int16_t nDirection_Pos[MOT_ID] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
int16_t nDirection_Pos[MOT_ID] = {1,1,1, 1,1,1, 1,1,1, 1,1,1};
unsigned char abSendData[SendTypeNum];
unsigned char abRecvData[ReciveTypeNum];

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"

/*****************************************************************************/
// position setting of motors 设置每个电机的位置
// S_ID:电机序号；Target: 位置（4字节），TargetVelocityRel: 偏移量
/*****************************************************************************/
void Set_Position(uint32_t S_ID, int32_t Target)
{
  abSendData[S_ID * NO_S_BYTE + TargetPositionRel] = Target & 0x000000FF;
  abSendData[S_ID * NO_S_BYTE + TargetPositionRel + 1] = (Target >> 8) & 0x000000FF;
  abSendData[S_ID * NO_S_BYTE + TargetPositionRel + 2] = (Target >> 16) & 0x000000FF;
  abSendData[S_ID * NO_S_BYTE + TargetPositionRel + 3] = (Target >> 24) & 0x000000FF;
}

/*****************************************************************************/
// 设置每个电机的Target速度
// S_ID:电机序号；Target：速度（4字节），TargetVelocityRel：偏移量
/*****************************************************************************/
void Set_TargetVelocity(uint32_t S_ID, int32_t Target)
{
  abSendData[S_ID * NO_S_BYTE + TargetVelocityRel] = Target & 0x000000FF;
  abSendData[S_ID * NO_S_BYTE + TargetVelocityRel + 1] = (Target >> 8) & 0x000000FF;
  abSendData[S_ID * NO_S_BYTE + TargetVelocityRel + 2] = (Target >> 16) & 0x000000FF;
  abSendData[S_ID * NO_S_BYTE + TargetVelocityRel + 3] = (Target >> 24) & 0x000000FF;
}

/*****************************************************************************/
// 设置每个电机Profile速度
// S_ID:电机序号；Profile：速度（4字节），ProfileVelocityRel； 偏移量
/*****************************************************************************/
void Set_ProfileVelocity(uint32_t S_ID, int32_t Target)
{
  abSendData[S_ID * NO_S_BYTE + ProfileVelocityRel] = Target & 0x000000FF;
  abSendData[S_ID * NO_S_BYTE + ProfileVelocityRel + 1] = (Target >> 8) & 0x000000FF;
  abSendData[S_ID * NO_S_BYTE + ProfileVelocityRel + 2] = (Target >> 16) & 0x000000FF;
  abSendData[S_ID * NO_S_BYTE + ProfileVelocityRel + 3] = (Target >> 24) & 0x000000FF;
}

/*****************************************************************************/
// 设置每个电机torque力矩
// S_ID:电机序号；Target：力矩（4字节）
/*****************************************************************************/
void Set_Torque(uint32_t S_ID, int16_t Target)
{
  abSendData[S_ID * NO_S_BYTE + TargetTorqueRel] = Target & 0x000000FF;
  abSendData[S_ID * NO_S_BYTE + TargetTorqueRel + 1] = (Target >> 8) & 0x000000FF;
}
/*****************************************************************************/
// 设置每个电机的工作模式
// S_ID:电机序号；Mode：工作模式；
/*****************************************************************************/
void Set_Mode(uint32_t S_ID, uint16_t Mode)
{
  abSendData[S_ID * NO_S_BYTE + OperationModeRel] = Mode & 0x000000FF;
  abSendData[S_ID * NO_S_BYTE + OperationModeRel] = (Mode >> 8) & 0x000000FF;
}
/*****************************************************************************/
// 设置每个电机的加速度
// S_ID:电机序号；aTemp:加速度
/*****************************************************************************/
void Set_Acceleration(uint32_t S_ID, int32_t aTemp)
{
  abSendData[S_ID * NO_S_BYTE + AccelerationRel] = aTemp & 0x000000FF;
  abSendData[S_ID * NO_S_BYTE + AccelerationRel + 1] = (aTemp >> 8) & 0x000000FF;
  abSendData[S_ID * NO_S_BYTE + AccelerationRel + 2] = (aTemp >> 16) & 0x000000FF;
  abSendData[S_ID * NO_S_BYTE + AccelerationRel + 3] = (aTemp >> 24) & 0x000000FF;
}
/*****************************************************************************/
// 设置每个电机的减加速度
// S_ID:电机序号；aTemp:减加速度
/*****************************************************************************/
void Set_Deceleration(uint32_t S_ID, int32_t aTemp)
{
  abSendData[S_ID * NO_S_BYTE + DecelerationRel] = aTemp & 0x000000FF;
  abSendData[S_ID * NO_S_BYTE + DecelerationRel + 1] = (aTemp >> 8) & 0x000000FF;
  abSendData[S_ID * NO_S_BYTE + DecelerationRel + 2] = (aTemp >> 16) & 0x000000FF;
  abSendData[S_ID * NO_S_BYTE + DecelerationRel + 3] = (aTemp >> 24) & 0x000000FF;
}
/*****************************************************************************/
// 设置每个电机的结束速度
// S_ID:电机序号；结束速度: aTemp
/*****************************************************************************/
void Set_EndVelocity(uint32_t S_ID, int32_t aTemp)
{
  abSendData[S_ID * NO_S_BYTE + EndVelocityRel] = aTemp & 0x000000FF;
  abSendData[S_ID * NO_S_BYTE + EndVelocityRel + 1] = (aTemp >> 8) & 0x000000FF;
  abSendData[S_ID * NO_S_BYTE + EndVelocityRel + 2] = (aTemp >> 16) & 0x000000FF;
  abSendData[S_ID * NO_S_BYTE + EndVelocityRel + 3] = (aTemp >> 24) & 0x000000FF;
}
///*****************************************************************************/
///* 设置控制字
///*
///*****************************************************************************/
void Set_ControlWord(uint32_t S_ID, uint16_t ControlWordTemp) // ZM 190507
{

  abSendData[S_ID * NO_S_BYTE + ControlWordRel] = ControlWordTemp & 0x00FF; // 0x07;
  abSendData[S_ID * NO_S_BYTE + ControlWordRel + 1] = (ControlWordTemp >> 8) & 0x00FF; // 0x07;
}

#pragma GCC diagnostic pop

/*****************************************************************************/
// 设置单个电机的全部参数
// S_ID:电机序号；Mode: 工作模式； Target
/*****************************************************************************/
void Set_Motor(uint32_t S_ID)
{
  Set_Mode(S_ID, MOT_Send[S_ID].Mode);
  Set_Position(S_ID, MOT_Send[S_ID].Position);
  Set_TargetVelocity(S_ID, MOT_Send[S_ID].TargetVelocity);
  Set_ProfileVelocity(S_ID, MOT_Send[S_ID].ProfileVelocity);
  Set_Torque(S_ID, MOT_Send[S_ID].Torque);
  Set_Acceleration(S_ID, MOT_Send[S_ID].Acceleration);
  Set_Deceleration(S_ID, MOT_Send[S_ID].Deceleration);
  Set_EndVelocity(S_ID, MOT_Send[S_ID].EndVelocity);
}
/*****************************************************************************/
// 发送数据
/*****************************************************************************/
int32_t IO_WriteZM(CIFXHANDLE hChannel)
{
  int32_t lRet;
  if(CIFX_NO_ERROR != (lRet = xChannelIOWrite(hChannel, 0, 0, sizeof(abSendData), abSendData, 100)))
  {
    printf("Error writing to IO Data area!\n");
    return lRet;
  }
  else
    return 0;
}

/*****************************************************************************/
// 读取数据
/*****************************************************************************/
int32_t Get_Mot_Data(CIFXHANDLE hChannel)
{
  int32_t lRet;
  uint32_t i = 0;
  if(CIFX_NO_ERROR != (lRet = xChannelIORead(hChannel, 0, 0, sizeof(abRecvData), abRecvData, 10)))
  {
    printf("Error reading IO Data area!\n");
    return 1;
  }
  else
  {
    for(i = 0; i < MOT_ID; i++)
    {
      memcpy((MOT_Recive + i), (abRecvData + i * NO_R_BYTE), NO_R_BYTE);
    }
  }
  for(size_t j = 0; j < ForceSensor_COUNT; ++j)
  {
    memcpy(&FS_Recive[j].status, &abRecvData[ForceSensorStartAddress + j * ForceSensor_BYTE], 2);
    memcpy(&FS_Recive[j].FX, &abRecvData[ForceSensorStartAddress + j * ForceSensor_BYTE + 2], ForceSensor_BYTE - 2);
  }
  //锟叫讹拷状态锟斤拷 Set_ControlWord(MotorNum, 0x86); //0x86   bit7---fault reset
  for(i = 0; i < MOT_ID; i++)
  {
    if((MOT_Recive[i].Status_Word & 0x00000008) != 0) // Fault锟斤拷锟斤拷 锟斤拷位Fault
    {
      abSendData[i * NO_S_BYTE + ControlWordRel] = abSendData[i * NO_S_BYTE + ControlWordRel] | 0x80;
    }
    else
    {
      abSendData[i * NO_S_BYTE + ControlWordRel] = EnOperation;
    }
  }
  return CIFX_NO_ERROR;
}
// 发生FaultRes复位 不判断
void FaultRes(CIFXHANDLE hChannel, uint32_t MotorID)
{
  int32_t ErrorCode = 0;
  usleep(200000);
  Get_Mot_Data(hChannel);
  Get_Mot_Data(hChannel);
  if((MOT_Recive[MotorID].Status_Word & 0x00000008) != 0) // Fault锟斤拷锟斤拷 锟斤拷位Fault
  {
    Set_ControlWord(MotorID, (MOT_Recive[MotorID].Status_Word | 0x80)); // 0x86   bit7---fault reset
    ErrorCode = IO_WriteZM(hChannel);
  }
  if(ErrorCode != 0)
  {
    printf("Error of FaultRes!\n");
  }
}

void FaultResAll(CIFXHANDLE hChannel)
{
  uint8_t i;
  usleep(200000);
  Get_Mot_Data(hChannel);
  Get_Mot_Data(hChannel);
  for(i = 0; i < MOT_ID; i++)
  {
    printf("Motor n Status_Word: %d 0x%X\n", i, MOT_Recive[i].Status_Word);
    if((MOT_Recive[i].Status_Word & 0x00000008) != 0) // Fault锟斤拷锟斤拷 锟斤拷位Fault
    {
      abSendData[i * NO_S_BYTE + ControlWordRel] = abSendData[i * NO_S_BYTE + ControlWordRel] | 0x80;
    }
  }
  int32_t ErrorCode = IO_WriteZM(hChannel);
  if(ErrorCode != 0)
  {
    printf("Error of FaultRes!\n");
  }
}

/*****************************************************************************
 *功能：断开
 *说明：S_ID：电机序列号
 *****************************************************************************/
void Power_OFF(CIFXHANDLE hChannel, uint32_t S_ID) // ZM 190507
{
  uint8_t i = 0;
  IO_WriteZM(hChannel);
  abSendData[S_ID * NO_S_BYTE + ControlWordRel] = SwithOff; // 0x06;
  Get_Mot_Data(hChannel);
  Get_Mot_Data(hChannel);
  while(((MOT_Recive[S_ID].Status_Word & 7) != PowerOffStatusFlag) && (i < 10))
  {
    usleep(100000);
    Get_Mot_Data(hChannel);
    i++;
  }
  if(i > 10)
  {
    printf("Error of Power_OFF motorNum %d\n", S_ID);
  }
}
/*****************************************************************************
 *功能：断开多个电机
 *说锟斤拷锟斤拷MotorNumTemp:锟斤拷锟斤拷锟斤拷锟�
 *****************************************************************************/
void Power_OFF_AllMoter(CIFXHANDLE hChannel, uint32_t MotorNumTemp) // ZM 2020.02.26
{
  for(uint32_t i = 0; i < MotorNumTemp; i++)
  {
    abSendData[i * NO_S_BYTE + ControlWordRel] = DisableVoltage; // 0x06;
  }
  IO_WriteZM(hChannel);
}

/*****************************************************************************
 *功能：开关使能
 *说明S_ID:电机序列号
 *****************************************************************************/
void Switch_On(CIFXHANDLE hChannel, uint32_t S_ID)
{
  uint8_t i = 0;
  abSendData[S_ID * NO_S_BYTE + ControlWordRel] = SHUTDOWN(cw); // SwithOnVoltageEn;   // 0x06;
  IO_WriteZM(hChannel);
  Get_Mot_Data(hChannel);
  Get_Mot_Data(hChannel);
  while((!(READY_TO_SWITCH_ON(MOT_Recive[S_ID].Status_Word))) && (i < 10)) // 0x21
  {
    usleep(100000);
    Get_Mot_Data(hChannel);
    i++;
  }
  if(i > 10)
  {
    printf("Error of Switch_On motorID %d\n", S_ID);
  }
}

/*****************************************************************************
 *功能： 所有电机开关使能
 *说明： S_ID：电机序列号
 *****************************************************************************/
void Switch_On_All(CIFXHANDLE hChannel, uint32_t Motor_NumTemp)
{
  uint8_t i = 0;
  uint32_t i1;
  for(i1 = 0; i1 < Motor_NumTemp; i1++)
  {
    abSendData[i1 * NO_S_BYTE + ControlWordRel] = SHUTDOWN(cw); // SwithOnVoltageEn;   // 0x06;
  }
  IO_WriteZM(hChannel);
  Get_Mot_Data(hChannel);
  Get_Mot_Data(hChannel);
  for(i1 = 0; i1 < Motor_NumTemp; i1++)
  {
    while((!(READY_TO_SWITCH_ON(MOT_Recive[i1].Status_Word))) && (i < 10)) // 0x21
    {
      usleep(100000);
      Get_Mot_Data(hChannel);
      i++;
    }
    if(i >= 10)
    {
      printf("Error of Switch_On motorID %d\n", i1);
    }
  }
}
/*****************************************************************************
 *功能：电压使能（没劲）
 *说明：S_ID:电机序列号
 *****************************************************************************/
void Voltage_En(CIFXHANDLE hChannel, uint32_t S_ID)
{
  uint8_t i = 0;
  abSendData[S_ID * NO_S_BYTE + ControlWordRel] = SWITCH_ON(cw); // 0x07;
  IO_WriteZM(hChannel);
  Get_Mot_Data(hChannel);
  Get_Mot_Data(hChannel);
  while((!(SWITCHED_ON(MOT_Recive[S_ID].Status_Word))) && (i < 10)) // 0x23
  {
    usleep(1000000);
    Get_Mot_Data(hChannel);
    i++;
  }
  if(i >= 10)
  {
    printf("Error of Voltage_En motorID %d\n", S_ID);
  }
}
/*****************************************************************************
 *功能：所有电机电压使能（没劲）
 *说明：S_ID:电机序列号
 *****************************************************************************/
void Voltage_En_All(CIFXHANDLE hChannel, uint32_t Motor_NumTemp)
{
  uint8_t i = 0;
  uint32_t i1 = 0;
  for(i1 = 0; i1 < Motor_NumTemp; i1++)
  {
    abSendData[i1 * NO_S_BYTE + ControlWordRel] = SWITCH_ON(cw); // 0x07;
  }
  IO_WriteZM(hChannel);
  Get_Mot_Data(hChannel);
  Get_Mot_Data(hChannel);
  for(i1 = 0; i1 < Motor_NumTemp; i1++)
  {
    while((!(SWITCHED_ON(MOT_Recive[i1].Status_Word))) && (i < 10)) // 0x23
    {
      usleep(100000);
      Get_Mot_Data(hChannel);
      i++;
    }
    if(i >= 10)
    {
      printf("Error of Voltage_En motorID %d\n", i1);
    }
  }
}

/*****************************************************************************
 *功能：伺服（有劲）
 *说明：S_ID：电机序列号
 *****************************************************************************/
void Enable_Operation(CIFXHANDLE hChannel, uint32_t S_ID)
{
  uint8_t i = 0;
  abSendData[S_ID * NO_S_BYTE + ControlWordRel] = ENABLE_OPERATIONAL(cw); // 0x0F;
  IO_WriteZM(hChannel);
  Get_Mot_Data(hChannel);
  Get_Mot_Data(hChannel);
  while((!(ONPERATION_ENABLED(MOT_Recive[S_ID].Status_Word))) && (i < 10)) // 0x27
  {
    usleep(100000);
    Get_Mot_Data(hChannel);
    i++;
  }
  if(i >= 10)
  {
    printf("Error of Enable_Operation motorID %d\n", S_ID);
  }
}

/*****************************************************************************
 *功能：伺服开单个电机（有劲）
 *说明：S_ID：电机序列号
 *****************************************************************************/
void Servo_On(CIFXHANDLE hChannel, uint32_t S_ID) // ZM 2020.04.2
{
  Switch_On(hChannel, S_ID);
  Voltage_En(hChannel, S_ID);
  Enable_Operation(hChannel, S_ID);
}

/*****************************************************************************
 *功能：伺服开所有电机（有劲）
 *说明：S_ID：电机序列号
 *****************************************************************************/
void Servo_On_All(CIFXHANDLE hChannel, uint32_t Motor_NumTemp) // ZM 2020.04.2
{
  Switch_On_All(hChannel, Motor_NumTemp);
  Voltage_En_All(hChannel, Motor_NumTemp);
  for(uint32_t i = 0; i < Motor_NumTemp; i++)
  {
    MOT_Send[i].Position = MOT_Recive[i].Position;
    Set_Motor(i);
  }
  for(uint32_t i = 0; i < Motor_NumTemp; i++)
  {
    Enable_Operation(hChannel, i);
  }
}
// 测试，所有电机初始化、赋值
void init_Motor(CIFXHANDLE hChannel)
{
  uint8_t i;
  int32_t EncodeFactor;
  for(i = 0; i < MOT_ID; i++)
  {
    EncodeFactor = nEncoder[i] / 4000;
    // MOT_Send[i].Control_Word = SwithOff;  //0x06
    MOT_Send[i].Control_Word = DisableVoltage;
    MOT_Send[i].Mode = ProPositionMode; // SynPositionMode;
    MOT_Send[i].TargetVelocity = 3000 * nGear[i] * EncodeFactor;
    MOT_Send[i].ProfileVelocity = 3000 * nGear[i] * EncodeFactor;
    MOT_Send[i].Acceleration = 0x00010000 * nGear[i] * EncodeFactor;
    MOT_Send[i].Deceleration = 0x00010000 * nGear[i] * EncodeFactor;
    MOT_Send[i].EndVelocity = 0;
    MOT_Send[i].Torque = 0;
    MOT_Send[i].Position = 0;
    Set_Motor(i);
  }
  sleep(5);
  IO_WriteZM(hChannel);

  // Fault检查
  printf("motor initialiaztion finished!\n");
  FaultResAll(hChannel); //错误检测
  printf("reset finished!\n");
}

void init_ForceSensor(CIFXHANDLE hChannel)
{
  Get_Mot_Data(hChannel);
  usleep(500000);
  Get_Mot_Data(hChannel);
  usleep(500000);
  Get_Mot_Data(hChannel);
  for(uint32_t i = 0; i < ForceSensor_COUNT; ++i)
  {
    printf("Force sensor n status: %d 0x%X\n", i, FS_Recive[i].status);
    InitFS_Recive[i] = FS_Recive[i];
  }
}

// 绝对位置运动初始化，单点设定位置模式，MotorNum:电机序号
void Set_Mode_PP(uint32_t MotorNum)
{
  Set_Mode(MotorNum, ProPositionMode);
}

// 循环同步位置模式初始化，MotorNum:电机序号
void Set_Mode_CSP(uint32_t MotorNum)
{
  Set_Mode(MotorNum, SynPositionMode);
}
// 循环同步力矩模式初始化，MotorNum:电机序号
void Set_Mode_CST(uint32_t MotorNum)
{
  Set_Mode(MotorNum, SynTorqueMode);
}

// 读多个电机MF
void ReadMulMotoMF(CIFXHANDLE hChannel)
{
  // 读错误代码
  uint32_t ErrorCodeTemp;
  uint8_t i;
  for(i = 0; i < MOT_ID; i++)
  {
    usleep(100000);
    ErrorCodeTemp = ReadMF(hChannel, i);
    printf("MF Ccde%u: 0x%X\n", i, ErrorCodeTemp); // 16进制
  }
}

// 读多个电机EC
void ReadMulMotoEC(CIFXHANDLE hChannel)
{
  uint32_t ErrorCodeTemp;
  uint8_t i;
  for(i = 0; i < MOT_ID; i++)
  {
    usleep(100000);
    ErrorCodeTemp = ReadEC(hChannel, i);
    printf("EC Ccde%u: 0x%X\n", i, ErrorCodeTemp);
  }
}

// 读多个电机EE
void ReadMulMotoEE(CIFXHANDLE hChannel)
{
  uint32_t ErrorCodeTemp;
  uint8_t i;
  for(i = 0; i < MOT_ID; i++)
  {
    usleep(100000);
    ErrorCodeTemp = ReadEE(hChannel, i);
    printf("EE Ccde%u: 0x%X\n", i, ErrorCodeTemp);
  }
}

// zm 2020.2.19
static uint32_t g_ulIdCnt = 0;
/***************************************************************************
 * Helper functions
 ****************************************************************************/
static long SendPacket(CIFXHANDLE hChannel, CIFX_PACKET * ptPacket)
{
  int32_t lRet = CIFX_NO_ERROR;

  ptPacket->tHeader.ulId = g_ulIdCnt;
  ptPacket->tHeader.ulDest = 0x20;
  ptPacket->tHeader.ulState = 0;

  lRet = xChannelPutPacket(hChannel, ptPacket, 1000);

  if(lRet)
  {
    printf("Error during xChannelPutPacket(hChannel, &ptPacket, 1000)\r\n");
    ShowError(lRet);
  }
  return lRet;
}

#define OS_Sleep(x) RtSleep(x)
static long RecvPacket(CIFXHANDLE hChannel, CIFX_PACKET * ptPacket)
{
  int32_t lRet = CIFX_NO_ERROR;
  uint32_t ulRecvCnt = 0, ulSendCnt = 0, ulWait = 20;

  while(--ulWait)
  {
    // OS_Sleep(50);
    usleep(50000);
    lRet = xChannelGetMBXState(hChannel, &ulRecvCnt, &ulSendCnt);
    if(lRet)
    {
      printf("Error during xChannelGetMBXState(hChannel,&ulRecvCnt, &ulSendCnt)\r\n");
    }

    if(ulRecvCnt)
    {
      lRet = xChannelGetPacket(hChannel, sizeof(*ptPacket), ptPacket, 0);
      if(lRet)
      {
        printf("Error during xChannelGetPacket(hChannel, sizeof(ptPacket), &ptPacket, 0)\r\n");
      }
      else
      {
        if(ptPacket->tHeader.ulId == g_ulIdCnt)
        {
          break;
        }
        else
        {
          printf("Warning a recived packet is out of sequence!");
        }
      }
    }
  }

  if(ulWait)
  {
    if(ptPacket->tHeader.ulState)
    {
      printf("Packet Error: 0x%08X\r\n", ptPacket->tHeader.ulState);
      lRet = (int32_t)ptPacket->tHeader.ulState;
    }
  }

  return lRet;
}

static long SendRecv(CIFXHANDLE hChannel, CIFX_PACKET * ptPacket)
{
  long lRet = 0;
  g_ulIdCnt += 1;

  lRet = SendPacket(hChannel, ptPacket);
  if(lRet == 0)
  {
    lRet = RecvPacket(hChannel, ptPacket);
  }
  return lRet;
}

//设置中断模式
#define RCX_IO_MODE_OEM_1 (0x5)
#define RCX_IO_MODE_OEM_2 (0x6)
long ECatMasterDev_SetSyncMode(CIFXHANDLE hChannel)
{
  CIFX_PACKET tPack = {0};
  RCX_SET_HANDSHAKE_CONFIG_REQ_T * ptPacket = (RCX_SET_HANDSHAKE_CONFIG_REQ_T *)&tPack;

  ptPacket->tHead.ulCmd = RCX_SET_HANDSHAKE_CONFIG_REQ;
  ptPacket->tHead.ulLen = RCX_SET_HANDSHAKE_CONFIG_REQ_SIZE;

  ptPacket->tData.bPDInHskMode = RCX_IO_MODE_OEM_1;
  ptPacket->tData.bPDInSource = 0;
  ptPacket->tData.usPDInErrorTh = 1;

  ptPacket->tData.bPDOutHskMode = RCX_IO_MODE_BUFF_HST_CTRL;
  ptPacket->tData.bPDOutSource = 0;
  ptPacket->tData.usPDOutErrorTh = 1;

  ptPacket->tData.bSyncHskMode = RCX_SYNC_MODE_OFF;
  ptPacket->tData.bSyncSource = 0;
  ptPacket->tData.usSyncErrorTh = 0;

  ptPacket->tData.aulReserved[0] = 0;
  ptPacket->tData.aulReserved[1] = 0;

  return SendRecv(hChannel, &tPack);
}
