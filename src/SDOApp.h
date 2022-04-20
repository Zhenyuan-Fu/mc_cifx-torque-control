#include "cifXErrors.h"
#include "cifXUser.h"

#define InterpolationPeriodIndx 0x60c2
#define InterpolationPeriodSubIndx 2
#define PositionOptionCodeBuffer 0x20 //不用应答状态字bit4 buffer中有数就按新点执行
#define PositionOptionCodePreCon 0x10 //不用应答状态字bit4 原来点执行完成立即执行新点

//写SDO数据包
typedef struct EherCar_Master_Packet_SDO_Download_Req_Data_Ttag
{
  uint32_t ulNodeID;
  uint32_t ulIndex;
  uint32_t ulSubIndex;
  uint32_t ulDataCnt;
  uint8_t * abSdoData;
} CIFX_PACKET_SDO_Download_Data;

//读SDO数据包
typedef struct EherCar_Master_Packet_SDO_Upload_Req_Data_Ttag
{
  uint32_t ulNodeID;
  uint32_t ulIndex;
  uint32_t ulSubIndex;
} CIFX_PACKET_SDO_Upload_Data;

typedef struct SDOReadResult
{
  int32_t error;
  uint32_t data;
} SDOReadResult;

SDOReadResult SDOReadData(CIFXHANDLE hChannel, uint32_t DataLength);
SDOReadResult SDORead(CIFXHANDLE hChannel,
                      uint32_t S_ID,
                      CIFX_PACKET_SDO_Upload_Data SDO_Up_Data_Temp,
                      uint32_t DataLength);

/*****************************************************************************/
//通过SDO包更新驱动器对象字典
// S_ID:电机序号； SDO__Download_Data:下发SDO数据包--结点号、索引、子索引、数据长度、数据
/*****************************************************************************/
int32_t SDODownload(CIFXHANDLE hChannel, uint32_t S_ID, CIFX_PACKET_SDO_Download_Data SDO_Download_Data);

/*****************************************************************************/
// SDO初始化
/*****************************************************************************/
void SDODInit(CIFXHANDLE hChannel, uint32_t S_ID);
//清零
int32_t ZeroClearSDOD(CIFXHANDLE hChannel, uint32_t S_ID);

//设置0x6075--Motor Rate Current 电流输入值1---A，1000--mA
void SetMotorRateCurrent(CIFXHANDLE hChannel, uint32_t S_ID);
//读0x6075--Motor Rate Current 电流输入值1---A，1000--mA
uint32_t ReadMotorRateCurrent(CIFXHANDLE hChannel, uint32_t S_ID);

//设置0x6076--Motor Rate Torque 力矩模式输入值1---A，1000--mA
uint32_t SetMotorRateTorque(CIFXHANDLE hChannel, uint32_t S_ID);
//读0x6076--Motor Rate Torque 力矩模式输入值1---A，1000--mA
uint32_t ReadMotorRateTorque(CIFXHANDLE hChannel, uint32_t S_ID);

uint32_t ReadMF(CIFXHANDLE hChannel, uint32_t S_ID);
uint32_t ReadEC(CIFXHANDLE hChannel, uint32_t S_ID);
//读EE(Elmo-OX306C)
uint32_t ReadEE(CIFXHANDLE hChannel, uint32_t S_ID);
//设置0x0x3146--MO=1 清除MF错误码
//设置0x0x6007--Abort connect option code  驱动器心跳停止功能取消
void SDODInit_All(CIFXHANDLE hChannel, uint32_t Mot_NumTemp);
