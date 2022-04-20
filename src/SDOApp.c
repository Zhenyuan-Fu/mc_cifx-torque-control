#include "SDOApp.h"
#include "cifXUser.h"

#include <stdio.h>
#include <unistd.h>

int32_t SDOUploadCMD(CIFXHANDLE hChannel, uint32_t S_ID, CIFX_PACKET_SDO_Upload_Data SDO_Up_Data)
{
  CIFX_PACKET SDO_Up_PACKETCMD;
  CIFX_PACKET_HEADER SendPacketHead;
  SendPacketHead.ulDest = 0x00000020;
  SendPacketHead.ulSrc = 0;
  SendPacketHead.ulDestId = 0;
  SendPacketHead.ulSrcId = 0;
  SendPacketHead.ulLen = 12;
  SendPacketHead.ulId = 0;
  SendPacketHead.ulState = 0;
  SendPacketHead.ulCmd = 0x00650006;
  SendPacketHead.ulExt = 0;
  SendPacketHead.ulRout = 0;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
  // node 32bit
  SDO_Up_Data.ulNodeID = 0x0100 + S_ID;
  SDO_Up_PACKETCMD.abData[0] = SDO_Up_Data.ulNodeID & 0x000000ff;
  SDO_Up_PACKETCMD.abData[1] = (SDO_Up_Data.ulNodeID & 0x0000ff00) >> 8;
  SDO_Up_PACKETCMD.abData[2] = (SDO_Up_Data.ulNodeID & 0x00ff0000) >> 16;
  SDO_Up_PACKETCMD.abData[3] = (SDO_Up_Data.ulNodeID & 0xff000000) >> 24;
  // node 32bit
  SDO_Up_PACKETCMD.abData[4] = SDO_Up_Data.ulIndex & 0x000000ff;
  SDO_Up_PACKETCMD.abData[5] = (SDO_Up_Data.ulIndex & 0x0000ff00) >> 8;
  SDO_Up_PACKETCMD.abData[6] = (SDO_Up_Data.ulIndex & 0x00ff0000) >> 16;
  SDO_Up_PACKETCMD.abData[7] = (SDO_Up_Data.ulIndex & 0xff000000) >> 24;
  // node 32bit
  SDO_Up_PACKETCMD.abData[8] = SDO_Up_Data.ulSubIndex & 0x000000ff;
  SDO_Up_PACKETCMD.abData[9] = (SDO_Up_Data.ulSubIndex & 0x0000ff00) >> 8;
  SDO_Up_PACKETCMD.abData[10] = (SDO_Up_Data.ulSubIndex & 0x00ff0000) >> 16;
  SDO_Up_PACKETCMD.abData[11] = (SDO_Up_Data.ulSubIndex & 0xff000000) >> 24;
#pragma GCC diagnostic pop

  SDO_Up_PACKETCMD.tHeader = SendPacketHead;
  return (xChannelPutPacket(hChannel, &SDO_Up_PACKETCMD, 30));
}

SDOReadResult SDOReadData(CIFXHANDLE hChannel, uint32_t DataLength)
{
  CIFX_PACKET SDO_Download_PACKET;
  CIFX_PACKET_HEADER SDO_DownloadtHead;
  uint32_t PacketLethTemp;
  uint32_t i;
  uint8_t abData2;
  PacketLethTemp = (16 + DataLength) + 4 * 10;
  SDOReadResult out = {0};
  out.error = xChannelGetPacket(hChannel, PacketLethTemp, &SDO_Download_PACKET, 30);
  if(out.error == 0)
  {
    SDO_DownloadtHead = SDO_Download_PACKET.tHeader;
    if(SDO_DownloadtHead.ulCmd == 0x00650007) // Read 650006--650007  Write:650008--650009
    {
      DataLength = SDO_Download_PACKET.abData[12] | ((uint32_t)SDO_Download_PACKET.abData[13] << 8)
                   | ((uint32_t)SDO_Download_PACKET.abData[14] << 16)
                   | ((uint32_t)SDO_Download_PACKET.abData[15] << 24);
      out.data = 0;
      for(i = DataLength; i > 1; i--)
      {
        abData2 = SDO_Download_PACKET.abData[15 + i];
        out.data = (out.data | abData2) << 8;
      }
      abData2 = SDO_Download_PACKET.abData[16];
      out.data = out.data | abData2;
    }
  }
  return out;
}

SDOReadResult SDORead(CIFXHANDLE hChannel,
                      uint32_t S_ID,
                      CIFX_PACKET_SDO_Upload_Data SDO_Up_Data_Temp,
                      uint32_t DataLength)
{
  SDOReadResult out = {0};
  out.error = SDOUploadCMD(hChannel, S_ID, SDO_Up_Data_Temp);
  usleep(100000);
  if(out.error == 0)
  {
    out = SDOReadData(hChannel, DataLength);
    while(out.error == CIFX_DEV_GET_NO_PACKET)
    {
      usleep(100000);
      out = SDOReadData(hChannel, DataLength);
    }
    return out;
  }
  else
  {
    printf("SDOReadCMD Error ErrorCode: %x/n", out.error);
  }
  return out;
}

int32_t SDODownload(CIFXHANDLE hChannel, uint32_t S_ID, CIFX_PACKET_SDO_Download_Data SDO_Download_Data)
{
  uint32_t DataLength;
  CIFX_PACKET SDO_Download_PACKET;
  CIFX_PACKET_HEADER SendPacketHead;
  uint8_t i;
  DataLength = SDO_Download_Data.ulDataCnt;
  SendPacketHead.ulDest = 0x00000020;
  SendPacketHead.ulSrc = 0;
  SendPacketHead.ulDestId = 0;
  SendPacketHead.ulSrcId = 0;
  SendPacketHead.ulLen = 16 + DataLength;
  SendPacketHead.ulId = 0;
  SendPacketHead.ulState = 0;
  SendPacketHead.ulCmd = 0x00650008;
  SendPacketHead.ulExt = 0;
  SendPacketHead.ulRout = 0;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
  // node 32bit
  SDO_Download_Data.ulNodeID = 0x0100 + S_ID;
  SDO_Download_PACKET.abData[0] = SDO_Download_Data.ulNodeID & 0x000000ff;
  SDO_Download_PACKET.abData[1] = (SDO_Download_Data.ulNodeID & 0x0000ff00) >> 8;
  SDO_Download_PACKET.abData[2] = (SDO_Download_Data.ulNodeID & 0x00ff0000) >> 16;
  SDO_Download_PACKET.abData[3] = (SDO_Download_Data.ulNodeID & 0xff000000) >> 24;
  // node 32bit
  SDO_Download_PACKET.abData[4] = SDO_Download_Data.ulIndex & 0x000000ff;
  SDO_Download_PACKET.abData[5] = (SDO_Download_Data.ulIndex & 0x0000ff00) >> 8;
  SDO_Download_PACKET.abData[6] = (SDO_Download_Data.ulIndex & 0x00ff0000) >> 16;
  SDO_Download_PACKET.abData[7] = (SDO_Download_Data.ulIndex & 0xff000000) >> 24;
  // node 32bit
  SDO_Download_PACKET.abData[8] = SDO_Download_Data.ulSubIndex & 0x000000ff;
  SDO_Download_PACKET.abData[9] = (SDO_Download_Data.ulSubIndex & 0x0000ff00) >> 8;
  SDO_Download_PACKET.abData[10] = (SDO_Download_Data.ulSubIndex & 0x00ff0000) >> 16;
  SDO_Download_PACKET.abData[11] = (SDO_Download_Data.ulSubIndex & 0xff000000) >> 24;
  // node 32bit
  SDO_Download_PACKET.abData[12] = SDO_Download_Data.ulDataCnt & 0x000000ff;
  SDO_Download_PACKET.abData[13] = (SDO_Download_Data.ulDataCnt & 0x0000ff00) >> 8;
  SDO_Download_PACKET.abData[14] = (SDO_Download_Data.ulDataCnt & 0x00ff0000) >> 16;
  SDO_Download_PACKET.abData[15] = (SDO_Download_Data.ulDataCnt & 0xff000000) >> 24;
#pragma GCC diagnostic pop
  for(i = 0; i < DataLength; i++)
  {
    SDO_Download_PACKET.abData[16 + i] = *(SDO_Download_Data.abSdoData + i);
  }

  SDO_Download_PACKET.tHeader = SendPacketHead;
  return (xChannelPutPacket(hChannel, &SDO_Download_PACKET, 0));
}

void SetMotorRateCurrent(CIFXHANDLE hChannel, uint32_t S_ID)
{
  uint8_t i, DataLength;
  uint8_t DataTemp[4];
  uint32_t maxCurrent;
  CIFX_PACKET_SDO_Download_Data SDO_Download_Data_Temp;

  SDO_Download_Data_Temp.ulNodeID = 0x0100 + S_ID;
  SDO_Download_Data_Temp.ulIndex = 0x6075;
  SDO_Download_Data_Temp.ulSubIndex = 0;
  SDO_Download_Data_Temp.ulDataCnt = 4;
  // FIXME NOT A GOOD WAY TO IDENTIFY THE MAX CURRENT
  if(S_ID == 4)
  {
    maxCurrent = 2500;
  }
  else
  {
    maxCurrent = 1000;
  }
  for(i = 0; i < SDO_Download_Data_Temp.ulDataCnt; i++)
  {
    DataTemp[i] = maxCurrent & 0x000000ff;
    maxCurrent = maxCurrent >> 8;
  }
  SDO_Download_Data_Temp.abSdoData = DataTemp;
  SDODownload(hChannel, S_ID, SDO_Download_Data_Temp);
  usleep(200000);
  DataLength = 4;
  SDOReadData(hChannel, DataLength);
}

uint32_t ReadMotorRateCurrent(CIFXHANDLE hChannel, uint32_t S_ID)
{
  CIFX_PACKET_SDO_Upload_Data SDO_Up_Data_Temp;
  uint8_t DataLength;
  SDO_Up_Data_Temp.ulNodeID = 0x0100 + S_ID;
  SDO_Up_Data_Temp.ulIndex = 0x6075;
  SDO_Up_Data_Temp.ulSubIndex = 0;
  DataLength = 4;
  SDOReadResult ReciveData = SDORead(hChannel, S_ID, SDO_Up_Data_Temp, DataLength);
  return ReciveData.data;
}

uint32_t SetMotorRateTorque(CIFXHANDLE hChannel, uint32_t S_ID)
{
  uint8_t i, DataLength;
  uint8_t DataTemp[4];
  uint32_t maxTorque;
  CIFX_PACKET_SDO_Download_Data SDO_Download_Data_Temp;
  SDO_Download_Data_Temp.ulNodeID = 0x0100 + S_ID;
  SDO_Download_Data_Temp.ulIndex = 0x6076;
  SDO_Download_Data_Temp.ulSubIndex = 0;
  SDO_Download_Data_Temp.ulDataCnt = 4;
  if(S_ID == 0)
  {
    maxTorque = 2500;
  }
  else
  {
    maxTorque = 1000;
  }
  for(i = 0; i < SDO_Download_Data_Temp.ulDataCnt; i++)
  {
    DataTemp[i] = maxTorque & 0x000000ff;
    maxTorque = maxTorque >> 8;
  }
  SDO_Download_Data_Temp.abSdoData = DataTemp;

  SDODownload(hChannel, S_ID, SDO_Download_Data_Temp);
  usleep(200000);
  DataLength = 4;
  SDOReadResult ReciveData = SDOReadData(hChannel, DataLength);
  return ReciveData.data;
}

uint32_t ReadMotorRateTorque(CIFXHANDLE hChannel, uint32_t S_ID)
{
  CIFX_PACKET_SDO_Upload_Data SDO_Up_Data_Temp;
  uint8_t DataLength;
  SDO_Up_Data_Temp.ulNodeID = 0x0100 + S_ID;
  SDO_Up_Data_Temp.ulIndex = 0x6076;
  SDO_Up_Data_Temp.ulSubIndex = 0;
  DataLength = 4;
  SDOReadResult ReciveData = SDORead(hChannel, S_ID, SDO_Up_Data_Temp, DataLength);
  return ReciveData.data;
}

uint32_t ReadMF(CIFXHANDLE hChannel, uint32_t S_ID)
{
  CIFX_PACKET_SDO_Upload_Data SDO_Up_Data_Temp;
  uint8_t DataLength;
  SDO_Up_Data_Temp.ulNodeID = 0x0100 + S_ID;
  SDO_Up_Data_Temp.ulIndex = 0x313D;
  SDO_Up_Data_Temp.ulSubIndex = 1;
  DataLength = 4;
  SDOReadResult ReciveData = SDORead(hChannel, S_ID, SDO_Up_Data_Temp, DataLength);
  return ReciveData.data;
}

uint32_t ReadEC(CIFXHANDLE hChannel, uint32_t S_ID)
{
  CIFX_PACKET_SDO_Upload_Data SDO_Up_Data_Temp;
  uint8_t DataLength;
  SDO_Up_Data_Temp.ulNodeID = 0x0100 + S_ID;
  SDO_Up_Data_Temp.ulIndex = 0x306A;
  SDO_Up_Data_Temp.ulSubIndex = 1;
  DataLength = 4;
  SDOReadResult ReciveData = SDORead(hChannel, S_ID, SDO_Up_Data_Temp, DataLength);
  return ReciveData.data;
}

uint32_t ReadEE(CIFXHANDLE hChannel, uint32_t S_ID)
{
  CIFX_PACKET_SDO_Upload_Data SDO_Up_Data_Temp;
  uint8_t DataLength;
  SDO_Up_Data_Temp.ulNodeID = 0x0100 + S_ID;
  SDO_Up_Data_Temp.ulIndex = 0x306C;
  SDO_Up_Data_Temp.ulSubIndex = 1;
  DataLength = 4;
  SDOReadResult ReciveData = SDORead(hChannel, S_ID, SDO_Up_Data_Temp, DataLength);
  return ReciveData.data;
}

void SDODInit(CIFXHANDLE hChannel, uint32_t S_ID)
{
  SetMotorRateTorque(hChannel, S_ID);
  usleep(100000);
  ReadMotorRateTorque(hChannel, S_ID);
  usleep(100000);
  SetMotorRateCurrent(hChannel, S_ID);
  usleep(10000);
  ReadMotorRateCurrent(hChannel, S_ID);
  usleep(100000);
}

void SDODInit_All(CIFXHANDLE hChannel, uint32_t Mot_NumTemp)
{
  for(uint32_t i = 0; i < Mot_NumTemp; i++)
  {
    SDODInit(hChannel, i);
  }
  printf("SDODInit Finished\n");
}

int32_t ZeroClearSDOD(CIFXHANDLE hChannel, uint32_t S_ID)
{
  uint8_t DataTemp[4];
  CIFX_PACKET_SDO_Download_Data SDO_Download_Data_Temp;
  SDO_Download_Data_Temp.ulNodeID = 0x0100 + S_ID;
  SDO_Download_Data_Temp.ulIndex = 0x319D;
  SDO_Download_Data_Temp.ulSubIndex = 1;
  SDO_Download_Data_Temp.ulDataCnt = 4;
  DataTemp[0] = 0;
  DataTemp[1] = 0;
  DataTemp[2] = 0;
  DataTemp[3] = 0;
  SDO_Download_Data_Temp.abSdoData = DataTemp; //д������ָ��
  int32_t ErrorCode = SDODownload(hChannel, S_ID, SDO_Download_Data_Temp);
  usleep(100000);
  return ErrorCode;
}
