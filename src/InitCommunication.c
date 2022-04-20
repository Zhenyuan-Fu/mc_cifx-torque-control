#include "InitCommunication.h"

#include <cifXEndianess.h>
#include <cifxlinux.h>

#include <Hil_SystemCmd.h>

#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include "Display.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdiscarded-qualifiers"
static char * CIFX_DEV = "cifX0";
#pragma GCC diagnostic pop

/*! Dumps a rcX packet to debug console
 *   \param ptPacket Pointer to packed being dumped
 */
void DumpPacket(CIFX_PACKET * ptPacket)
{
#ifdef DEBUG
  printf("%s() called\n", __FUNCTION__);
#endif
  printf("Dest   : 0x%08lX      ID   : 0x%08lX\r\n", (long unsigned int)HOST_TO_LE32(ptPacket->tHeader.ulDest),
         (long unsigned int)HOST_TO_LE32(ptPacket->tHeader.ulId));
  printf("Src    : 0x%08lX      Sta  : 0x%08lX\r\n", (long unsigned int)HOST_TO_LE32(ptPacket->tHeader.ulSrc),
         (long unsigned int)HOST_TO_LE32(ptPacket->tHeader.ulState));
  printf("DestID : 0x%08lX      Cmd  : 0x%08lX\r\n", (long unsigned int)HOST_TO_LE32(ptPacket->tHeader.ulDestId),
         (long unsigned int)HOST_TO_LE32(ptPacket->tHeader.ulCmd));
  printf("SrcID  : 0x%08lX      Ext  : 0x%08lX\r\n", (long unsigned int)HOST_TO_LE32(ptPacket->tHeader.ulSrcId),
         (long unsigned int)HOST_TO_LE32(ptPacket->tHeader.ulExt));
  printf("Len    : 0x%08lX      Rout : 0x%08lX\r\n", (long unsigned int)HOST_TO_LE32(ptPacket->tHeader.ulLen),
         (long unsigned int)HOST_TO_LE32(ptPacket->tHeader.ulRout));

  printf("Data:");
  DumpData(ptPacket->abData, HOST_TO_LE32(ptPacket->tHeader.ulLen));
}

/*! Function to display driver information
 *   \param  hDriver  Handle to cifX driver
 *   \param  ptVTable Pointer to cifX API function table
 *   \return CIFX_NO_ERROR on success
 */
void DisplayDriverInformation()
{
  int32_t lRet = CIFX_NO_ERROR;
  DRIVER_INFORMATION tDriverInfo;
  char szDrvVersion[32] = "";
  CIFXHANDLE hDriver = NULL;

  if(CIFX_NO_ERROR == (lRet = xDriverOpen(&hDriver)))
  {
    printf("\n---------- Display Driver Version ----------\n");
    if(CIFX_NO_ERROR != (lRet = xDriverGetInformation(NULL, sizeof(tDriverInfo), &tDriverInfo)))
      ShowError(lRet);
    else if(CIFX_NO_ERROR != (lRet = cifXGetDriverVersion(sizeof(szDrvVersion) / sizeof(*szDrvVersion), szDrvVersion)))
      ShowError(lRet);
    else
      printf("Driver Version: %s, based on %.32s \n\n", szDrvVersion, tDriverInfo.abDriverVersion);

    /* close previously opened driver */
    xDriverClose(hDriver);
  }

  printf(" State = 0x%08X\r\n", (unsigned int)lRet);
  printf("----------------------------------------------------\r\n");
}

/*! Function to demonstrate communication channel functionality
 *   Packet Transfer and I/O Data exchange
 *   \return CIFX_NO_ERROR on success
 */
int32_t ChannelDemo(CIFXHANDLE * hDriver, CIFXHANDLE * hChannel)
{
#ifdef DEBUG
  printf("%s() called\n", __FUNCTION__);
#endif
  int32_t lRet = xDriverOpen(hDriver);

  printf("---------- Communication Channel demo ----------\r\n");

  if(CIFX_NO_ERROR == lRet)
  {
    /* Driver/Toolkit successfully opened */
    lRet = xChannelOpen(*hDriver, CIFX_DEV, 0, hChannel);

    if(CIFX_NO_ERROR != lRet)
    {
      printf("Error opening Channel!");
    }
    else
    {
      CHANNEL_INFORMATION tChannelInfo;

      /* Channel successfully opened, so query basic information */
      if(CIFX_NO_ERROR != (lRet = xChannelInfo(*hChannel, sizeof(CHANNEL_INFORMATION), &tChannelInfo)))
      {
        printf("Error querying system information block\r\n");
      }
      else
      {
        printf("Communication Channel Info:\r\n");
        printf("Device Number    : %lu\r\n", (long unsigned int)tChannelInfo.ulDeviceNumber);
        printf("Serial Number    : %lu\r\n", (long unsigned int)tChannelInfo.ulSerialNumber);
        printf("Firmware         : %s\r\n", tChannelInfo.abFWName);
        printf("FW Version       : %u.%u.%u build %u\r\n", tChannelInfo.usFWMajor, tChannelInfo.usFWMinor,
               tChannelInfo.usFWRevision, tChannelInfo.usFWBuild);
        printf("FW Date          : %02u/%02u/%04u\r\n", tChannelInfo.bFWMonth, tChannelInfo.bFWDay,
               tChannelInfo.usFWYear);

        printf("Mailbox Size     : %lu\r\n", (long unsigned int)tChannelInfo.ulMailboxSize);
      }
    }
  }
  printf(" State = 0x%08X\r\n", (unsigned int)lRet);
  printf("----------------------------------------------------\r\n");

  return lRet;
}

/*! Function to demonstrate system channel functionality (PacketTransfer)
 *   \return CIFX_NO_ERROR on success
 */
int32_t SysdeviceDemo()
{
#ifdef DEBUG
  printf("%s() called\n", __FUNCTION__);
#endif
  CIFXHANDLE hDriver = NULL;
  int32_t lRet = xDriverOpen(&hDriver);

  printf("---------- System Device handling demo ----------\r\n");

  if(CIFX_NO_ERROR == lRet)
  {
    /* Driver/Toolkit successfully opened */
    CIFXHANDLE hSys = NULL;
    lRet = xSysdeviceOpen(hDriver, CIFX_DEV, &hSys);

    if(CIFX_NO_ERROR != lRet)
    {
      printf("Error opening SystemDevice!\r\n");
    }
    else
    {
      SYSTEM_CHANNEL_SYSTEM_INFO_BLOCK tSysInfoBlock;
      SYSTEM_CHANNEL_SYSTEM_INFORMATION tSysInfo;
      SYSTEM_CHANNEL_SYSTEM_CONTROL_BLOCK tControlBlock;
      SYSTEM_CHANNEL_SYSTEM_STATUS_BLOCK tStatusBlock;

      /* System channel successfully opened, try to read the System Info Block */
      if(CIFX_NO_ERROR
         != (lRet = xSysdeviceInfo(hSys, CIFX_INFO_CMD_SYSTEM_INFO_BLOCK, sizeof(tSysInfoBlock), &tSysInfoBlock)))
      {
        printf("Error querying system information block\r\n");
      }
      else
      {
        printf("System Channel Info Block:\r\n");
        printf("==========================\r\n");
        printf("DPM Cookie       : %.4s\r\n", (char *)tSysInfoBlock.abCookie);
        printf("DPM Size         : %lu\r\n", (long unsigned int)tSysInfoBlock.ulDpmTotalSize);
        printf("Device Number    : %lu\r\n", (long unsigned int)tSysInfoBlock.ulDeviceNumber);
        printf("Serial Number    : %lu\r\n", (long unsigned int)tSysInfoBlock.ulSerialNumber);
        printf("HW Options       : 0x%04X 0x%04X 0x%04X 0x%04X\r\n", tSysInfoBlock.ausHwOptions[0],
               tSysInfoBlock.ausHwOptions[1], tSysInfoBlock.ausHwOptions[2], tSysInfoBlock.ausHwOptions[3]);
        printf("Manufacturer     : %u\r\n", tSysInfoBlock.usManufacturer);
        printf("Production Date  : %u\r\n", tSysInfoBlock.usProductionDate);
        printf("Device Class     : %u\r\n", tSysInfoBlock.usDeviceClass);
        printf("HW Revision      : %u\r\n", tSysInfoBlock.bHwRevision);
        printf("HW Compatibility : %u\r\n", tSysInfoBlock.bHwCompatibility);

        printf("License Flags 1  : 0x%08X\r\n", tSysInfoBlock.ulLicenseFlags1);
        printf("License Flags 2  : 0x%08X\r\n", tSysInfoBlock.ulLicenseFlags2);
        printf("LicenseID        : 0x%04X\r\n", tSysInfoBlock.usNetxLicenseID);
        printf("LicenseFlags     : 0x%04X\r\n", tSysInfoBlock.usNetxLicenseFlags);
        printf("==========================\r\n");
      }

      /* Try to read the System Information */
      if(CIFX_NO_ERROR != (lRet = xSysdeviceInfo(hSys, CIFX_INFO_CMD_SYSTEM_INFORMATION, sizeof(tSysInfo), &tSysInfo)))
      {
        printf("Error querying system information\r\n");
      }
      else
      {
        printf("System Information:\r\n");
        printf("===================\r\n");
        printf("System Error     : 0x%08X\r\n", tSysInfo.ulSystemError);
        printf("DPM Size         : %lu\r\n", (long unsigned int)tSysInfo.ulDpmTotalSize);
        printf("Mailbox size     : %lu\r\n", (long unsigned int)tSysInfo.ulMBXSize);
        printf("Device Number    : %lu\r\n", (long unsigned int)tSysInfo.ulDeviceNumber);
        printf("Serial Number    : %lu\r\n", (long unsigned int)tSysInfo.ulSerialNumber);
        printf("Open Count       : %lu\r\n", (long unsigned int)tSysInfo.ulOpenCnt);
        printf("===================\r\n");
      }

      /* Try to read the System Control Block */
      if(CIFX_NO_ERROR
         != (lRet = xSysdeviceInfo(hSys, CIFX_INFO_CMD_SYSTEM_CONTROL_BLOCK, sizeof(tControlBlock), &tControlBlock)))
      {
        printf("Error querying system control block\r\n");
      }
      else
      {
        printf("System Control Block:\r\n");
        printf("=====================\r\n");
        printf("Command COS      : 0x%08X\r\n", tControlBlock.ulSystemCommandCOS);
        printf("System Control   : 0x%08X\r\n", tControlBlock.ulSystemControl);
        printf("=====================\r\n");
      }

      printf("Waiting 2s to let cifX card calculate CPU load!\r\n");
      sleep(2);

      /* Try to read the System Status Block */
      if(CIFX_NO_ERROR
         != (lRet = xSysdeviceInfo(hSys, CIFX_INFO_CMD_SYSTEM_STATUS_BLOCK, sizeof(tStatusBlock), &tStatusBlock)))
      {
        printf("Error querying system status block\r\n");
      }
      else
      {
        printf("System Status Block:\r\n");
        printf("====================\r\n");
        printf("System COS       : 0x%08X\r\n", tStatusBlock.ulSystemCOS);
        printf("System Status    : 0x%08X\r\n", tStatusBlock.ulSystemStatus);
        printf("System Error     : 0x%08X\r\n", tStatusBlock.ulSystemError);
        printf("Time since start : %lu\r\n", (long unsigned int)tStatusBlock.ulTimeSinceStart);
        printf("CPU Load [%%]     : %.2f\r\n", (float)tStatusBlock.usCpuLoad / 100);
        printf("====================\r\n");
      }

      unsigned long ulSendPktCount = 0;
      unsigned long ulRecvPktCount = 0;

      printf("\r\n");
      printf("Trying to read Security Eeprom:\r\n");
      printf("===============================\r\n");

      /* Read Security EEPROM zone 1*/
      xSysdeviceGetMBXState(hSys, (uint32_t *)&ulRecvPktCount, (uint32_t *)&ulSendPktCount);
      printf("System Mailbox State: MaxSend = %lu, Pending Receive = %lu\r\n", ulSendPktCount, ulRecvPktCount);

      HIL_SECURITY_EEPROM_READ_REQ_T tCryptoRead;
      HIL_SECURITY_EEPROM_READ_CNF_T tCryptoReadCnf;

      tCryptoRead.tHead.ulDest = HOST_TO_LE32(HIL_PACKET_DEST_SYSTEM);
      tCryptoRead.tHead.ulLen = HOST_TO_LE32(sizeof(tCryptoRead.tData));
      tCryptoRead.tHead.ulCmd = HOST_TO_LE32(HIL_SECURITY_EEPROM_READ_REQ);
      tCryptoRead.tData.ulZoneId = HOST_TO_LE32(1);

      if(CIFX_NO_ERROR != (lRet = xSysdevicePutPacket(hSys, (CIFX_PACKET *)&tCryptoRead, 10)))
      {
        printf("Error sending packet to device (0x%X)!\r\n", lRet);
      }
      else
      {
        printf("Send Packet (Read Crypto Flash Zone 1):\r\n");
        DumpPacket((CIFX_PACKET *)&tCryptoRead);

        xSysdeviceGetMBXState(hSys, (uint32_t *)&ulRecvPktCount, (uint32_t *)&ulSendPktCount);
        printf("System Mailbox State: MaxSend = %lu, Pending Receive = %lu\r\n", ulSendPktCount, ulRecvPktCount);

        if(CIFX_NO_ERROR
           != (lRet = xSysdeviceGetPacket(hSys, sizeof(tCryptoReadCnf), (CIFX_PACKET *)&tCryptoReadCnf, 20)))
        {
          printf("Error getting packet from device! (lRet=0x%08X)\r\n", (unsigned int)lRet);
        }
        else
        {
          printf("Received Packet (Read Crypto Flash Zone 1):\r\n");
          DumpPacket((CIFX_PACKET *)&tCryptoReadCnf);

          xSysdeviceGetMBXState(hSys, (uint32_t *)&ulRecvPktCount, (uint32_t *)&ulSendPktCount);
          printf("System Mailbox State: MaxSend = %lu, Pending Receive = %lu\r\n", ulSendPktCount, ulRecvPktCount);
        }
      }

      printf("===============================\r\n");
      printf("\r\n");

      xSysdeviceClose(hSys);
    }

    xDriverClose(hDriver);
  }

  printf(" State = 0x%08X\r\n", (unsigned int)lRet);
  printf("----------------------------------------------------\r\n");

  return lRet;
}

/*! Function to demonstrate the board/channel enumeration
 *   \param  hDriver  Handle to cifX driver
 *   \return CIFX_NO_ERROR on success
 */
long EnumBoardDemo()
{
  uint32_t ulBoard = 0;
  BOARD_INFORMATION tBoardInfo = {0};
  CIFXHANDLE hDriver = NULL;
  int32_t lRet = xDriverOpen(&hDriver);
  printf("\n---------- Board/Channel enumeration demo ----------\n");
  if(CIFX_NO_ERROR == lRet)
  {
    /* Iterate over all boards */
    while(CIFX_NO_ERROR == xDriverEnumBoards(hDriver, ulBoard, sizeof(tBoardInfo), &tBoardInfo))
    {
      printf("Found Board %s\n", tBoardInfo.abBoardName);
      if(strlen((char *)tBoardInfo.abBoardAlias) != 0) printf(" Alias        : %s\n", tBoardInfo.abBoardAlias);

      printf(" DeviceNumber : %u\n", tBoardInfo.tSystemInfo.ulDeviceNumber);
      printf(" SerialNumber : %u\n", tBoardInfo.tSystemInfo.ulSerialNumber);
      printf(" Board ID     : %u\n", tBoardInfo.ulBoardID);
      printf(" System Error : 0x%08X\n", tBoardInfo.ulSystemError);
      printf(" Channels     : %u\n", tBoardInfo.ulChannelCnt);
      printf(" DPM Size     : %u\n", tBoardInfo.ulDpmTotalSize);

      {
        uint32_t ulChannel = 0;
        CHANNEL_INFORMATION tChannelInfo;

        /* iterate over all channels on the current board */
        while(CIFX_NO_ERROR == xDriverEnumChannels(hDriver, ulBoard, ulChannel, sizeof(tChannelInfo), &tChannelInfo))
        {
          printf(" - Channel %u:\n", ulChannel);
          printf("    Firmware : %s\n", tChannelInfo.abFWName);
          printf("    Version  : %u.%u.%u build %u\n", tChannelInfo.usFWMajor, tChannelInfo.usFWMinor,
                 tChannelInfo.usFWRevision, tChannelInfo.usFWBuild);
          printf("    Date     : %02u/%02u/%04u\n", tChannelInfo.bFWMonth, tChannelInfo.bFWDay, tChannelInfo.usFWYear);

          ++ulChannel;
        }
      }
      ++ulBoard;
    }
    xDriverClose(hDriver);
  }
  printf(" State = 0x%08X\n", lRet);
  printf("----------------------------------------------------\n");

  return lRet;
}

void InitCommunication(CIFXHANDLE * driver, CIFXHANDLE * channel)
{
  // 2020.6.1
  struct CIFX_LINUX_INIT init = {
      .init_options = CIFX_DRIVER_INIT_AUTOSCAN,
      .iCardNumber = 0,
      .fEnableCardLocking = 0,
      .base_dir = NULL,
      .poll_interval = 0, // non-interrupt 0:500ms
      .poll_StackSize = 0, /* set to 0 to use default */
      .trace_level = 255, /* form log */
      .user_card_cnt = 0,
      .user_cards = NULL,
  };

#ifdef DEBUG
  printf("%s() called\n", __FUNCTION__);
#endif

  /* First of all initialize toolkit */
  int32_t lRet = cifXDriverInit(&init);

  if(CIFX_NO_ERROR == lRet)
  {
    /* Display version of cifXRTXDrv and cifXToolkit */
    DisplayDriverInformation();

    /* Demonstrate the board/channel enumeration */
    EnumBoardDemo();

    /* Demonstrate system channel functionality */
    SysdeviceDemo();

    /* Demonstrate communication channel functionality */
    ChannelDemo(driver, channel);
  }
}
