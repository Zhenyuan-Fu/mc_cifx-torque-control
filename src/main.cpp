#include "cifXEndianess.h"
#include "cifxlinux.h"

#include "Hil_Packet.h"
#include "Hil_SystemCmd.h"

#include "utils.h"

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

#include <array>
#include <tuple>

extern "C"
{
#include "CalculateProg.h"
#include "Display.h"
#include "IOAPPzm.h"
#include "InitCommunication.h"
#include "SDOApp.h"
#include "cifXUser.h"
}

#include <mc_control/mc_global_controller.h>
#include <mc_rtc/version.h>

using TableJointData_t = std::array<std::tuple<std::string, double, double, double, double>, MOT_ID>;
using TableForceData_t =
    std::array<std::tuple<std::string, double, double, double, double, double, double>, ForceSensor_COUNT>;

CIFXHANDLE hDriver = nullptr;
CIFXHANDLE hChannel = nullptr;
mc_control::MCGlobalController * controller = nullptr;
std::vector<double> currents;
std::vector<double> torques;
std::vector<double> encoders;
std::vector<double> velocities;
std::vector<double> encoders_command;
// !! Index to force sensor name
std::vector<std::string> forceSensors = {"LeftFootForceSensor", "RightFootForceSensor","LegSensor"};
std::map<std::string, sva::ForceVecd> wrenches;
// Safety parameter: if the difference between the command and the encoder exceeds this, servo-off
static constexpr double JOINT_MAX_ERROR = 8; // degree
static constexpr size_t JOINT_MAX_ERROR_COUNT = 50;
static size_t ERROR_COUNT[MOT_ID] = {0};

static std::chrono::duration<double, std::milli> loop_dt{0};

std::atomic<bool> controllerReady{false};

void updateWrenches()
{
  for(size_t i = 0; i < ForceSensor_COUNT; ++i)
  {
    auto & wrench = wrenches[forceSensors[i]];
    const auto & fs = FS_Recive[i];
    const auto & init = InitFS_Recive[i];
    wrench.force().x() = convert(fs.FX) - convert(init.FX);
    wrench.force().y() = convert(fs.FY) - convert(init.FY);
    wrench.force().z() = convert(fs.FZ) - convert(init.FZ);
    wrench.couple().x() = convert(fs.TX) - convert(init.TX);
    wrench.couple().y() = convert(fs.TY) - convert(init.TY);
    wrench.couple().z() = convert(fs.TZ) - convert(init.TZ);
  }
}

inline double rad2deg(double rad)
{
  return rad * 180 / M_PI;
}

typedef struct SYNC_CALLBACK_DATAtag
{
  uint8_t bSyncHSMode;
  CIFXHANDLE hDevice;
} SYNC_CALLBACK_DATA;

SYNC_CALLBACK_DATA tSynchData;

unsigned char StopFlg = 0; //结束标志

uint16_t ulWriteErr = 0;
uint32_t ulReadReq = 0;
uint16_t ulReadErr = 0;

// 解除注册
void ECatMasterDev_UnRegisterInEvent(CIFXHANDLE hChannel)
{
  int32_t lRet = 0;

  lRet = xChannelUnregisterNotification(hChannel, CIFX_NOTIFY_PD0_IN);
  if(CIFX_NO_ERROR != lRet)
  {
    printf("Error while unregister input event callback function\r\n");
    ShowError(lRet);
  }
}
// RegisterInEvent 运行之后
//void APIENTRY PdoInEventCallback(uint32_t /*ulNotification*/, uint32_t /*ulDataLen*/, void * /*pvData*/, void * pvUser)
//{
//  static auto prev_loop_entry = std::chrono::high_resolution_clock::now();
//  auto now = std::chrono::high_resolution_clock::now();
//  loop_dt = now - prev_loop_entry;
//  prev_loop_entry = now;
//  CIFXHANDLE hChannel = static_cast<SYNC_CALLBACK_DATA *>(pvUser)->hDevice;
//
//  int32_t ErrorCode = Get_Mot_Data(hChannel);
//  if(ErrorCode != CIFX_NO_ERROR)
//  {
//    ulReadErr++;
//  }
//  if(!controllerReady)
//  {
//    mc_rtc::log::warning("Previous callback didn't finish running before this is called");
//    ulReadReq++;
//    return;
//  }
//  controllerReady = false;
//  for(size_t i = 0; i < MOT_ID; i++)
//  {
//    encoders[i] = Mot2JointPosition(MOT_Recive[i].Position, i);
//    velocities[i] = Mot2JointVelocity(MOT_Recive[i].Velocity, encoders[i], i);
//    currents[i] = MOT_Recive[i].Current;
//    torques[i] = MOT_Recive[i].Torque;
//  }
//  for(size_t i = MOT_ID; i < controller->robot().refJointOrder().size(); ++i)
//  {
//    encoders[i] = 0;
//    velocities[i] = 0;
//    currents[i] = 0;
//    torques[i] = 0;
//  }
//  updateWrenches();
//  controller->setEncoderValues(encoders);
//  controller->setEncoderVelocities(velocities);
//  controller->setWrenches(wrenches);
//  if(controller->run())
//  {
//    const auto & robot = controller->robot();
//    const auto & rjo = robot.refJointOrder();
//    for(uint32_t i = 0; i < MOT_ID; ++i)
//    {
//      const auto & j = rjo[i];
//      auto jCommand = robot.mbc().q[robot.jointIndexByName(j)][0];
//      double error = rad2deg(jCommand - encoders[i]);
//      if(fabs(error) > JOINT_MAX_ERROR)
//      {
//        ERROR_COUNT[i]++;
//        if(ERROR_COUNT[i] >= JOINT_MAX_ERROR_COUNT)
//        {
//          mc_rtc::log::critical(
//              "Joint error on {} exceed 8 degrees, servo OFF for safety (command: {:0.2f}, actual: {:0.2f})", j,
//              rad2deg(jCommand), rad2deg(encoders[i]));
//          controller->running = false;
//        }
//      }
//      else
//      {
//        ERROR_COUNT[i] = 0;
//      }
//      auto motor = Joint2Mot(jCommand, i);
//      encoders_command[i] = motor;
//      MOT_Send[i].Position = motor;
//      Set_Position(i, motor);
//    }
//  }
//
//  if(controller->running)
//  {
//    ErrorCode = IO_WriteZM(hChannel);
//    if(ErrorCode != CIFX_NO_ERROR)
//    {
//      ulWriteErr++;
//    }
//  }
//  else
//  {
//    Power_OFF_AllMoter(hChannel, MOT_ID);
//    StopFlg = 1;
//  }
//  ulReadReq++;
//  controllerReady = true;
//}

void APIENTRY PdoInEventCallback(uint32_t /*ulNotification*/, uint32_t /*ulDataLen*/, void * /*pvData*/, void * pvUser)
{
  static auto prev_loop_entry = std::chrono::high_resolution_clock::now();
  auto now = std::chrono::high_resolution_clock::now();
  loop_dt = now - prev_loop_entry;
  prev_loop_entry = now;
  CIFXHANDLE hChannel = static_cast<SYNC_CALLBACK_DATA *>(pvUser)->hDevice;

  int32_t ErrorCode = Get_Mot_Data(hChannel);
  if(ErrorCode != CIFX_NO_ERROR)
  {
    ulReadErr++;
  }
  if(!controllerReady)
  {
    mc_rtc::log::warning("Previous callback didn't finish running before this is called");
    ulReadReq++;
    return;
  }
  controllerReady = false;
  for(size_t i = 0; i < MOT_ID; i++)
  {
    encoders[i] = Mot2JointPosition(MOT_Recive[i].Position, i);
    velocities[i] = Mot2JointVelocity(MOT_Recive[i].Velocity, encoders[i], i);
    currents[i] = MOT_Recive[i].Current;
    torques[i] = MOT_Recive[i].Torque;
  }
  for(size_t i = MOT_ID; i < controller->robot().refJointOrder().size(); ++i)
  {
    encoders[i] = 0;
    velocities[i] = 0;
    currents[i] = 0;
    torques[i] = 0;
  }
  updateWrenches();
  controller->setEncoderValues(encoders);
  controller->setEncoderVelocities(velocities);
  controller->setWrenches(wrenches);
  if(controller->run())
  {
    const auto & robot = controller->robot();
    const auto & rjo = robot.refJointOrder();
    for(uint32_t i = 0; i < MOT_ID; ++i)
    {
      const auto & j = rjo[i];
      auto jCommand = robot.mbc().jointTorque[robot.jointIndexByName(j)][0];
      double error = rad2deg(jCommand - encoders[i]);
      if(fabs(error) > JOINT_MAX_ERROR)
      {
        ERROR_COUNT[i]++;
        if(ERROR_COUNT[i] >= JOINT_MAX_ERROR_COUNT)
        {
          mc_rtc::log::critical(
              "Joint error on {} exceed 8 degrees, servo OFF for safety (command: {:0.2f}, actual: {:0.2f})", j,
              rad2deg(jCommand), rad2deg(encoders[i]));
          controller->running = false;
        }
      }
      else
      {
        ERROR_COUNT[i] = 0;
      }
      auto motor = JointTorque2MotorCurrent(jCommand, i);
      encoders_command[i] = motor;
      MOT_Send[i].Torque = motor;
      Set_Torque(i, motor);
    }
  }

  if(controller->running)
  {
    ErrorCode = IO_WriteZM(hChannel);
    if(ErrorCode != CIFX_NO_ERROR)
    {
      ulWriteErr++;
    }
  }
  else
  {
    Power_OFF_AllMoter(hChannel, MOT_ID);
    StopFlg = 1;
  }
  ulReadReq++;
  controllerReady = true;
}

void PrintStatistic()
{
  printf("Read Req:  %8d\n", ulReadReq);
  printf("Read Err:  %8d\n", ulReadErr);
  printf("Write Err: %8d\n", ulWriteErr);
}

/*! Function to demonstrate event handling
 *   \param  hDriver  Handle to cifX driver
 *   \return CIFX_NO_ERROR on success
 */
void RegisterInEvent(CIFXHANDLE hChannel)
{
  int32_t lRet = CIFX_NO_ERROR;
  printf("\n--- Event handling demo ---\r\n");
  // Open channel
  if(lRet != CIFX_NO_ERROR)
  {
    // Read driver error description
    ShowError(lRet);
  }
  else
  {
    uint32_t ulState = 0;
    tSynchData = {0, hChannel};
    if((CIFX_NO_ERROR
        != (lRet =
                xChannelRegisterNotification(hChannel, CIFX_NOTIFY_PD0_IN, PdoInEventCallback, (void *)&tSynchData))))
    {
      // Failed to register one of the events */
      // Read driver error description
      printf("Error while register input event callback function\r\n");
      ShowError(lRet);
    }
    else
    {
      /* Get actual host state */
      if((lRet = xChannelHostState(hChannel, CIFX_HOST_STATE_READ, &ulState, 0L)) != CIFX_NO_ERROR)
      {
        // Read driver error description
        ShowError(lRet);
      }

      /* Set host ready */
      if((lRet = xChannelHostState(hChannel, CIFX_HOST_STATE_READY, NULL, 2000L)) != CIFX_NO_ERROR)
      {
        // Read driver error description
        ShowError(lRet);
      }
    }
  }
}

// 当前位置设为零点
void ZeroPosition(CIFXHANDLE hChannel, uint32_t Mot_NumTemp)
{
  for(uint32_t i = 0; i < Mot_NumTemp; i++)
  {
    ZeroClearSDOD(hChannel, i);
    usleep(500000);
  }
}

int main()
{
  if(mc_rtc::version() != mc_rtc::MC_RTC_VERSION)
  {
    mc_rtc::log::critical(
        "mc_rtc runtime version ({}) does not match compile version ({}), recompile mc_cifx before continuing",
        mc_rtc::version(), mc_rtc::MC_RTC_VERSION);
    return 1;
  }

  /* Lock memory */
  if(mlockall(MCL_CURRENT | MCL_FUTURE) == -1)
  {
    printf("mlockall failed: %m\n");
    if(errno == ENOMEM)
    {
      printf("\nIt is likely your user does not have enough memory limits, you can change the limits by adding the "
             "following line to /etc/security/limits.conf:\n\n");
      printf("%s - memlock unlimited\n\n", getlogin());
      printf("Then log-in and log-out\n");
    }
    return -2;
  }

  /* Set high priority */
  if(!setProcessHighPiority())
  {
    mc_rtc::log::critical("Failed to set process scheduling policy");
    return -2;
  }

  controller = new mc_control::MCGlobalController();
  controller->running = true;
  encoders.resize(controller->robot().refJointOrder().size());
  encoders_command.resize(controller->robot().refJointOrder().size());
  velocities.resize(controller->robot().refJointOrder().size());
  currents.resize(controller->robot().refJointOrder().size());
  torques.resize(controller->robot().refJointOrder().size());
  controller->controller().logger().addLogEntry("perf_LoopDt", [&]() { return loop_dt.count(); });
  controller->controller().logger().addLogEntry("currentIn", [&]() -> const std::vector<double> & { return currents; });
  controller->controller().logger().addLogEntry("torqueIn", [&]() -> const std::vector<double> & { return torques; });
  controller->controller().logger().addLogEntry("motorOut", [&]() -> const std::vector<double> & { return encoders_command; });

  uint32_t i;
  InitCommunication(&hDriver, &hChannel);
  SDODInit_All(hChannel, MOT_ID);

  init_Motor(hChannel);
  ZeroPosition(hChannel, MOT_ID);// TODO COMMENT THIS
  Get_Mot_Data(hChannel);
  Get_Mot_Data(hChannel);

  Power_OFF_AllMoter(hChannel, MOT_ID);

  for(i = 0; i < MOT_ID; i++)
  {
    Set_Mode_CST(i);
  }
  Servo_On_All(hChannel, MOT_ID);
  Get_Mot_Data(hChannel);
  Get_Mot_Data(hChannel);

  init_ForceSensor(hChannel);

  std::vector<double> initq;
  for(size_t i = 0; i < MOT_ID; ++i)
  {
    initq.push_back(Mot2JointPosition(MOT_Recive[i].Position, i));
    ERROR_COUNT[i] = 0;
  }
  for(size_t i = MOT_ID; i < controller->robot().refJointOrder().size(); ++i)
  {
    initq.push_back(0);
  }
  std::cout << "Initialize mc_rtc with configuration:\n";
  for(const auto & qi : initq)
  {
    std::cout << qi << ", ";
  }
  std::cout << "\n";
  controller->setEncoderValues(initq);
  controller->init(initq);
  mc_rtc::log::info("Run first iteration outside real-time");
  bool ret = controller->run();
  mc_rtc::log::info("First iteration status: {}", ret ? "OK" : "FAIL");
  controllerReady = true;
  controller->controller().gui()->addElement({},
                                             mc_rtc::gui::Button("Servo off", [&]() { controller->running = false; }));
  controller->controller().gui()->addElement(
      {"Robot state"},
      mc_rtc::gui::Table("Status", {"Joint", "Motor command", "Encoder", "Command", "Speed"}, {"{}", "{:0.2f}", "{:0.2f}", "{:0.2f}", "{:0.2f}"},
                         [&]() -> const TableJointData_t & {
                           static TableJointData_t data;
                           const auto & ctl = controller->controller();
                           const auto & robot = ctl.robot();
                           for(size_t i = 0; i < MOT_ID; ++i)
                           {
                             const auto & j = ctl.robot().refJointOrder()[i];
                             auto jIndex = robot.jointIndexByName(j);
                             auto command = robot.mbc().q[jIndex][0];
                             data[i] = {j, encoders_command[i], rad2deg(encoders[i]), rad2deg(command), rad2deg(velocities[i])};
                           }
                           return data;
                         }),
      mc_rtc::gui::Table("Force", {"Sensor", "FX", "FY", "FZ", "TX", "TY", "TZ"},
                         {"{}", "{:0.2f}", "{:0.2f}", "{:0.2f}", "{:0.2f}", "{:0.2f}", "{:0.2f}"},
                         [&]() -> const TableForceData_t & {
                           static TableForceData_t data;
                           for(size_t i = 0; i < ForceSensor_COUNT; ++i)
                           {
                             const auto & w = wrenches[forceSensors[i]];
                             const auto & f = w.force();
                             const auto & t = w.couple();
                             data[i] = {forceSensors[i], f.x(), f.y(), f.z(), t.x(), t.y(), t.z()};
                           }
                           return data;
                         }));
  StopFlg = 0;

  ulWriteErr = 0;
  ulReadReq = 0;
  ulReadErr = 0;
  /* Demonstrate event handling */
  RegisterInEvent(hChannel);
  /* Set Sync mode */
  ECatMasterDev_SetSyncMode(hChannel);

  while(1)
  {
    if(StopFlg == 1)
    {
      ECatMasterDev_UnRegisterInEvent(hChannel);
      printf("\n");
      printf("------------------------------------------------------------------\n");
      PrintStatistic();
      // error statistics
      ReadMulMotoMF(hChannel);
      ReadMulMotoEC(hChannel);
      ReadMulMotoEE(hChannel);

      xChannelClose(hChannel);
      xDriverClose(hDriver);

      cifXDriverDeinit();
      return 0;
    }
    else
      usleep(1000);
  }
  return 0;
}
