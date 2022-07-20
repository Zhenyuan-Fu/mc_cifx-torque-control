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
#include <mc_filter/LowPass.h>
#include <mc_rtc/version.h>

using TableJointData_t = std::array<std::tuple<std::string, double, double, double, double, double>, MOT_ID>;
using TableForceData_t =
    std::array<std::tuple<std::string, double, double, double, double, double, double>, ForceSensor_COUNT>;
using TableIMUData_t =
    std::array<std::tuple<std::string, double, double, double, double, double, double, double, double, double>,
               IMU_COUNT>;

CIFXHANDLE hDriver = nullptr;
CIFXHANDLE hChannel = nullptr;
mc_control::MCGlobalController * controller = nullptr;
std::vector<double> currents;
std::vector<double> torques;
std::vector<double> encoders;
std::vector<double> velocities;
std::vector<double> encoders_command;
std::vector<double> torques_command;
// !! Index to IMU name
std::vector<std::string> bodySensors = {"Accelerometer"};
mc_control::MCGlobalController::QuaternionMap bodySensorsOrientation;
std::map<std::string, Eigen::Vector3d> bodySensorsAngularVelocity;
std::map<std::string, Eigen::Vector3d> bodySensorsLinearAcceleration;
// !! Index to force sensor name
// std::vector<std::string> forceSensors = {"LeftFootForceSensor", "RightFootForceSensor","LegSensor"};
std::vector<std::string> forceSensors = {"LeftFootForceSensor", "RightFootForceSensor"};
struct ForceFilter : public mc_filter::LowPass<sva::ForceVecd>
{
  static constexpr double dt = 0.003;
  static constexpr double period = 0.03;
  ForceFilter() : mc_filter::LowPass<sva::ForceVecd>(dt, period) {}
};
std::array<ForceFilter, ForceSensor_COUNT> forceFilters;
std::map<std::string, sva::ForceVecd> wrenches;
// arm
// joints kp kd setting (all same now)
//double joints_kp = 220;
//double joints_kd = 14;
// ankle
double joints_kp = 100;
double joints_kd = 5.;

// Safety parameter: if the difference between the command and the encoder exceeds this, servo-off
static constexpr double JOINT_MAX_ERROR = 10; // degree
static constexpr size_t JOINT_MAX_ERROR_COUNT = 50;
static size_t ERROR_COUNT[MOT_ID] = {0};

std::atomic<bool> zero_imu{false};
std::atomic<bool> transpose_imu_raw_reading{false};
std::atomic<bool> use_imu_offset{true};
std::atomic<bool> use_imu_offset_right{false};
Eigen::Matrix3d imu_origin = Eigen::Matrix3d::Identity();
Eigen::Matrix3d imu_reading = Eigen::Matrix3d::Identity();
Eigen::Vector3d imu_offset{M_PI, 0, 0};
Eigen::Vector3d imu_multiplier{1.0, 1.0, 1.0};

Eigen::Vector3d imu_vel_offset = Eigen::Vector3d::Zero();
Eigen::Vector3d imu_acc_offset = Eigen::Vector3d::Zero();

static std::chrono::duration<double, std::milli> loop_dt{0};

std::atomic<bool> controllerReady{false};

void convert(const ForceSensor & fs, sva::ForceVecd & out)
{
  out.force().x() = convert(fs.FX);
  out.force().y() = convert(fs.FY);
  out.force().z() = convert(fs.FZ);
  out.couple().x() = convert(fs.TX);
  out.couple().y() = convert(fs.TY);
  out.couple().z() = convert(fs.TZ);
}

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

void updateBodySensors()
{
  Eigen::Vector3d rpy;
  for(size_t i = 0; i < IMU_COUNT; ++i)
  {
    const auto & bs = bodySensors[i];
    const auto & IMU = IMU_Recive[i];
    // Orientation
    {
      auto & ori = bodySensorsOrientation[bs];
      rpy << deg2rad(convert(IMU.dwRoll)), deg2rad(convert(IMU.dwPitch)), deg2rad(convert(IMU.dwYaw));
      rpy.x() *= imu_multiplier.x();
      rpy.y() *= imu_multiplier.y();
      rpy.z() *= imu_multiplier.z();
      Eigen::Matrix3d ori_mat = mc_rbdyn::rpyToMat(rpy);
      if(transpose_imu_raw_reading)
      {
        ori_mat = ori_mat.transpose();
      }
      if(use_imu_offset)
      {
        if(use_imu_offset_right)
        {
          ori_mat = ori_mat * mc_rbdyn::rpyToMat(imu_offset);
        }
        else
        {
          ori_mat = mc_rbdyn::rpyToMat(imu_offset) * ori_mat;
        }
      }
      if(zero_imu)
      {
        zero_imu = false;
        //imu_origin = ori_mat.transpose();
      }
      ori_mat = imu_origin * ori_mat;
      imu_reading = ori_mat;
      ori = Eigen::Quaterniond(ori_mat).normalized();
      // //printf("%08X;\n", IMU.dwYaw);
      // printf("\nroll:%f\n",convert(IMU.dwRoll));
      // printf("pitch:%f\n",convert(IMU.dwPitch));
      // printf("yaw:%f\n",convert(IMU.dwYaw));
    }
    // Angulary velocity
    {
      auto & vel = bodySensorsAngularVelocity[bs];
      vel << convert(IMU.dwWx), convert(IMU.dwWy), convert(IMU.dwWz);
      vel = imu_origin * (vel - imu_vel_offset);
    }
    // Liner acceleration
    {
      auto & acc = bodySensorsLinearAcceleration[bs];
      acc << convert(IMU.dwAx), convert(IMU.dwAy), convert(IMU.dwAz);
      acc = imu_origin * (acc - imu_acc_offset);
      // printf("a_z:%f",convert(IMU.dwAz));
    }
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

double jointPD(double q_ref, double q, double qdot_ref, double qdot)
{
  double p_error = q_ref -q;
  double v_error = qdot_ref - qdot;
  double ret = (joints_kp * p_error + joints_kd * v_error);
  return ret;
}

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

  // TODO care
//  for(uint32_t i = 0; i < MOT_ID; i++)
//  {
//    Set_Mode_CST(i);
//    std::cout << "change the Mode" << std::endl;
//  }

  updateBodySensors();
  updateWrenches();
  controller->setEncoderValues(encoders);
  controller->setEncoderVelocities(velocities);
  controller->setWrenches(wrenches);
  controller->setSensorOrientations(bodySensorsOrientation);
  controller->setSensorAngularVelocities(bodySensorsAngularVelocity);
  controller->setSensorLinearAccelerations(bodySensorsLinearAcceleration);

  if(controller->run())
  {
    const auto & robot = controller->robot();
    const auto & rjo = robot.refJointOrder();
    for(uint32_t i = 0; i < MOT_ID; ++i)
    {
      const auto & j = rjo[i];
      auto cifx_next_ctrl_q = robot.mbc().q[robot.jointIndexByName(j)][0];
      auto cifx_next_ctrl_alpha = robot.mbc().alpha[robot.jointIndexByName(j)][0];
      auto cifx_next_ctrl_torque = robot.mbc().jointTorque[robot.jointIndexByName(j)][0];

      double q_ref = cifx_next_ctrl_q;
      double alpha_ref = cifx_next_ctrl_alpha;

      double jCommand  = cifx_next_ctrl_torque + jointPD(q_ref, encoders[i], alpha_ref, velocities[i]);

//      if(jCommand > 150.0){
//        jCommand = 150;
//      }else if(jCommand < -150){
//        jCommand = -150;
//      }

      double error = rad2deg(cifx_next_ctrl_q - encoders[i]);

      if(fabs(error) > JOINT_MAX_ERROR)
      {
        ERROR_COUNT[i]++;
        if(ERROR_COUNT[i] >= JOINT_MAX_ERROR_COUNT)
        {
          mc_rtc::log::critical(
              "Joint error on {} exceed 8 degrees, servo OFF for safety (command: {:0.2f}, actual: {:0.2f})", j,
              rad2deg(q_ref), rad2deg(encoders[i]));
          controller->running = false;
        }
      }
      else
      {
        ERROR_COUNT[i] = 0;
      }
      auto motor = JointTorque2MotorCurrent(jCommand, i);
      encoders_command[i] = motor;
      torques_command[i] = jCommand;

      if(motor > 2000.0){
        motor = 2000;
      }else if(motor < -2000){
        motor = -2000;
      }

      MOT_Send[i].Torque = motor;
      Set_Torque(i, motor);
      //
      //if (MOT_Recive[i].ModeDis == ProPositionMode){
//        std::cout << "MOT Mode = " << MOT_Recive[i].ModeDis << std::endl;
      //}
    }
  }

  if(controller->running)
  {
    ErrorCode = IO_WriteZM(hChannel);
//    std::cout << "MOT_Send[i].Mode == ProPositionMode" << MOT_Send[0].Mode << std::endl;
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

void init_ForceSensor_Filters()
{
  sva::ForceVecd tmp;
  for(size_t i = 0; i < ForceSensor_COUNT; ++i)
  {
    convert(InitFS_Recive[i], tmp);
    forceFilters[i].reset(tmp);
  }
}

void init_IMU_offset(CIFXHANDLE hChannel)
{
  size_t measures = 300;
  Eigen::Vector3d imu_vel;
  Eigen::Vector3d imu_vel_sum;
  Eigen::Vector3d imu_acc;
  Eigen::Vector3d imu_acc_sum;
  for(size_t i = 0; i < measures; ++i)
  {
    Get_Mot_Data(hChannel);
    const auto & IMU = IMU_Recive[0];
    imu_vel << convert(IMU.dwWx), convert(IMU.dwWy), convert(IMU.dwWz);
    imu_vel_sum += imu_vel;
    imu_acc << convert(IMU.dwAx), convert(IMU.dwAy), convert(IMU.dwAz);
    imu_acc_sum += imu_acc;
  }
  imu_vel_offset = imu_vel_sum / measures;
  imu_acc_offset = imu_acc_sum / measures;
  mc_rtc::log::info("imu velocity offset: {}", imu_vel_offset.transpose());
  mc_rtc::log::info("imu acceleration offset: {}", imu_acc_offset.transpose());
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
  torques_command.resize(controller->robot().refJointOrder().size());
  velocities.resize(controller->robot().refJointOrder().size());
  currents.resize(controller->robot().refJointOrder().size());
  torques.resize(controller->robot().refJointOrder().size());
  controller->controller().logger().addLogEntry("perf_LoopDt", [&]() { return loop_dt.count(); });
  controller->controller().logger().addLogEntry("currentIn", [&]() -> const std::vector<double> & { return currents; });
  controller->controller().logger().addLogEntry("torqueIn", [&]() -> const std::vector<double> & { return torques; });
  controller->controller().logger().addLogEntry("motorOut", [&]() -> const std::vector<double> & { return encoders_command; });
  controller->controller().logger().addLogEntry("torqueCmdOut", [&]() -> const std::vector<double> & { return torques_command; });

  //
  controller->controller().logger().addLogEntry("IMU_roll", []() { return IMU_Recive[0].dwRoll; });
  controller->controller().logger().addLogEntry("IMU_pitch", []() { return IMU_Recive[0].dwPitch; });
  controller->controller().logger().addLogEntry("IMU_yaw", []() { return IMU_Recive[0].dwYaw; });
  controller->controller().logger().addLogEntry("IMU_Ax", []() { return IMU_Recive[0].dwAx; });
  controller->controller().logger().addLogEntry("IMU_Ay", []() { return IMU_Recive[0].dwAy; });
  controller->controller().logger().addLogEntry("IMU_Az", []() { return IMU_Recive[0].dwAz; });
  controller->controller().logger().addLogEntry("IMU_Wx", []() { return IMU_Recive[0].dwWx; });
  controller->controller().logger().addLogEntry("IMU_Wy", []() { return IMU_Recive[0].dwWy; });
  controller->controller().logger().addLogEntry("IMU_Wz", []() { return IMU_Recive[0].dwWz; });

  uint32_t i;
  InitCommunication(&hDriver, &hChannel);
  SDODInit_All(hChannel, MOT_ID);

  init_Motor(hChannel);

//  ZeroPosition(hChannel, MOT_ID);// TODO COMMENT THIS
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
  init_ForceSensor_Filters();

  init_IMU_offset(hChannel);

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
      mc_rtc::gui::Table("Status", {"Joint", "Torque Command", "PD_Torque Command", "Encoder", "Command", "Speed"}, {"{}", "{:0.2f}", "{:0.2f}", "{:0.2f}", "{:0.2f}", "{:0.2f}"},
                         [&]() -> const TableJointData_t & {
                           static TableJointData_t data;
                           const auto & ctl = controller->controller();
                           const auto & robot = ctl.robot();
                           for(size_t i = 0; i < MOT_ID; ++i)
                           {
                             const auto & j = ctl.robot().refJointOrder()[i];
                             auto jIndex = robot.jointIndexByName(j);
                             auto joint_torque = robot.mbc().jointTorque[jIndex][0];
                             auto pd_torque_command = torques_command[i];
                             auto command = robot.mbc().q[jIndex][0];
                             data[i] = {j, joint_torque,  pd_torque_command, rad2deg(encoders[i]), rad2deg(command), rad2deg(velocities[i])};
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
                         }),
      mc_rtc::gui::Table(
          "IMU", {"Sensor", "R", "P", "Y", "WX", "WY", "WZ", "AX", "AY", "AZ"},
          {"{}", "{:0.2f}", "{:0.2f}", "{:0.2f}", "{:0.2f}", "{:0.2f}", "{:0.2f}", "{:0.2f}", "{:0.2f}", "{:0.2f}"},
          [&]() -> const TableIMUData_t & {
            static TableIMUData_t data;
            for(size_t i = 0; i < IMU_COUNT; ++i)
            {
              const auto & bs = bodySensors[i];
              const auto & ori = mc_rbdyn::rpyFromQuat(bodySensorsOrientation[bs]);
              const auto & w = bodySensorsAngularVelocity[bs];
              const auto & acc = bodySensorsLinearAcceleration[bs];
              data[i] = {bs, ori.x(), ori.y(), ori.z(), w.x(), w.y(), w.z(), acc.x(), acc.y(), acc.z()};
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
