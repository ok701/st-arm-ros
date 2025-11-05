#ifndef DYNAMIXEL_H
#define DYNAMIXEL_H

#include <eigen3/Eigen/Dense>
#include <vector>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include <unordered_map>




#define TORQUE_TO_VALUE   217.39     // XM430-W350
#define RAD_TO_VALUE      651.8981    // 1 rev = 4096

#define PROTOCOL_VERSION      2.0 

// #define DXL_ID                1
#define BAUDRATE              1000000
#define DEVICE_NAME           "/dev/ttyUSB0"

using Eigen::VectorXd;

/// Dynamixel types
enum DynamixelSeriesType
{
  kSeriesAX = 0,
  kSeriesRX = 1,
  kSeriesDX = 2,
  kSeriesEX = 3,
  kSeriesLegacyMX = 4,
  kSeriesMX = 5,
  kSeriesX = 6,
  kSeriesLegacyPro = 7,
  kSeriesP = 8,
  kSeriesUnknown = 9
};

/// Standard control table, for newer models of dynamixels supporting protocol 2.0:
/// - X (2.0)
enum DynamixelStandardRegisterTable
{
  // EEPROM
  kRegStandard_ModelNumber = 0,
  kRegStandard_ModelInfo = 2,
  kRegStandard_FirmwareVersion = 6,
  kRegStandard_ID = 7,
  kRegStandard_BaudRate = 8,
  kRegStandard_ReturnDelayTime = 9,
  kRegStandard_DriveMode = 10,        // !
  kRegStandard_OperatingMode = 11,
  kRegStandard_ShadowID = 12,
  kRegStandard_ProtocolVersion = 13,
  kRegStandard_HomingOffset = 20,
  kRegStandard_MovingThreshold = 24,
  kRegStandard_TemperatureLimit = 31,
  kRegStandard_MaxVoltageLimit = 32,
  kRegStandard_MinVoltageLimit = 34,
  kRegStandard_PWMLimit = 36,
  kRegStandard_CurrentLimit = 38,
  kRegStandard_AccelerationLimit = 40,
  kRegStandard_VelocityLimit = 44,
  kRegStandard_MaxPositionLimit = 48,
  kRegStandard_MinPositionLimit = 52, 
  kRegStandard_DataPort1Mode = 56,
  kRegStandard_DataPort2Mode = 57,
  kRegStandard_DataPort3Mode = 58,
  kRegStandard_Shutdown = 63,

  // RAM
  kRegStandard_TorqueEnable = 64,
  kRegStandard_LED = 65,
  kRegStandard_StatusReturnLevel = 68,
  kRegStandard_RegisteredInstruction = 69,
  kRegStandard_HardwareErrorStatus = 70,
  kRegStandard_VelocityIGain = 76,
  kRegStandard_VelocityPGain = 78,
  kRegStandard_PositionDGain = 80,
  kRegStandard_PositionIGain = 82,
  kRegStandard_PositionPGain = 84,
  kRegStandard_Feedforward2ndGain = 88,
  kRegStandard_Feedforward1stGain = 90,
  kRegStandard_BusWatchdog = 98,
  kRegStandard_GoalPWM = 100,
  kRegStandard_GoalCurrent = 102,
  kRegStandard_GoalVelocity = 104,
  kRegStandard_ProfileAcceleration = 108,
  kRegStandard_ProfileVelocity = 112,
  kRegStandard_GoalPosition = 116,
  kRegStandard_RealtimeTick = 120,
  kRegStandard_Moving = 122,
  kRegStandard_MovingStatus = 123,
  kRegStandard_PresentPWM = 124,
  kRegStandard_PresentCurrent = 126,
  kRegStandard_PresentVelocity = 128,
  kRegStandard_PresentPosition = 132,
  kRegStandard_VelocityTrajectory = 136,
  kRegStandard_PositionTrajectory = 140,
  kRegStandard_PresentInputVoltage = 144,
  kRegStandard_PresentTemperature = 146,
  kRegStandard_DataPort1 = 152,
  kRegStandard_DataPort2 = 154,
  kRegStandard_DataPort3 = 156,
  kRegStandard_IndirectAddress1 = 168,
  kRegStandard_IndirectData1 = 224,
};

/// Struct that describes the dynamixel motor's static and physical properties
struct DynamixelSpec
{
  std::string name;              ///< The Model Name
  uint16_t model_number;         ///< Model number (e.g 29 = MX-28)
  DynamixelSeriesType type;      ///< Model type (e.g MX, AX, Pro)
  bool external_ports;           ///< If this model has data ports
  int encoder_cpr;               ///< Motor encoder counts per revolution
  double encoder_range_deg;      ///< Motor encoder range in degrees
  double velocity_radps_to_reg;  ///< Conversion factor from velocity in radians/sec to register counts
  double effort_reg_max;         ///< Max possible value for effort register
  double effort_reg_to_mA;       ///< Conversion factor from register values to current in mA
};


class Dynamixel{
 public:
  Dynamixel();
  ~Dynamixel();

  void SetTorqueRef(VectorXd);
  VectorXd GetTorqueAct();
  void SetThetaRef(VectorXd);
  VectorXd GetThetaAct();
  VectorXd GetThetaDot();
  VectorXd GetThetaDotEstimated();
  void Loop(bool RxTh, bool RxThDot, bool TxTorque);
  void CalculateEstimatedThetaDot(int);

 private:  
  dynamixel::PortHandler * portHandler;
  dynamixel::PacketHandler * packetHandler;

  const int dx_id[4] = {3, 4, 5, 6};
  // float zero_manual_offset_[4] = {0.2424, 0, 0, 0};
  float zero_manual_offset_[4] = {0, 0, 0, 0};
  uint32_t position[4] = {0, 0, 0, 0};
  uint32_t velocity[4] = {0, 0, 0, 0};
  int32_t ref_torque_value_[4] = {0, 0, 0, 0};

  VectorXd ref_th_value_ = VectorXd::Zero(4);
  VectorXd ref_th_ = VectorXd::Zero(4);
  VectorXd ref_th_dot_ = VectorXd::Zero(4);
  VectorXd ref_torque_ = VectorXd::Zero(4);
  VectorXd th_ = VectorXd::Zero(4);
  VectorXd th_last_ = VectorXd::Zero(4);
  VectorXd th_dot_ = VectorXd::Zero(4);
  VectorXd th_dot_est_ = VectorXd::Zero(4);
  VectorXd tau_ = VectorXd::Zero(4);

  void syncReadTheta();
  void syncReadThetaDot();
  void syncWriteTheta();
  void syncWriteTorque();
  void getParam(int32_t data, uint8_t *param);
  float convertValue2Radian(int32_t value);
  int32_t torqueToValue(double);
};

#endif  // DYNAMIXEL_H