#include "dynamixel.h"
#define PI                3.141592

Dynamixel::Dynamixel(){ //생성자
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME); //장치 이름(DEVICE_NAME)을 받아 포트를 설정
  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION); //패킷 받을 수 있도록 프로토콜 버전 기록 - 현재는 "2" RS485
  
  if (!portHandler->openPort()) ROS_ERROR("Failed to open the port!"); // 포트가 안 잡히면 포트 못 잡았다고 터미널 메세지 출력
  if (!portHandler->setBaudRate(BAUDRATE)) ROS_ERROR("Failed to set the baudrate!"); // 보드레이트가 안 잡히면 못 잡았다고 메세지 출력
  
  for(uint8_t i=0; i<4; i++) // 다이나믹셀 4개의 모터에 대해 통신 확인이 필요
  {
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dx_id[i], kRegStandard_OperatingMode, 0, &dxl_error);   // current_mode
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // write1ByteTxRx  (PortHandler *port, uint8_t id, uint16_t address, uint8_t data, uint8_t *error = 0) = 0
    // uint16_t address : 입력해준 주소로
    // uint8_t data : 데이터에 맞게 값을 입력하면
    // 0을 반환 -> COMM_SUCCESS = 0
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // kRegStandard_OperatingMode : 제어모드
    // 0 : 토크제어모드
    
    if (dxl_comm_result != COMM_SUCCESS) ROS_ERROR("Failed to set torque control mode for Dynamixel ID %d", i); // tx, rx 연결 실패 시
  }

  // for(uint8_t i=0;i<4;i++) packetHandler->write1ByteTxRx(portHandler, dx_id[i], kRegStandard_OperatingMode, 3, &dxl_error);   // position_mode

  for(uint8_t i=0; i<4; i++)
  {
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dx_id[i], kRegStandard_TorqueEnable, 1, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) ROS_ERROR("Failed to enable torque for Dynamixel ID %d", i);
  } 

  for(uint8_t i=0; i<4; i++)
  {
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dx_id[i], kRegStandard_LED, 1, &dxl_error);
    // kRegStandard_LED : LED ON
    if (dxl_comm_result != COMM_SUCCESS) ROS_ERROR("Failed to enable LED for Dynamixel ID %d", i);
  } 
}


Dynamixel::~Dynamixel(){ // 
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  for(uint8_t i=0; i<4; i++)
  {
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dx_id[i], kRegStandard_TorqueEnable, 0, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) ROS_ERROR("Failed to disable torque for Dynamixel ID %d", i);
  } 

  for(uint8_t i=0; i<4; i++)
  {
    packetHandler->write1ByteTxRx(portHandler, dx_id[i], kRegStandard_LED, 0, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) ROS_ERROR("Failed to disable LED for Dynamixel ID %d", i);
  } 
  
  portHandler->closePort();
}


void Dynamixel::Loop(bool RxTh, bool RxThDot, bool TxTorque)
{
  if(RxTh) syncReadTheta();
  if(RxThDot) syncReadThetaDot();
  if(TxTorque) syncWriteTorque();
}


VectorXd Dynamixel::GetThetaAct()
{
  return th_;
}


void Dynamixel::syncReadTheta()
{
  dynamixel::GroupSyncRead groupSyncRead(portHandler, packetHandler, kRegStandard_PresentPosition, 4);
  for (uint8_t i=0; i < 4; i++) groupSyncRead.addParam(dx_id[i]);
  groupSyncRead.fastSyncReadTxRxPacket();
  for(uint8_t i=0; i < 4; i++) position[i] = groupSyncRead.getData(dx_id[i], kRegStandard_PresentPosition, 4);
  groupSyncRead.clearParam();
  for(uint8_t i=0; i < 4; i++) th_[i] = convertValue2Radian(position[i]) - PI + zero_manual_offset_[i];
}


VectorXd Dynamixel::GetThetaDot()
{
  VectorXd vel_(4);
  for(uint8_t i=0; i < 4; i++) 
  {
    if(velocity[i] > 4294900000) vel_[i] = (velocity[i] - 4294967295) * 0.003816667; //4,294,967,295 = 0xFFFFFFFF   // 1 = 0.229rpm   // 1 = 0.003816667
    else vel_[i] = velocity[i] * 0.003816667;
  }
  return vel_;
}


void Dynamixel::syncReadThetaDot()
{
  dynamixel::GroupSyncRead groupSyncReadThDot(portHandler, packetHandler, kRegStandard_PresentVelocity, 4);
  for (uint8_t i=0; i < 4; i++) groupSyncReadThDot.addParam(dx_id[i]);
  groupSyncReadThDot.fastSyncReadTxRxPacket();
  for(uint8_t i=0; i < 4; i++) velocity[i] = groupSyncReadThDot.getData(dx_id[i], kRegStandard_PresentVelocity, 4);
  groupSyncReadThDot.clearParam();
}


void Dynamixel::SetTorqueRef(VectorXd a_torque)
{
  for (uint8_t i=0; i < 4; i++) ref_torque_[i] = a_torque[i];
  // ref_torque_[3] = 0;
}


void Dynamixel::syncWriteTorque()
{
  dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, kRegStandard_GoalCurrent, 2);

  uint8_t parameter[4] = {0, 0, 0, 0};
  for (uint8_t i=0; i < 4; i++)
  {
    // ref_torque_value_[i] = ref_torque_[i] * TORQUE_TO_VALUE;
    ref_torque_value_[i] = torqueToValue(ref_torque_[i]);
    if(ref_torque_value_[i] > 1000) ref_torque_value_[i] = 1000;
    else if(ref_torque_value_[i] < -1000) ref_torque_value_[i] = -1000;
  }
  for (uint8_t i=0; i < 4; i++){
    getParam(ref_torque_value_[i], parameter);
    groupSyncWrite.addParam(dx_id[i], (uint8_t *)&parameter);
  }
  groupSyncWrite.txPacket();
  groupSyncWrite.clearParam();
  // std::cout << "torque value 1: " << ref_torque_value_[1] << std::endl;
}


// {XM430-W350} => effort to current  / 1.78*i-?    // current to value  / 2.69f
// {XM430-W350} => 500 = 1.345[A] = 2.3941[Nm] => 208.85 = 1[Nm]

// {XM430-W210} => if effort to current = 1.3*i      => 500 = 1.345[A] = 1.7485 Nm => 1[Nm] = 285
// {XM430-W210} => if effort to current = 1.3*i-0.32 => 500 = 1.345[A] = 1.4285 Nm => 1[Nm] = 350
int32_t Dynamixel::torqueToValue(double torque)
{
  int32_t value_ = int(torque * 285);     // XM430-W210
  // int32_t value_ = int(torque * 208.427);  // XM430-W350
  // if(value_ > 1) value_ += 10;       // friction compensation
  // else if(value_ < -1) value_ -= 10;
  return value_;
}


void Dynamixel::SetThetaRef(VectorXd theta)
{
  for (uint8_t i=0; i < 4; i++) ref_th_[i] = theta[i];
}


void Dynamixel::syncWriteTheta()
{
  dynamixel::GroupSyncWrite gSyncWriteTh(portHandler, packetHandler, kRegStandard_GoalPosition, 4);

  uint8_t parameter[4] = {0, 0, 0, 0};

  for (uint8_t i=0; i < 4; i++){
    ref_th_value_ = ref_th_ * RAD_TO_VALUE;
    getParam(ref_th_value_[i], parameter);
    gSyncWriteTh.addParam(dx_id[i], (uint8_t *)&parameter);
  }
  gSyncWriteTh.txPacket();
  gSyncWriteTh.clearParam();
}


void Dynamixel::getParam(int32_t data, uint8_t *param)
{
  param[0] = DXL_LOBYTE(DXL_LOWORD(data));
  param[1] = DXL_HIBYTE(DXL_LOWORD(data));
  param[2] = DXL_LOBYTE(DXL_HIWORD(data));
  param[3] = DXL_HIBYTE(DXL_HIWORD(data));
}


float Dynamixel::convertValue2Radian(int32_t value)
{
  float radian = value / RAD_TO_VALUE;
  return radian;
}


VectorXd Dynamixel::GetThetaDotEstimated()
{
  return th_dot_est_;
}


void Dynamixel::CalculateEstimatedThetaDot(int dt_us)
{
  th_dot_est_ = (th_last_ - th_) / (-dt_us * 0.00001);
  th_last_ = th_;
}