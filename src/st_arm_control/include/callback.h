#ifndef CALLBACK_H
#define CALLBACK_H

#include <iostream>
#include <unistd.h>
#include "spi2can.h"
#include "rt_utils.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "dynamics.h"
#include "motor_controller.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/PoseStamped.h"
#include "dynamixel.h"

class Callback
{
public:
  Callback();

  void HMDTFCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void joint_states_callback(const sensor_msgs::JointState::ConstPtr &msg);
  void SwitchMode(const std_msgs::Int32ConstPtr &msg);
  void SwitchGainTaskSpaceP(const std_msgs::Float32MultiArrayConstPtr &msg);
  void SwitchGainTaskSpaceD(const std_msgs::Float32MultiArrayConstPtr &msg);
  void SwitchGainTaskSpaceW(const std_msgs::Float32MultiArrayConstPtr &msg);
  void SwitchGainP(const std_msgs::Float32MultiArrayConstPtr &msg);
  void SwitchGainD(const std_msgs::Float32MultiArrayConstPtr &msg);
  void SwitchGainR(const std_msgs::Float32MultiArrayConstPtr &msg);
  void InitializePose(const std_msgs::BoolConstPtr &msg);
  void GripperCallback(const std_msgs::Float32ConstPtr &msg);

private:

};

#endif // CALLBACK_H