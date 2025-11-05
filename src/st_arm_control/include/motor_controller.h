#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "shared_memory.h"
#include "dynamics.h"
#include "dynamixel.h"

#define   tic2radL   2607.435432674516
#define   tic2radX   10430.21970545193
// #define   window_size 20
#define   window_size_rmd 20  //sma fillter window size RMD motor 
#define   window_size_dym 4   //sma fillter windwo size dynamixel motor
#define   num_of_motors 7
#define   num_of_motors_rmd 3
#define   num_of_motors_dym 4

using Eigen::MatrixXd;


class Motor_Controller{

public:   
  int count;
  bool first_loop = true;

  VectorXd th_joint = VectorXd::Zero(num_of_motors);
  VectorXd th_dot = VectorXd::Zero(num_of_motors);
  VectorXd th_dot_sma_filtered = VectorXd::Zero(num_of_motors);
  VectorXd th_dot_sma_filtered_rmd = VectorXd::Zero(num_of_motors_rmd);
  VectorXd th_dot_sma_filtered_dym = VectorXd::Zero(num_of_motors_dym);
  // MatrixXd sma = MatrixXd::Zero(window_size, num_of_motors);
  MatrixXd sma_rmd = MatrixXd::Zero(window_size_rmd, num_of_motors_rmd);  //sma fillter RMD motor
  MatrixXd sma_dym = MatrixXd::Zero(window_size_dym, num_of_motors_dym);  //sma fillter dynamixel motor


  VectorXd torque_wrist = VectorXd::Zero(4); 



  Motor_Controller();
  //~Motor_Controller();

  VectorXd GetThetaX();
  VectorXd GetThetaL();
  VectorXd GetTheta();
  VectorXd GetJointTheta();
  VectorXd GetThetaDot();
  VectorXd GetThetaDotEst();
  VectorXd GetThetaDotSMAF();
  VectorXd GetTorque();
  void ReadTheta();    
  void SetTorque(VectorXd tau);  
  void SetPosition(VectorXd theta);  
  void EnableMotor();
  void EnableFilter();
  void ReadCurrentLoopPI();
};


#endif // MOTOR_CONTROLLER_H