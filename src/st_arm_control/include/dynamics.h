#ifndef DYNAMICS_H
#define DYNAMICS_H

#include "motor_controller.h"


#include <eigen3/Eigen/Dense>

#include <string.h>
#include <iostream>
#include <boost/bind.hpp>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Int32.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <rbdl/rbdl.h>

using Eigen::Vector3d;
using Eigen::Quaterniond;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Matrix3d;


namespace RBDL = RigidBodyDynamics;
namespace RBDLMath = RigidBodyDynamics::Math;

using RBDLModel = RBDL::Model;
using RBDLBody = RBDL::Body;
using RBDLVector3d = RBDL::Math::Vector3d;
using RBDLVectorNd = RBDL::Math::VectorNd;
using RBDLMatrixNd = RBDL::Math::MatrixNd;
using RBDLMatrix3d = RBDL::Math::Matrix3d;
using RBDLJoint = RBDL::Joint;


#define deg2rad		0.017453292519943
#define rad2deg		57.295779513082323
#define g           9.81   
#define PI          3.1415

// #define L1 0.16
#define L1 0.10
#define L2 0.25
#define L3 0.25
#define L4 0.00
#define L5 0.1045
#define L6 0.07

//for caluculationg COM of st_arm_v3 
#define joint0_to_L1_com 0.09579
#define joint1_to_L2_com 0.21653
#define joint2_to_L3_com 0.16062
#define joint3_to_L4_com 0.04238
#define joint4_to_L5_com 0.02707
#define joint5_to_L6_com 0.06938

// st_arm_v3 mass
#define m_Link1 0.54333
#define m_Link2 0.67378
#define m_Link3 0.19195
#define m_Link4 0.083173
#define m_Link5 0.083173
#define m_Link6 0.35586

#define m_Arm 1.931266 // (m_Link1~6 합친거)
#define M1 1.931266 // m_Arm
#define M2 1.38793 //m_Link2+m_Link3+m_Link4+m_Link5+m_Link6;
#define M3 0.71415 //m_Link3+m_Link4+m_Link5+m_Link6;
#define M4 0.52220 //m_Link4+m_Link5+m_Link6;
#define M5 0.43903 //m_Link5+m_Link6;
#define M6 0.35586 //m_Link6;
#define inner_dt 0.001

typedef struct
{
  RBDLModel* rbdl_model;
  RBDLVectorNd q, q_dot, q_d_dot, tau;
  RBDLMatrixNd jacobian, jacobian_prev, jacobian_dot, jacobian_inverse;

  unsigned int base_id, shoulder_yaw_id, shoulder_pitch_id, elbow_pitch_id, wrist_pitch_id, wrist_roll_id, wrist_yaw_id;                               //id have information of the body
  RBDLBody base_link, shoulder_yaw_link, shoulder_pitch_link, elbow_pitch_link, wrist_pitch_link, wrist_roll_link, wrist_yaw_link;
  RBDLJoint base_joint, shoulder_yaw_joint, shoulder_pitch_joint, elbow_pitch_joint, wrist_pitch_joint, wrist_roll_joint, wrist_yaw_joint;
  RBDLMatrix3d base_inertia, shoulder_yaw_inertia, shoulder_pitch_inertia, elbow_pitch_inertia, wrist_pitch_inertia, wrist_roll_inertia, wrist_yaw_inertia; //Inertia of links
} Arm_RBDL;


namespace Dynamics
{
    class JMDynamics
    {
        const std::vector<std::string> joint_names = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};

        Arm_RBDL arm_rbdl;


        double dt = 0.002;
        double time = 0;
        double trajectory = 0;

        double cnt_time = 0;
        unsigned int cnt = 0;   
        double deg1 = (1/180)*PI;

        Vector3d ee_rotation_x, ee_rotation_y, ee_rotation_z, ref_ee_rotation_x, ref_ee_rotation_y, ref_ee_rotation_z;
        Vector3d ee_orientation_error, ee_force, ee_momentum;

        Matrix3d ee_rotation;
        Matrix3d ref_ee_rotation;

        Quaterniond ref_ee_quaternion;  
        Quaterniond ee_quaternion;  
        VectorXd initial_pose = VectorXd::Zero(6);
        
        MatrixXd A0 = MatrixXd::Zero(4,4);  MatrixXd A1 = MatrixXd::Zero(4,4); MatrixXd A2 = MatrixXd::Zero(4,4); 
        MatrixXd A3 = MatrixXd::Zero(4,4);
        MatrixXd A4 = MatrixXd::Zero(4,4);  MatrixXd A5 = MatrixXd::Zero(4,4); MatrixXd A6 = MatrixXd::Zero(4,4);
        MatrixXd T00 = MatrixXd::Zero(4,4); MatrixXd T01 = MatrixXd::Zero(4,4); MatrixXd T02 = MatrixXd::Zero(4,4); 
        MatrixXd T03 = MatrixXd::Zero(4,4);
        MatrixXd T04 = MatrixXd::Zero(4,4); MatrixXd T05 = MatrixXd::Zero(4,4); MatrixXd T06 = MatrixXd::Zero(4,4);
        MatrixXd T12 = MatrixXd::Zero(4,4); MatrixXd T23 = MatrixXd::Zero(4,4); MatrixXd T34 = MatrixXd::Zero(4,4); 
        MatrixXd T45 = MatrixXd::Zero(4,4); MatrixXd T56 = MatrixXd::Zero(4,4);    
        Vector3d a0, a1, a2, a3, a4, a5, P6_P0, P6_P1, P6_P2, P6_P3, P6_P4, P6_P5;

        Vector3d manipulator_com, 
                ref_com_position, 
                shoulder_link_com, 
                arm_link_com, 
                elbow_link_com, 
                forearm_link_com, 
                wrist_link_com, 
                endeffector_link_com;

        Vector3d C1, C2, C3, C4, C5, C6;
        Vector3d C1_P0, C2_P1, C3_P2, C4_P3, C5_P4, C6_P5;
        Vector3d J1_CoM, J2_CoM, J3_CoM, J4_CoM, J5_CoM, J6_CoM;
        Vector3d initial_com_position, desired_com_position, virtual_spring_com, com_force; 
        VectorXd virtual_spring_rotational = VectorXd::Zero(6); 
        VectorXd tau = VectorXd::Zero(6);
        VectorXd tau_com = VectorXd::Zero(6);
        VectorXd tau_rotational = VectorXd::Zero(6);
        MatrixXd J_CoM = MatrixXd::Zero(3,6);

        MatrixXd joint_limit = MatrixXd::Zero(2,6);
        VectorXd threshold = VectorXd(6);



        MatrixXd Jacobian = MatrixXd::Zero(6,6);    
        VectorXd J1 = VectorXd::Zero(6); VectorXd J2 = VectorXd::Zero(6); VectorXd J3 = VectorXd::Zero(6);
        VectorXd J4 = VectorXd::Zero(6); VectorXd J5 = VectorXd::Zero(6); VectorXd J6 = VectorXd::Zero(6); 

        VectorXd tau_nonlinear_effects = VectorXd::Zero(6);
        VectorXd tau_gravity_compensation = VectorXd::Zero(6);
        VectorXd gripper_torque = VectorXd::Zero(2);
        VectorXd virtual_spring = VectorXd::Zero(6);

        // Temporary variables
        Quaterniond om_ee_quaternion;
        Vector3d om_ee_position;
        Matrix3d om_ee_rotation;
        MatrixXd om_A0 = MatrixXd::Zero(4,4); MatrixXd om_A1 = MatrixXd::Zero(4,4); MatrixXd om_A2 = MatrixXd::Zero(4,4); 
        MatrixXd om_A3 = MatrixXd::Zero(4,4); MatrixXd om_A4 = MatrixXd::Zero(4,4);  MatrixXd om_A5 = MatrixXd::Zero(4,4); 
        MatrixXd om_A6 = MatrixXd::Zero(4,4); MatrixXd om_T06 = MatrixXd::Zero(4,4);
        // End of Temporary variables

        std::vector<double> present_joint_angle_;

        enum ControlMode
        {
            gravity_compensation = 0,
            manipulation_mode,
            vision_mode,
            draw_infinity,
            not_defined_1,
            not_defined_2    
        };
        enum ControlMode control_mode;

    public:
        JMDynamics();
        //~JMDynamics();
        VectorXd th_joint = VectorXd::Zero(6);
        VectorXd last_th_joint = VectorXd::Zero(6);
        VectorXd th_dot_joint = VectorXd::Zero(6);
        VectorXd th_motor = VectorXd::Zero(6);
        VectorXd th_incremental = VectorXd::Zero(6);
        VectorXd zero_th = VectorXd::Zero(6);
        VectorXd th_shoulder = VectorXd::Zero(3);
        VectorXd th_wrist = VectorXd::Zero(4);
        VectorXd th = VectorXd::Zero(7);
        VectorXd ref_th = VectorXd::Zero(7);
        VectorXd last_th = VectorXd::Zero(6);
        VectorXd th_dot = VectorXd::Zero(6);
        VectorXd th_dot_estimated = VectorXd::Zero(7);
        VectorXd th_dot_sma_filtered = VectorXd::Zero(7);
        VectorXd last_th_dot = VectorXd::Zero(7);
        VectorXd th_d_dot = VectorXd::Zero(7);  
        VectorXd zero_vector_6 = VectorXd::Zero(6);
        float th_gripper{0};

        int count = 0;
        int step_time = 5;

        VectorXd joint_gear_reduction = VectorXd::Zero(6);

        Quaterniond hmd_quaternion;

        Vector3d ee_position, ee_velocity, pre_ee_position, ref_ee_position, initial_ee_position, hmd_position, ee_position_last;
        Vector3d gain_p, gain_d, gain_w;
        VectorXd gain_r = VectorXd(6);

        VectorXd om_th = VectorXd::Zero(6);
        VectorXd joint_torque = VectorXd::Zero(7);
        VectorXd tau_viscous_damping = VectorXd::Zero(6);

        VectorXd gain_p_joint_space = VectorXd::Zero(7);
        VectorXd gain_d_joint_space = VectorXd::Zero(7);

        // VectorXd gain_p_task_space = VectorXd(3);
        Vector3d gain_p_task_space;
        Vector3d gain_d_task_space;
        Vector3d gain_w_task_space;

        VectorXd GetTorque();
        void SetTheta(VectorXd thetas);
        void SetThetaWrist(VectorXd thetas);
        void SetThetaDot(VectorXd);
        void SetThetaDotSMAF(VectorXd);
        void SetThetaDotEst(VectorXd);
        void SetGripperValue(float);
        void CalculateRefEEPose();
        //void OM_joint_states_callback(const sensor_msgs::JointState::ConstPtr &msg);
        void SetOMTheta(VectorXd thetas);
        void GenerateTorqueJointSpacePD();
        void GenerateTorqueTaskSpacePD();
        void GenerateTorqueOneMotorTuning();
        void GenerateTrajectory();
        void GenerateTorqueGravityCompensation();
        void CalculateJointTheta();
        void GenerateTorqueManipulationMode();
        void GenerateTorqueVisionMode();
        void GenerateGripperTorque();
        void PostureGeneration();
        void Loop();
        void SwitchMode(const std_msgs::Int32ConstPtr & msg);


        void SetRBDLVariables();
        void InitializeRBDLVariables();
    };
}


#endif // DYNAMICS_H