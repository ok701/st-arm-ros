#ifndef ST_ARM_PLUGIN_H
#define ST_ARM_PLUGIN_H


#include <eigen3/Eigen/Dense>
#include <ignition/math.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>

#include <string.h>
#include <iostream>
#include <boost/bind.hpp>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int8.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <rbdl/rbdl.h>

// namespace들을 알려준다.
using gazebo::physics::ModelPtr;
using gazebo::physics::LinkPtr;
using gazebo::sensors::ImuSensorPtr;
using gazebo::sensors::SensorPtr;
using gazebo::physics::JointPtr;
using gazebo::event::ConnectionPtr;
using gazebo::common::Time;

using Eigen::Vector3d;
using Eigen::Quaterniond;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Matrix3d;
using Eigen::Matrix4d;

using Eigen::Vector3f;
using Eigen::Quaternionf;
using Eigen::VectorXf;
using Eigen::MatrixXf;
using Eigen::Matrix3f;
using Eigen::Matrix4f;

using std::atan2;
using std::sqrt;

namespace RBDL = RigidBodyDynamics;
namespace RBDLMath = RigidBodyDynamics::Math;

// v2 RBDL
using RBDLModel = RBDL::Model;
using RBDLBody = RBDL::Body;
using RBDLVector3d = RBDL::Math::Vector3d; // Eigen::Vector3d 이라는 동일 이름으로 인해 RBDL::Math::Vector3d는 RBDLVector3d로 변경해준다.
using RBDLVectorNd = RBDL::Math::VectorNd;
using RBDLMatrixNd = RBDL::Math::MatrixNd;
using RBDLMatrix3d = RBDL::Math::Matrix3d;
using RBDLJoint = RBDL::Joint;


#define NUM_OF_JOINTS_WITH_TOOL 8 // TOOL = End Effector, 그래서 좌,우 하나씩 prismatic joint로 바꾸어서 8개가 되었다.
#define NUM_OF_JOINTS_WITHOUT_TOOL 6 // e.e. 제외 joint 수
#define JOINT_VEL_LIMIT 100

#define PI 3.14159265358979
#define M2R 2*PI/4096
#define DEG2RAD		0.017453292519943
#define RAD2DEG		57.295779513082323
#define G -9.81;

// 링크들의 길이_ v2
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


// typedef struct
// {
//   RBDLModel* rbdl_model;
//   RBDLVectorNd q, q_dot, q_d_dot, tau;
//   RBDLMatrixNd jacobian, jacobian_prev, jacobian_dot, jacobian_inverse;

//   unsigned int base_id, shoulder_yaw_id, shoulder_pitch_id, elbow_pitch_id, wrist_pitch_id, wrist_roll_id, wrist_yaw_id;                               //id have information of the body
//   RBDLBody base_link, shoulder_yaw_link, shoulder_pitch_link, elbow_pitch_link, wrist_pitch_link, wrist_roll_link, wrist_yaw_link;
//   RBDLJoint base_joint, shoulder_yaw_joint, shoulder_pitch_joint, elbow_pitch_joint, wrist_pitch_joint, wrist_roll_joint, wrist_yaw_joint;
//   RBDLMatrix3d base_inertia, shoulder_yaw_inertia, shoulder_pitch_inertia, elbow_pitch_inertia, wrist_pitch_inertia, wrist_roll_inertia, wrist_yaw_inertia; //Inertia of links
// } Arm_RBDL;

typedef struct
{
  RBDLModel* rbdl_model;
  RBDLVectorNd q, q_dot, q_d_dot, q_d_dot_ctc, tau_nonlinear, tau_inertia, ee_x0, ee_x_dot;
  RBDLVectorNd virtual_damping, virtual_spring, x_desired_d_dot;
  RBDLVectorNd x_ctc_d_dot, x_actual_dot, x_actual, x_desired_dot, x_desired_dot_last, x_desired, x_desired_last;
  RBDLVectorNd x_error, x_error_dot;
  RBDLMatrixNd jacobian, jacobian_swap, jacobian_prev, jacobian_dot, jacobian_inverse;
  RBDLMatrixNd jacobian_ana, jacobian_ana_swap, jacobian_ana_prev, jacobian_ana_dot, jacobian_ana_inverse;
  RBDLMatrixNd geometric_to_analytic;
  RBDLMatrixNd inertia_matrix;
  RBDLMatrixNd ts_p, ts_v; //Task Space Gain P, D
  RBDLMatrix3d ee_ori_act, ee_ori_act_trans;
  RBDLVector3d ee_pos_act, rpy_ee, rpy_desired, position_desired;

  unsigned int base_id, shoulder_yaw_id, shoulder_pitch_id, elbow_pitch_id, wrist_pitch_id, wrist_roll_id, wrist_yaw_id, gripper_id;                        //id have information of the body
  RBDLBody base_link, shoulder_yaw_link, shoulder_pitch_link, elbow_pitch_link, wrist_pitch_link, wrist_roll_link, wrist_yaw_link, gripper_link;
  RBDLJoint base_joint, shoulder_yaw_joint, shoulder_pitch_joint, elbow_pitch_joint, wrist_pitch_joint, wrist_roll_joint, wrist_yaw_joint, gripper_joint;
  RBDLMatrix3d base_inertia, shoulder_yaw_inertia, shoulder_pitch_inertia, elbow_pitch_inertia, wrist_pitch_inertia, wrist_roll_inertia, wrist_yaw_inertia, gripper_inertia; //Inertia of links
} Arm_RBDL;

Arm_RBDL arm_rbdl;

namespace gazebo
{
  class STArmPlugin : public ModelPlugin
  {
    ModelPtr model;
    LinkPtr Base, Link1, Link2, Link3, Link4, Link5, Link6, LinkGripperL, LinkGripperR;
    JointPtr Joint1, Joint2, Joint3, Joint4, Joint5, Joint6, JointGripperL, JointGripperR;


    //*************** RBQ3 Variables**************//

    // JointPtr FL_hip_joint, FL_thigh_joint, FL_calf_joint,
    //           FR_hip_joint, FR_thigh_joint, FR_calf_joint,
    //           RL_hip_joint, RL_thigh_joint, RL_calf_joint,
    //           RR_hip_joint, RR_thigh_joint, RR_calf_joint;

    JointPtr HRR, HRP, HRK, HLR, HLP, HLK, FRR, FRP, FRK, FLR, FLP, FLK;

    LinkPtr rbq3_base_link;
    JointPtr rbq3_base_joint;
    SensorPtr Sensor;
    ImuSensorPtr RBQ3BaseImu;

    Vector3d rbq3_ref_trajectory, amplitude, frequency, horizontal_translation, vertical_translation, rbq3_base_range_of_motion;
    float traj_time;
    float rbq_base_gain_p;
    float rbq_base_gain_d;
    bool is_move_rbq3{true};
    
    Vector3d rbq3_base_imu_rpy, rbq3_base_rpy, rbq3_base_rpy_ref, rbq3_base_rpy_dot, rbq3_base_torque;

    const std::vector<std::string> rbq3_joint_names = {"HRR", "HRP", "HRK", "HLR", "HLP", "HLK", "FRR", "FRP", "FRK", "FLR", "FLP", "FLK"};

    VectorXd quad_th = VectorXd::Zero(12);
    VectorXd quad_last_th = VectorXd::Zero(12);
    VectorXd quad_th_dot = VectorXd::Zero(12);
    VectorXd quad_joint_torque = VectorXd::Zero(12);
    VectorXd quad_th_ref = VectorXd::Zero(12);

    ros::Publisher pub_rbq3_joint_state;

    // temporary Publisher
    ros::Publisher pub_EE_pose;

    ros::Publisher pub_virtual_spring_torque_0;
    std_msgs::Float32 msg_virtual_spring_torque_0;
    ros::Publisher pub_virtual_spring_torque_1;
    std_msgs::Float32 msg_virtual_spring_torque_1;
    ros::Publisher pub_virtual_spring_torque_2;
    std_msgs::Float32 msg_virtual_spring_torque_2;

    ros::Publisher pub_limited_torque_2;
    std_msgs::Float32 msg_limited_torque_2;

    ros::Publisher pub_ee_pi;
    std_msgs::Float32 msg_ee_theta;
    ros::Publisher pub_ee_theta;
    std_msgs::Float32 msg_ee_pi;
    ros::Publisher pub_ee_psi;
    std_msgs::Float32 msg_ee_psi;
 

    const std::vector<std::string> joint_names = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint_g_l", "joint_g_r"};
    
    common::Time last_update_time;
    common::Time current_time;
    event::ConnectionPtr update_connection;
    double dt = 0.001;
    double time{0};
    double trajectory;
    double deg = (2/180)*PI; //for threshold

    double pi = 0, theta = 0, psi = 0;

    //*************** IK Variables**************//
    MatrixXd JacobianForIK = MatrixXd::Zero(6,6);

    Matrix3d fk_current_orientation;
    Vector3d fk_current_position;

    //*************** Weight estimation **************//
    VectorXd pose_difference = VectorXd::Zero(6);
    Vector3d position_difference = VectorXd::Zero(3);
    float position_difference_magnitude;
    float force_magnitude;
    float estimated_object_weight{0};
    float estimated_object_weight_difference{0};
    float last_estimated_object_weight{0};
    float real_object_weight;
    bool is_start_estimation{false};

    ros::Publisher pub_weight_est_pose_difference;
    ros::Publisher pub_weight_est_estimated_obj_weight;
    ros::Subscriber sub_weight_est_real_obj_weight;
    ros::Subscriber sub_weight_est_start_estimation;

    void EstimateObjectWeight();
    void SwitchOnAddingEstimatedObjWeightToRBDL(const std_msgs::Int32Ptr & msg);


    //*************** Trajectory Variables**************//
    double step_time{0};
    double cnt_time{0};
    unsigned int cnt{0};
    unsigned int start_flag{0};

    MatrixXd unit_6 = MatrixXd::Zero(6,6);


    // float qw{0};
    // float qx{0};
    // float qy{0};
    // float qz{0};

    // float pre_data_x{0};
    // float pre_data_y{0};
    // float pre_data_z{0};

    // float input_P = 200;
    // float input_D{0};

    Vector3d manipulator_com, 
            ref_com_position, 
            shoulder_link_com, 
            arm_link_com, 
            elbow_link_com, 
            forearm_link_com, 
            wrist_link_com, 
            endeffector_link_com;

    Vector3d C1, C2, C3, C4, C5, C6;
    Vector3d a0, a1, a2, a3, a4, a5;
    Vector3d C1_P0, C2_P1, C3_P2, C4_P3, C5_P4, C6_P5;
    Vector3d P6_P0, P6_P1, P6_P2, P6_P3, P6_P4, P6_P5;
    Vector3d J1_CoM, J2_CoM, J3_CoM, J4_CoM, J5_CoM, J6_CoM;
    Vector3d initial_com_position, desired_com_position, virtual_spring_com, com_force; 
    VectorXd virtual_spring_rotational = VectorXd::Zero(6); 
    VectorXd tau_com = VectorXd::Zero(6);
    VectorXd tau_rotational = VectorXd::Zero(6);
    
    VectorXd J1 = VectorXd::Zero(6); 
    VectorXd J2 = VectorXd::Zero(6); 
    VectorXd J3 = VectorXd::Zero(6); 
    VectorXd J4 = VectorXd::Zero(6); 
    VectorXd J5 = VectorXd::Zero(6); 
    VectorXd J6 = VectorXd::Zero(6);

    MatrixXd Jacobian = MatrixXd::Zero(6,6);

    MatrixXd Jacobian_tp = MatrixXd::Zero(6,6); // Jacobian transpose
    MatrixXd J_for_dpi = MatrixXd::Zero(6,6); // 중간 연산 재료
    MatrixXd J_inverse_for_dpi = MatrixXd::Zero(6,6); // 중간 연산 재료 inverse
    MatrixXd Jacobian_e_dpi = MatrixXd::Zero(6,6); // Error Damped Pseudo Inverse Jacobian
    MatrixXd W_term = MatrixXd::Zero(6,6); // 조정항

    MatrixXd J_CoM = MatrixXd::Zero(3,6);
    double estimation_error_innerproduct = 0; // 추정 오차의 내적


    Vector3d  ee_position, 
              ee_velocity, 
              pre_ee_position, 
              ref_ee_position, 
              initial_ee_position, 
              hmd_position;

    Vector3d gain_p, gain_d, gain_w;
    VectorXd gain_p_joint_space = VectorXd(NUM_OF_JOINTS_WITH_TOOL);
    VectorXd gain_d_joint_space = VectorXd(NUM_OF_JOINTS_WITH_TOOL);
    VectorXd gain_p_joint_space_idle = VectorXd(NUM_OF_JOINTS_WITH_TOOL);
    VectorXd gain_d_joint_space_idle = VectorXd(NUM_OF_JOINTS_WITH_TOOL);
    VectorXd gain_r = VectorXd(6);
    VectorXd threshold = VectorXd(6);
    Vector3d gain_p_task_space, gain_w_task_space;

    VectorXd temp_gain_p = VectorXd(6);    
    VectorXd temp_gain_v = VectorXd(6);


    Vector3d ee_rotation_x, ee_rotation_y, ee_rotation_z, 
            ref_ee_rotation_x, ref_ee_rotation_y, ref_ee_rotation_z,
            ee_orientation_error, ee_position_error, ee_force, ee_momentum;

    Matrix3d ee_rotation, ref_ee_rotation;

    Quaterniond ref_ee_quaternion;
    Quaterniond ee_quaternion;
    Quaterniond hmd_quaternion;

    VectorXd th = VectorXd::Zero(NUM_OF_JOINTS_WITH_TOOL);
    VectorXd ref_th = VectorXd::Zero(NUM_OF_JOINTS_WITH_TOOL);
    VectorXd last_th = VectorXd::Zero(NUM_OF_JOINTS_WITH_TOOL);
    VectorXd th_dot = VectorXd::Zero(NUM_OF_JOINTS_WITH_TOOL);
    VectorXd last_th_dot = VectorXd::Zero(NUM_OF_JOINTS_WITH_TOOL);
    VectorXd th_d_dot = VectorXd::Zero(NUM_OF_JOINTS_WITH_TOOL);
    VectorXd init_position = VectorXd::Zero(NUM_OF_JOINTS_WITH_TOOL);

    VectorXd ik_th = VectorXd::Zero(6);
    VectorXd ik_current_pose = VectorXd::Zero(6);
    
    MatrixXd A0 = MatrixXd::Zero(4,4);  MatrixXd A1 = MatrixXd::Zero(4,4); MatrixXd A2 = MatrixXd::Zero(4,4); 
    MatrixXd A3 = MatrixXd::Zero(4,4);
    MatrixXd A4 = MatrixXd::Zero(4,4);  MatrixXd A5 = MatrixXd::Zero(4,4); MatrixXd A6 = MatrixXd::Zero(4,4);
    MatrixXd T00 = MatrixXd::Zero(4,4); MatrixXd T01 = MatrixXd::Zero(4,4); MatrixXd T02 = MatrixXd::Zero(4,4); 
    MatrixXd T03 = MatrixXd::Zero(4,4);
    MatrixXd T04 = MatrixXd::Zero(4,4); MatrixXd T05 = MatrixXd::Zero(4,4); MatrixXd T06 = MatrixXd::Zero(4,4);
    MatrixXd T12 = MatrixXd::Zero(4,4); MatrixXd T23 = MatrixXd::Zero(4,4); MatrixXd T34 = MatrixXd::Zero(4,4); 
    MatrixXd T45 = MatrixXd::Zero(4,4); MatrixXd T56 = MatrixXd::Zero(4,4);

    MatrixXd joint_limit = MatrixXd::Zero(2,6);

    VectorXd joint_torque = VectorXd::Zero(NUM_OF_JOINTS_WITH_TOOL);
    VectorXd tau_gravity_compensation = VectorXd::Zero(NUM_OF_JOINTS_WITH_TOOL);
    // VectorXd gripper_torque = VectorXd::Zero(2);
    VectorXd virtual_spring = VectorXd::Zero(6);
    VectorXd estimation_error = VectorXd::Zero(6);
    VectorXd tau_viscous_damping = VectorXd::Zero(NUM_OF_JOINTS_WITH_TOOL);
    VectorXd tau_rbdl = VectorXd::Zero(6);
    VectorXd tau = VectorXd::Zero(6);
    VectorXd tau_vs = VectorXd::Zero(6);
    VectorXd tau_limit = VectorXd::Zero(6);


    // Temporary variables

    // End of Temporary variables

    ros::NodeHandle node_handle;
    ros::Publisher pub_joint_state;
    ros::Publisher pub_joint_state_deg;
    ros::Publisher pub_joint_state_ik;
    ros::Publisher pub_ee_pose;
    ros::Publisher pub_ref_ee_pose;
    ros::Publisher pub_base_imu_pose;

    ros::Subscriber sub_gripper_state;
    ros::Subscriber sub_hmd_pose;
    ros::Subscriber sub_mode_selector;
    ros::Subscriber sub_gain_p_task_space;
    ros::Subscriber sub_gain_w_task_space;
    ros::Subscriber sub_gain_p_joint_space;
    ros::Subscriber sub_gain_d_joint_space;
    ros::Subscriber sub_gain_r;
    ros::Subscriber sub_rbq3_motion_switch;

    //ros::Publisher pub_gazebo_camera;   //++

    enum ControlMode
    {
      IDLE = 0,
      Motion_1,
      Motion_2,
      Motion_3,
      Motion_4,
      Motion_5,
      Motion_6,
      Motion_7,
      Motion_8,
      Motion_9,
    };
    enum ControlMode control_mode;

    void Load(ModelPtr _model, sdf::ElementPtr/*, sensors::SensorPtr _parent*/);    //++
    void Loop();
    void GetLinks();
    void GetJoints();
    void GetSensors();
    void InitROSPubSetting();

    void SetRBDLVariables();
    void InitializeRBDLVariables();
    void InitializeRBDLVariablesWithObj(float);

    void GetJointPosition();
    void GetJointVelocity();
    void GetJointAcceleration();
    void GetSensorValues();
    void SetJointTorque();
    void ROSMsgPublish();

    void PostureGeneration();
    void Idle();
    void Motion1();
    void Motion2();
    void Motion3();
    void Motion4();
    void Motion5();
    void Motion6();
    void Motion7();
    void Motion8();
    void Motion9();
    void SwitchMode(const std_msgs::Int32Ptr & msg);
    void SwitchGain(const std_msgs::Int32Ptr & msg);
    void SwitchGainJointSpaceP(const std_msgs::Float32MultiArrayConstPtr &msg);
    void SwitchGainJointSpaceD(const std_msgs::Float32MultiArrayConstPtr &msg);
    void SwitchGainTaskSpaceP(const std_msgs::Float32MultiArrayConstPtr &msg);
    void SwitchGainTaskSpaceW(const std_msgs::Float32MultiArrayConstPtr &msg);
    void SwitchGainR(const std_msgs::Float32MultiArrayConstPtr &msg);
    void SwitchModeRBQ3(const std_msgs::Bool &msg);
    void GripperStateCallback(const std_msgs::Float32ConstPtr &msg);
    void HMDPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

    void RBQ3Motion1();
    void RBQ3Motion2();
    void GetRBQ3Joints();
    void GetRBQ3JointPosition();
    void GetRBQ3JointVelocity();
    void SetRBQ3JointTorque();
    Vector3d GetRBQ3RightIK(Vector3d position);
    Vector3d GetRBQ3LeftIK(Vector3d position);
    
    void GripperControl();
    float Map(float x, float in_min, float in_max, float out_min, float out_max);
  
    bool InverseSolverUsingJacobian(Vector3d a_target_position, Matrix3d a_target_orientation);
    bool InverseSolverUsingSRJacobian(Vector3d target_position, Matrix3d target_orientation);
    bool IK(Vector3d target_position, Matrix3d target_orientation);

    void GetJacobians(VectorXd a_th, MatrixXd Jacobian, Matrix4d Current_Pose);
    void GetJacobians(VectorXd a_th, MatrixXd Jacobian, Vector3d current_position, Matrix3d current_orientation);

    void UpdateJacobian(VectorXd a_th);
    void Calc_Feedback_Pose(Arm_RBDL &rbdl);

    Vector3d PositionDifference(Vector3d desired_position, Vector3d present_position);
    Vector3d OrientationDifference(Matrix3d desired_orientation, Matrix3d present_orientation);
    Vector3d MatrixLogarithm(Matrix3d rotation_matrix);
    Matrix3d skewSymmetricMatrix(Vector3d v);
    MatrixXd getDampedPseudoInverse(MatrixXd Jacobian, float lamda);
    VectorXd PoseDifference(Vector3d desired_position, Matrix3d desired_orientation, Matrix4d present_pose);
    VectorXd PoseDifference(Vector3d desired_position, Matrix3d desired_orientation, Vector3d present_position, Matrix3d present_orientation);


    // void SolveInverseKinematics();
    // VectorXd SolveForwardKinematics(VectorXd a_th);
    // VectorXd poseDifference(Vector3d desired_position, Vector3d present_position,
    //                             Matrix3d desired_orientation, Matrix3d present_orientation);
    // MatrixXd jacobian();
  };
  GZ_REGISTER_MODEL_PLUGIN(STArmPlugin); // ST-Arm class를 gazebo 모델에 등록해준다.
}

#endif  // end of the ST_ARM_PLUGIN_H