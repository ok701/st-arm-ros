#include "st_arm_plugin.h"

namespace gazebo
{
  void STArmPlugin::Load(ModelPtr _model, sdf::ElementPtr/*, sensors::SensorPtr _parent*/)
  {
    this->model = _model; //여기서 this는 객체의 주소를 가리킨다. 이 model 객체(class)의 주소값을 가리킨다.
    GetLinks(); //각 링크들의 정보를 얻어낸다.
    GetJoints(); // 각 joint들의 정보를 얻어낸다.
    GetSensors(); // 센서들의 정보를 얻어낸다.
    InitROSPubSetting();
    std::cout << "Before Calling RBDL Initialize function" << std::endl; //앞의 코드가 문제가 있을 시 나오지않는 출력

    // InitializeRBDLVariables();
    InitializeRBDLVariablesWithObj(0);
    //InitializeRBDLVariablesWithObj_v1(0);

    
    this->last_update_time = this->model->GetWorld()->SimTime(); // 가제보월드에서 가제보만의 시간을 불러온다. 
    //가제보시간은 컴퓨터의 처리능력에 따라 느려지거나 할 수 있다. 그래서 현실시간(컴퓨터의 시간)이 아닌 가제보시간에서 로봇이 움직여야 제대로된 작동을 하게된다.

    this->update_connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&STArmPlugin::Loop, this)); // 계속해서 Loop하는 스레드를 만든다.
    
    //event::Events::ConnectWorldUpdateBegin
    //이 함수의 경우, 우리의 Plugin's Update method(이 코드의 경우, STArmPlugin::Loop이다)를 가제보 시뮬레이터의 Events 클래스 인스턴스에 대한 콜백 기능으로써 사용하게된다.
    //그래서 가제보가 우리의 Plugin's method를 시뮬레이션의 매 time step마다 부를 수 있도록 허용해준다.
    // 
    //boost::bind
    //특정 함수를 호출할 때, 그 매개변수 값을 함께 넘겨주기 위해 bind를 사용한다.

    is_move_rbq3 = true;
    if(is_move_rbq3)
    {
      GetRBQ3Joints();
    }

    std::cout << "Load..." << std::endl;
  }


  void STArmPlugin::GetLinks() //모델(digital twin)의 각 링크들마다, 자신들이 어떤 정보를 가진 링크들인지 입력하는 함수, updated : 2023-01-20 쓰고 있지 않는 함수
  {
    this->Base  = this->model->GetLink("base_link");
    this->Link1 = this->model->GetLink("shoulder_yaw_link");
    this->Link2 = this->model->GetLink("shoulder_pitch_link");
    this->Link3 = this->model->GetLink("elbow_pitch_link");
    this->Link4 = this->model->GetLink("wrist_pitch_link");
    this->Link5 = this->model->GetLink("wrist_roll_link");
    this->Link6 = this->model->GetLink("wrist_yaw_link");
    this->LinkGripperL = this->model->GetLink("gripper_left_link");
    this->LinkGripperR = this->model->GetLink("gripper_right_link");
  }


  void STArmPlugin::GetJoints() //모델(digital twin)의 각 joint들마다, 자신들이 어떤 정보를 가진 joint들인지 입력하는 함수
  {
    this->Joint1 = this->model->GetJoint("shoulder_yaw_joint");
    this->Joint2 = this->model->GetJoint("shoulder_pitch_joint");
    this->Joint3 = this->model->GetJoint("elbow_pitch_joint");
    this->Joint4 = this->model->GetJoint("wrist_pitch_joint");
    this->Joint5 = this->model->GetJoint("wrist_roll_joint");
    this->Joint6 = this->model->GetJoint("wrist_yaw_joint");
    this->JointGripperL = this->model->GetJoint("gripper_left_joint");
    this->JointGripperR = this->model->GetJoint("gripper_right_joint");
  }


  void STArmPlugin::GetSensors() //모델(digital twin)의 센서, 자신들이 어떤 정보를 가진 joint들인지 입력하는 함수
  {
    this->Sensor = sensors::get_sensor("rbq3_base_imu");
    this->RBQ3BaseImu = std::dynamic_pointer_cast<sensors::ImuSensor>(Sensor);
  }


  void STArmPlugin::InitROSPubSetting()
  {
    pub_joint_state = node_handle.advertise<sensor_msgs::JointState>("st_arm/joint_states", 100);
    pub_joint_state_deg = node_handle.advertise<sensor_msgs::JointState>("st_arm/joint_states_deg", 100);
    pub_joint_state_ik = node_handle.advertise<sensor_msgs::JointState>("st_arm/joint_states_ik", 100);
    sub_mode_selector = node_handle.subscribe("st_arm/mode_selector", 100, &gazebo::STArmPlugin::SwitchMode, this); 
    sub_gain_p_task_space = node_handle.subscribe("st_arm/TS_Kp", 100, &gazebo::STArmPlugin::SwitchGainTaskSpaceP, this); 
    sub_gain_w_task_space = node_handle.subscribe("st_arm/TS_Kw", 100, &gazebo::STArmPlugin::SwitchGainTaskSpaceW, this); 
    sub_gain_p_joint_space = node_handle.subscribe("st_arm/JS_Kp", 100, &gazebo::STArmPlugin::SwitchGainJointSpaceP, this); 
    sub_gain_d_joint_space = node_handle.subscribe("st_arm/JS_Kd", 100, &gazebo::STArmPlugin::SwitchGainJointSpaceD, this); 
    sub_gain_r = node_handle.subscribe("st_arm/JS_Kr", 100, &gazebo::STArmPlugin::SwitchGainR, this); 
    
    sub_gripper_state = node_handle.subscribe("unity/gripper_state", 100, &gazebo::STArmPlugin::GripperStateCallback, this);
    sub_hmd_pose = node_handle.subscribe("unity/virtual_box_pose", 100, &gazebo::STArmPlugin::HMDPoseCallback, this);

    sub_rbq3_motion_switch = node_handle.subscribe("rbq3/motion_switch", 100, &gazebo::STArmPlugin::SwitchModeRBQ3, this);

    pub_ee_pose = node_handle.advertise<geometry_msgs::TransformStamped>("st_arm/ee_pose__noting_here", 10);
    pub_ref_ee_pose = node_handle.advertise<geometry_msgs::TransformStamped>("st_arm/ref_ee_pose", 10);
  
    pub_rbq3_joint_state = node_handle.advertise<sensor_msgs::JointState>("rbq3/joint_states", 200);
    pub_weight_est_pose_difference = node_handle.advertise<std_msgs::Float32MultiArray>("st_arm/pose_difference", 100);
    pub_weight_est_estimated_obj_weight = node_handle.advertise<std_msgs::Float32>("st_arm/estimated_obj_weight", 100);
    sub_weight_est_start_estimation = node_handle.subscribe("unity/calibrate_obj_weight", 100, &gazebo::STArmPlugin::SwitchOnAddingEstimatedObjWeightToRBDL, this);

    //virtual spring torque : tau_vs
    pub_virtual_spring_torque_0 = node_handle.advertise<std_msgs::Float32>("vs_torque_JOINT0", 10);
    pub_virtual_spring_torque_1 = node_handle.advertise<std_msgs::Float32>("vs_torque_JOINT1", 10);
    pub_virtual_spring_torque_2 = node_handle.advertise<std_msgs::Float32>("vs_torque_JOINT2", 10);

    //limited torque
    pub_limited_torque_2 = node_handle.advertise<std_msgs::Float32>("limited_torque_JOINT2", 10);

    //EE eulerangle
    pub_ee_pi = node_handle.advertise<std_msgs::Float32>("ee_pi", 100);
    pub_ee_theta = node_handle.advertise<std_msgs::Float32>("ee_theta", 100);
    pub_ee_psi = node_handle.advertise<std_msgs::Float32>("ee_psi", 100);

    //Pose
    pub_EE_pose = node_handle.advertise<geometry_msgs::Pose>("st_arm/EE_pose", 100);

  }


  void STArmPlugin::InitializeRBDLVariables()
  {
    rbdl_check_api_version(RBDL_API_VERSION);
    std::cout << "Checked RBDL API VERSION" << std::endl;

    arm_rbdl.rbdl_model = new RBDLModel();
    arm_rbdl.rbdl_model->gravity = RBDL::Math::Vector3d(0.0, 0.0, -9.81);

    arm_rbdl.base_inertia = RBDLMatrix3d(0.00033, 0,        0,
                                         0,       0.00034,  0,
                                         0,       0,        0.00056);

    arm_rbdl.shoulder_yaw_inertia = RBDLMatrix3d( 0.00024,  0,        0,
                                                  0,        0.00040,  0,
                                                  0,        0,        0.00026);

    arm_rbdl.shoulder_pitch_inertia = RBDLMatrix3d(0.00028, 0,        0,
                                                   0,       0.00064,  0,
                                                   0,       0,        0.00048);

    arm_rbdl.elbow_pitch_inertia = RBDLMatrix3d(0.00003,  0,        0,
                                                0,        0.00019,  0,
                                                0,        0,        0.00020);

    arm_rbdl.wrist_pitch_inertia = RBDLMatrix3d(0.00002,  0,        0,
                                                0,        0.00002,  0,
                                                0,        0,        0.00001);

    arm_rbdl.wrist_roll_inertia = RBDLMatrix3d(0.00001,  0,        0,
                                                0,        0.00002,  0,
                                                0,        0,        0.00001);

    arm_rbdl.wrist_yaw_inertia = RBDLMatrix3d(0.00006,  0,        0,
                                                0,        0.00003,  0,
                                                0,        0,        0.00005);

    // arm_rbdl.base_link = RBDLBody(0.59468, RBDLVector3d(0, 0.00033, 0.03107), arm_rbdl.base_inertia);
    // arm_rbdl.base_joint = RBDLJoint(RBDL::JointType::JointTypeFixed, RBDLVector3d(0,0,0));
    // arm_rbdl.base_id = arm_rbdl.rbdl_model->RBDLModel::AddBody(0, RBDL::Math::Xtrans(RBDLVector3d(0,0,0)), arm_rbdl.base_joint, arm_rbdl.base_link);

    arm_rbdl.shoulder_yaw_link = RBDLBody(0.55230, RBDLVector3d(0.00007, -0.00199, 0.09998), arm_rbdl.shoulder_yaw_inertia);
    arm_rbdl.shoulder_yaw_joint = RBDLJoint(RBDL::JointType::JointTypeRevolute, RBDLVector3d(0,0,1));
    arm_rbdl.shoulder_yaw_id = arm_rbdl.rbdl_model->RBDLModel::AddBody(0, RBDL::Math::Xtrans(RBDLVector3d(0,0,0)), arm_rbdl.shoulder_yaw_joint, arm_rbdl.shoulder_yaw_link);

    arm_rbdl.shoulder_pitch_link = RBDLBody(0.65326, RBDLVector3d(0.22204, 0.04573, 0), arm_rbdl.shoulder_pitch_inertia);
    arm_rbdl.shoulder_pitch_joint = RBDLJoint(RBDL::JointType::JointTypeRevolute, RBDLVector3d(0,1,0));
    arm_rbdl.shoulder_pitch_id = arm_rbdl.rbdl_model->RBDLModel::AddBody(1, RBDL::Math::Xtrans(RBDLVector3d(0, 0, 0.1019)), arm_rbdl.shoulder_pitch_joint, arm_rbdl.shoulder_pitch_link);

    arm_rbdl.elbow_pitch_link = RBDLBody(0.17029, RBDLVector3d(0.17044, 0.00120, 0.00004), arm_rbdl.elbow_pitch_inertia);
    arm_rbdl.elbow_pitch_joint = RBDLJoint(RBDL::JointType::JointTypeRevolute, RBDLVector3d(0,1,0));
    arm_rbdl.elbow_pitch_id = arm_rbdl.rbdl_model->RBDLModel::AddBody(2, RBDL::Math::Xtrans(RBDLVector3d(0.25,0,0)), arm_rbdl.elbow_pitch_joint, arm_rbdl.elbow_pitch_link);

    arm_rbdl.wrist_pitch_link = RBDLBody(0.09234, RBDLVector3d(0.04278, 0, 0.01132), arm_rbdl.wrist_pitch_inertia);
    arm_rbdl.wrist_pitch_joint = RBDLJoint(RBDL::JointType::JointTypeRevolute, RBDLVector3d(0,1,0));
    arm_rbdl.wrist_pitch_id = arm_rbdl.rbdl_model->RBDLModel::AddBody(3, RBDL::Math::Xtrans(RBDLVector3d(0.25,0,0)), arm_rbdl.wrist_pitch_joint, arm_rbdl.wrist_pitch_link);

    arm_rbdl.wrist_roll_link = RBDLBody(0.08696, RBDLVector3d(0.09137, 0, 0.00036), arm_rbdl.wrist_roll_inertia);
    arm_rbdl.wrist_roll_joint = RBDLJoint(RBDL::JointType::JointTypeRevolute, RBDLVector3d(1,0,0));
    arm_rbdl.wrist_roll_id = arm_rbdl.rbdl_model->RBDLModel::AddBody(4, RBDL::Math::Xtrans(RBDLVector3d(0,0,0)), arm_rbdl.wrist_roll_joint, arm_rbdl.wrist_roll_link);

    arm_rbdl.wrist_yaw_link = RBDLBody(0.14876, RBDLVector3d(0.05210, 0.00034, 0.02218), arm_rbdl.wrist_yaw_inertia);
    arm_rbdl.wrist_yaw_joint = RBDLJoint(RBDL::JointType::JointTypeRevolute, RBDLVector3d(0,0,1));
    arm_rbdl.wrist_yaw_id = arm_rbdl.rbdl_model->RBDLModel::AddBody(5, RBDL::Math::Xtrans(RBDLVector3d(0.1045,0,0)), arm_rbdl.wrist_yaw_joint, arm_rbdl.wrist_yaw_link);

    arm_rbdl.q = RBDLVectorNd::Zero(6);
    arm_rbdl.q_dot = RBDLVectorNd::Zero(6);
    arm_rbdl.q_d_dot = RBDLVectorNd::Zero(6);
    arm_rbdl.tau_nonlinear = RBDLVectorNd::Zero(6);

    arm_rbdl.jacobian = RBDLMatrixNd::Zero(6,6);
    arm_rbdl.jacobian_prev = RBDLMatrixNd::Zero(6,6);
    arm_rbdl.jacobian_dot = RBDLMatrixNd::Zero(6,6);
    arm_rbdl.jacobian_inverse = RBDLMatrixNd::Zero(6,6);

    std::cout << "RBDL Initialize function success" << std::endl;
  }


  void STArmPlugin::InitializeRBDLVariablesWithObj(float a_obj_weight) 
  {
    rbdl_check_api_version(RBDL_API_VERSION);
    std::cout << "Checked RBDL API VERSION" << std::endl;

    arm_rbdl.rbdl_model = new RBDLModel();
    arm_rbdl.rbdl_model->gravity = RBDL::Math::Vector3d(0.0, 0.0, -9.81);

    arm_rbdl.base_inertia = RBDLMatrix3d(0.0002, 0,        0,
                                         0,       0.0004,  0,
                                         0,       0,        0.00026);

    arm_rbdl.shoulder_yaw_inertia = RBDLMatrix3d( 0.00024,  0,        0,
                                                  0,        0.00004,  0,
                                                  0,        0,        0.00026);

    arm_rbdl.shoulder_pitch_inertia = RBDLMatrix3d(0.00029, 0,        0,
                                                   0,       0.00072,  0,
                                                   0,       0,        0.00056);

    arm_rbdl.elbow_pitch_inertia = RBDLMatrix3d(0.00003,  0,        0,
                                                0,        0.00025,  0,
                                                0,        0,        0.00026);

    arm_rbdl.wrist_pitch_inertia = RBDLMatrix3d(0.00002,  0,        0,
                                                0,        0.00002,  0,
                                                0,        0,        0.00001);

    arm_rbdl.wrist_roll_inertia = RBDLMatrix3d(0.00001,  0,        0,
                                                0,        0.00002,  0,
                                                0,        0,        0.00002);

    arm_rbdl.wrist_yaw_inertia = RBDLMatrix3d(0.00015,  0,        0,
                                                0,        0.00008,  0,
                                                0,        0,        0.000116);

    arm_rbdl.gripper_inertia = RBDLMatrix3d(0.00015,  0,        0,
                                                0,        0.00007,  0,
                                                0,        0,        0.00012);

    // arm_rbdl.base_link = RBDLBody(0.59468, RBDLVector3d(0, 0.00033, 0.03107), arm_rbdl.base_inertia);
    // arm_rbdl.base_joint = RBDLJoint(RBDL::JointType::JointTypeFixed, RBDLVector3d(0,0,0));
    // arm_rbdl.base_id = arm_rbdl.rbdl_model->RBDLModel::AddBody(0, RBDL::Math::Xtrans(RBDLVector3d(0,0,0)), arm_rbdl.base_joint, arm_rbdl.base_link);

    arm_rbdl.shoulder_yaw_link = RBDLBody(0.54333, RBDLVector3d(-0.00021, -0.001515, 0.095825), arm_rbdl.shoulder_yaw_inertia);
    arm_rbdl.shoulder_yaw_joint = RBDLJoint(RBDL::JointType::JointTypeRevolute, RBDLVector3d(0,0,1));
    arm_rbdl.shoulder_yaw_id = arm_rbdl.rbdl_model->RBDLModel::AddBody(0, RBDL::Math::Xtrans(RBDLVector3d(0,0,0)), arm_rbdl.shoulder_yaw_joint, arm_rbdl.shoulder_yaw_link);

    arm_rbdl.shoulder_pitch_link = RBDLBody(0.67378, RBDLVector3d(0.21581, 0.045076, -0.0013), arm_rbdl.shoulder_pitch_inertia);
    arm_rbdl.shoulder_pitch_joint = RBDLJoint(RBDL::JointType::JointTypeRevolute, RBDLVector3d(0,1,0));
    arm_rbdl.shoulder_pitch_id = arm_rbdl.rbdl_model->RBDLModel::AddBody(1, RBDL::Math::Xtrans(RBDLVector3d(0, 0, 0.0981)), arm_rbdl.shoulder_pitch_joint, arm_rbdl.shoulder_pitch_link);

    arm_rbdl.elbow_pitch_link = RBDLBody(0.19195, RBDLVector3d(0.16286, 0.001042, -0.00001), arm_rbdl.elbow_pitch_inertia);
    arm_rbdl.elbow_pitch_joint = RBDLJoint(RBDL::JointType::JointTypeRevolute, RBDLVector3d(0,1,0));
    arm_rbdl.elbow_pitch_id = arm_rbdl.rbdl_model->RBDLModel::AddBody(2, RBDL::Math::Xtrans(RBDLVector3d(0.25,0,0)), arm_rbdl.elbow_pitch_joint, arm_rbdl.elbow_pitch_link);

    arm_rbdl.wrist_pitch_link = RBDLBody(0.083173, RBDLVector3d(0.045085, 0, 0.011658), arm_rbdl.wrist_pitch_inertia);
    arm_rbdl.wrist_pitch_joint = RBDLJoint(RBDL::JointType::JointTypeRevolute, RBDLVector3d(0,1,0));
    arm_rbdl.wrist_pitch_id = arm_rbdl.rbdl_model->RBDLModel::AddBody(3, RBDL::Math::Xtrans(RBDLVector3d(0.25,0,0)), arm_rbdl.wrist_pitch_joint, arm_rbdl.wrist_pitch_link);

    arm_rbdl.wrist_roll_link = RBDLBody(0.083173, RBDLVector3d(0.092842, 0, 0.00009), arm_rbdl.wrist_roll_inertia);
    arm_rbdl.wrist_roll_joint = RBDLJoint(RBDL::JointType::JointTypeRevolute, RBDLVector3d(1,0,0));
    arm_rbdl.wrist_roll_id = arm_rbdl.rbdl_model->RBDLModel::AddBody(4, RBDL::Math::Xtrans(RBDLVector3d(0,0,0)), arm_rbdl.wrist_roll_joint, arm_rbdl.wrist_roll_link);

    arm_rbdl.wrist_yaw_link = RBDLBody(0.35586, RBDLVector3d(0.052329, 0.0001, 0.048909), arm_rbdl.wrist_yaw_inertia);
    arm_rbdl.wrist_yaw_joint = RBDLJoint(RBDL::JointType::JointTypeRevolute, RBDLVector3d(0,0,1));
    arm_rbdl.wrist_yaw_id = arm_rbdl.rbdl_model->RBDLModel::AddBody(5, RBDL::Math::Xtrans(RBDLVector3d(0.1045,0,0)), arm_rbdl.wrist_yaw_joint, arm_rbdl.wrist_yaw_link);

    arm_rbdl.gripper_link = RBDLBody(a_obj_weight, RBDLVector3d(0.0, 0.0, 0.0), arm_rbdl.gripper_inertia);
    arm_rbdl.gripper_joint = RBDLJoint(RBDL::JointType::JointTypeFixed);
    arm_rbdl.gripper_id = arm_rbdl.rbdl_model->RBDLModel::AddBody(6, RBDL::Math::Xtrans(RBDLVector3d(0.135,0,0)), arm_rbdl.gripper_joint, arm_rbdl.gripper_link);

    arm_rbdl.q = RBDLVectorNd::Zero(6);
    arm_rbdl.q_dot = RBDLVectorNd::Zero(6);
    arm_rbdl.q_d_dot = RBDLVectorNd::Zero(6);
    arm_rbdl.q_d_dot_ctc = RBDLVectorNd::Zero(6);
    arm_rbdl.tau_nonlinear = RBDLVectorNd::Zero(6);
    arm_rbdl.tau_inertia = RBDLVectorNd::Zero(6);
    arm_rbdl.ee_x0 = RBDLVectorNd::Zero(6);
    arm_rbdl.ee_x_dot = RBDLVectorNd::Zero(6);

    arm_rbdl.virtual_damping = RBDLVectorNd::Zero(6);
    arm_rbdl.virtual_spring = RBDLVectorNd::Zero(6);
    arm_rbdl.x_desired_d_dot = RBDLVectorNd::Zero(6);

    arm_rbdl.x_ctc_d_dot = RBDLVectorNd::Zero(6);
    arm_rbdl.x_actual = RBDLVectorNd::Zero(6);
    arm_rbdl.x_actual_dot = RBDLVectorNd::Zero(6);
    arm_rbdl.x_desired_dot = RBDLVectorNd::Zero(6);
    arm_rbdl.x_desired_dot_last = RBDLVectorNd::Zero(6);
    arm_rbdl.x_desired = RBDLVectorNd::Zero(6);
    arm_rbdl.x_desired_last = RBDLVectorNd::Zero(6);
    arm_rbdl.x_error = RBDLVectorNd::Zero(6);
    arm_rbdl.x_error_dot = RBDLVectorNd::Zero(6);

    arm_rbdl.jacobian = RBDLMatrixNd::Zero(6,6);
    arm_rbdl.jacobian_prev = RBDLMatrixNd::Zero(6,6);
    arm_rbdl.jacobian_dot = RBDLMatrixNd::Zero(6,6);
    arm_rbdl.jacobian_inverse = RBDLMatrixNd::Zero(6,6);

    arm_rbdl.jacobian_ana = RBDLMatrixNd::Zero(6,6);
    arm_rbdl.jacobian_ana_swap = RBDLMatrixNd::Zero(6,6);
    arm_rbdl.jacobian_ana_prev  = RBDLMatrixNd::Zero(6,6);
    arm_rbdl.jacobian_ana_dot = RBDLMatrixNd::Zero(6,6);
    arm_rbdl.jacobian_ana_inverse = RBDLMatrixNd::Zero(6,6);

    arm_rbdl.geometric_to_analytic = RBDLMatrixNd::Zero(6,6);

    arm_rbdl.inertia_matrix = RBDLMatrixNd::Zero(6,6);

    arm_rbdl.ts_p = RBDLMatrixNd::Zero(6,6);
    arm_rbdl.ts_v = RBDLMatrixNd::Zero(6,6);

    arm_rbdl.ee_ori_act = RBDLMatrix3d::Zero(3,3);
    arm_rbdl.ee_ori_act_trans = RBDLMatrix3d::Zero(3,3);
    
    arm_rbdl.ee_pos_act = RBDLVector3d::Zero(3);
    arm_rbdl.position_desired = RBDLVector3d::Zero(3);
    arm_rbdl.rpy_ee = RBDLVector3d::Zero(3);
    arm_rbdl.rpy_desired = RBDLVector3d::Zero(3);

    W_term << 0.0981,    0,    0, 0,      0,     0,
                   0, 0.25,    0, 0,      0,     0,
                   0,    0, 0.25, 0,      0,     0,
                   0,    0,    0, 0,      0,     0,
                   0,    0,    0, 0, 0.1045,     0,
                   0,    0,    0, 0,      0, 0.135;

    std::cout << "RBDL Initialize function success" << std::endl;
  }
 

  void STArmPlugin::Loop()
  {
    current_time = this->model->GetWorld()->SimTime(); //현재시간
    dt = current_time.Double() - last_update_time.Double(); //현재시간 - 과거시간
    last_update_time = current_time; // dt처리 후 현재시간을 과거시간으로 치환
    
    GetJointPosition();
    GetJointVelocity();
    GetJointAcceleration();
    GetSensorValues();
    SetRBDLVariables();
    ROSMsgPublish();
    PostureGeneration(); 
    GripperControl();
    SetJointTorque();
    if(is_move_rbq3)
    {
      GetRBQ3JointPosition();
      GetRBQ3JointVelocity();
      RBQ3Motion2();
      SetRBQ3JointTorque();
    } 
  }


  void STArmPlugin::GetJointPosition()
  {
    th[0] = this->Joint1->Position(2);
    th[1] = this->Joint2->Position(1);
    th[2] = this->Joint3->Position(1);
    th[3] = this->Joint4->Position(1);
    th[4] = this->Joint5->Position(0);
    th[5] = this->Joint6->Position(2);
    th[6] = this->JointGripperL->Position(1);
    th[7] = this->JointGripperR->Position(1);
  }


  void STArmPlugin::GetJointVelocity()
  {

    // th_dot[0] = this->Joint1->GetVelocity(2);
    // th_dot[1] = this->Joint2->GetVelocity(1);
    // th_dot[2] = this->Joint3->GetVelocity(1);
    // th_dot[3] = this->Joint4->GetVelocity(1);
    // th_dot[4] = this->Joint5->GetVelocity(0);
    // th_dot[5] = this->Joint6->GetVelocity(2);
    // th_dot[6] = this->JointGripperL->GetVelocity(1);
    // th_dot[7] = this->JointGripperR->GetVelocity(1);

    
    for(uint8_t i=0; i<NUM_OF_JOINTS_WITH_TOOL; i++)
    {
      if(dt > 0.0005) th_dot[i] = (th[i] - last_th[i]) / dt; // 왜 0.005초?
      last_th[i] = th[i];
    }

    for(uint8_t i=0; i<NUM_OF_JOINTS_WITH_TOOL; i++) // limit 걸어놓기
    {
      if(abs(th_dot[i]) > JOINT_VEL_LIMIT) th_dot[i] = 0;
    }
  }


  void STArmPlugin::GetJointAcceleration()
  {
    th_d_dot = (th_dot - last_th_dot) / dt;
    last_th_dot = th_dot;

    for(uint8_t i=0; i<NUM_OF_JOINTS_WITH_TOOL; i++)
    {
      if(abs(th_d_dot[i]) > JOINT_VEL_LIMIT) th_dot[i] = 0;
    }

  }


  void STArmPlugin::GetSensorValues()
  {
    auto pose = this->model->WorldPose();
    auto rbq3_base_imu_quat = this->RBQ3BaseImu->Orientation();

    auto xyz = rbq3_base_imu_quat.Euler();
    rbq3_base_imu_rpy = {xyz[0], xyz[1], xyz[2]};
  }


  void STArmPlugin::ROSMsgPublish()
  {
    sensor_msgs::JointState joint_state_msg;
    
    joint_state_msg.header.stamp = ros::Time::now();
    for (uint8_t i=0; i<6; i++)
    {
      joint_state_msg.name.push_back((std::string)joint_names.at(i));
      joint_state_msg.position.push_back((float)(th[i]));
      joint_state_msg.velocity.push_back((float)(th_dot[i]));
      joint_state_msg.effort.push_back((float)joint_torque[i]);
    }
    joint_state_msg.position.push_back(Map(th[6], -0.03, 0, 0, 2));
    pub_joint_state.publish(joint_state_msg); 

    // joint_state_msg.header.stamp = ros::Time::now();
    // for (uint8_t i=0; i<NUM_OF_JOINTS_WITH_TOOL; i++)
    // {
    //   joint_state_msg.name.push_back((std::string)joint_names.at(i));
    //   joint_state_msg.position.push_back((float)(th[i]*RAD2DEG));
    //   joint_state_msg.velocity.push_back((float)(th_dot[i]));
    //   joint_state_msg.effort.push_back((float)joint_torque[i]);
    // }
    // pub_joint_state_deg.publish(joint_state_msg); 

    sensor_msgs::JointState msg;
    msg.header.stamp = ros::Time::now();
    for (uint8_t i=0; i<6; i++)
    {
      msg.name.push_back((std::string)joint_names.at(i));
      msg.position.push_back((float)(ik_th[i]));
      msg.velocity.push_back((float)(ik_current_pose[i]));
    }
    pub_joint_state_ik.publish(msg); 

    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header.stamp = ros::Time::now();
    ee_quaternion = ee_rotation;
    tf_msg.transform.translation.x = ee_position(0);
    tf_msg.transform.translation.y = ee_position(1);
    tf_msg.transform.translation.z = ee_position(2);
    tf_msg.transform.rotation.x = ee_quaternion.x();
    tf_msg.transform.rotation.y = ee_quaternion.y();
    tf_msg.transform.rotation.z = ee_quaternion.z();
    tf_msg.transform.rotation.w = ee_quaternion.w();

    pub_ee_pose.publish(tf_msg); 

    tf_msg.header.stamp = ros::Time::now();
    ref_ee_quaternion = ref_ee_rotation;
    tf_msg.transform.translation.x = ref_ee_position(0);
    tf_msg.transform.translation.y = ref_ee_position(1);
    tf_msg.transform.translation.z = ref_ee_position(2);
    tf_msg.transform.rotation.x = ref_ee_quaternion.x();
    tf_msg.transform.rotation.y = ref_ee_quaternion.y();
    tf_msg.transform.rotation.z = ref_ee_quaternion.z();
    tf_msg.transform.rotation.w = ref_ee_quaternion.w();

    pub_ref_ee_pose.publish(tf_msg); 

    sensor_msgs::JointState rbq3_joint_state_msg;
    
    joint_state_msg.header.stamp = ros::Time::now();
    for (uint8_t i=0; i<12; i++)
    {
      rbq3_joint_state_msg.name.push_back((std::string)rbq3_joint_names.at(i));
      rbq3_joint_state_msg.position.push_back((float)(quad_th[i]));
      rbq3_joint_state_msg.velocity.push_back((float)(quad_th_dot[i]));
      rbq3_joint_state_msg.effort.push_back((float)quad_joint_torque[i]);
    }
    pub_rbq3_joint_state.publish(rbq3_joint_state_msg); 

    std_msgs::Float32MultiArray weight_est_pose_difference_msg;
    for (uint8_t i=0; i<pose_difference.size(); i++)
    {
      weight_est_pose_difference_msg.data.push_back(pose_difference(i));
    }
    pub_weight_est_pose_difference.publish(weight_est_pose_difference_msg);

    std_msgs::Float32 weight_est_estimated_object_weight_msg;
    weight_est_estimated_object_weight_msg.data = estimated_object_weight;
    pub_weight_est_estimated_obj_weight.publish(weight_est_estimated_object_weight_msg);

    //e.e position msg checking
    geometry_msgs::Pose ee_pose_msg;
    ee_pose_msg.position.x = (float)(ee_position[0]);
    ee_pose_msg.position.y = (float)(ee_position[1]);
    ee_pose_msg.position.z = (float)(ee_position[2]);
    ee_pose_msg.orientation.x  = (float)(arm_rbdl.rpy_ee[0]);
    ee_pose_msg.orientation.y  = (float)(arm_rbdl.rpy_ee[1]);
    ee_pose_msg.orientation.z  = (float)(arm_rbdl.rpy_ee[2]);
    pub_EE_pose.publish(ee_pose_msg);


    //virtual spring torque
    msg_virtual_spring_torque_0.data = tau_vs[0];
    pub_virtual_spring_torque_0.publish(msg_virtual_spring_torque_0);
    msg_virtual_spring_torque_1.data = tau_vs[1];
    pub_virtual_spring_torque_1.publish(msg_virtual_spring_torque_1);
    msg_virtual_spring_torque_2.data = tau_vs[2];
    pub_virtual_spring_torque_2.publish(msg_virtual_spring_torque_2);

    //limited torque
    msg_limited_torque_2.data = tau_limit[2];
    pub_limited_torque_2.publish(msg_limited_torque_2);


    //actual EE eulerangle
    msg_ee_pi.data = psi;
    pub_ee_pi.publish(msg_ee_psi);
    msg_ee_theta.data = theta;
    pub_ee_theta.publish(msg_ee_theta);
    msg_ee_psi.data = pi;
    pub_ee_psi.publish(msg_ee_pi);




  }


  void STArmPlugin::PostureGeneration()
  {
    if (control_mode == IDLE) Idle();
    else if (control_mode == Motion_1) Motion1(); 
    else if (control_mode == Motion_2) Motion2();
    else if (control_mode == Motion_3) Motion3();
    else if (control_mode == Motion_4) Motion4();
    else if (control_mode == Motion_5) Motion5();
    else if (control_mode == Motion_6) Motion6();
    else if (control_mode == Motion_7) Motion7();
    else if (control_mode == Motion_8) Motion8();
    else if (control_mode == Motion_9) Motion9();
    else Idle();
  }


  void STArmPlugin::SetJointTorque() //각 조인트에 계산된 토크를 넣어준다. N/m, Loop중 마지막 실행함수
  {
    this->Joint1->SetForce(2, joint_torque(0)); // z축, yaw
    this->Joint2->SetForce(1, joint_torque(1)); // y축, pitch
    this->Joint3->SetForce(1, joint_torque(2)); // y축, pitch
    this->Joint4->SetForce(1, joint_torque(3)); // y축, pitch
    this->Joint5->SetForce(0, joint_torque(4)); // x축, roll
    this->Joint6->SetForce(2, joint_torque(5)); // z축, yaw
    this->JointGripperL->SetForce(1, joint_torque(6)); // y축, prismatic
    this->JointGripperR->SetForce(1, joint_torque(7)); // y축, prismatic
  }


  void STArmPlugin::SwitchMode(const std_msgs::Int32Ptr & msg)
  {
    cnt = 0;
    if      (msg -> data == 0) control_mode = IDLE;
    else if (msg -> data == 1) control_mode = Motion_1;
    else if (msg -> data == 2) control_mode = Motion_2;  
    else if (msg -> data == 3) control_mode = Motion_3;  
    else if (msg -> data == 4) control_mode = Motion_4; 
    else if (msg -> data == 5) control_mode = Motion_5;   
    else if (msg -> data == 6) control_mode = Motion_6;   
    else if (msg -> data == 7) control_mode = Motion_7;
    else if (msg -> data == 8) control_mode = Motion_8;
    else if (msg -> data == 9) control_mode = Motion_9;
    else                       control_mode = IDLE;    
  }


  void STArmPlugin::SwitchOnAddingEstimatedObjWeightToRBDL(const std_msgs::Int32Ptr & msg)
  {
    if      (msg -> data == 0) is_start_estimation = false;
    else if (msg -> data == 1) is_start_estimation = true;
  }


  void STArmPlugin::Idle()
  {
    gain_p_joint_space_idle << 200, 200, 200, 100, 100, 100, 35, 35;
    // gain_d_joint_space_idle << 20, 20, 20, 10, 10, 10, 3, 3;
    // gain_d_joint_space_idle << 1, 1, 1, 1, 1, 1, 1, 1;
    gain_d_joint_space_idle << 0, 0, 0, 0, 0, 0, 0, 0;
    // gain_d_joint_space_idle = gain_p_joint_space_idle * 0.1;
    step_time = 4; 
    
    cnt_time = cnt * inner_dt;
    trajectory = 0.5 * (1 - cos(PI * (cnt_time / step_time)));
    
    if(cnt_time <= step_time)
    {
      ref_th[0] =   0 * trajectory * DEG2RAD;
      ref_th[1] = -60 * trajectory * DEG2RAD;
      ref_th[2] =  90 * trajectory * DEG2RAD;
      ref_th[3] = -30 * trajectory * DEG2RAD;
      ref_th[4] =   0 * trajectory * DEG2RAD;
      ref_th[5] =   0 * trajectory * DEG2RAD;
      //ref_th[6] =  -0.03 * trajectory;
      //ref_th[7] =  -0.03 * trajectory;
    }

    // joint_torque = gain_p_joint_space_idle * (ref_th - th) - gain_d_joint_space_idle * th_dot;

    for (uint8_t i=0; i<6; i++)
    {
      joint_torque[i] = gain_p_joint_space_idle[i] * (ref_th[i] - th[i]) - gain_d_joint_space_idle[i] * th_dot[i];
    }

    cnt++;
  }

  // Task Space PD Control ||	Infinity Drawer || Gravity Compensation
  void STArmPlugin::Motion1()
  { 
    step_time = 3;
    
    cnt_time = cnt*inner_dt;   

    gain_p = gain_p_task_space;
    gain_w = gain_w_task_space; 
    gain_r << 1, 1, 1, 1, 1, 1; //adjust GC intensity


    threshold << deg, deg, deg, deg, deg, deg;
    joint_limit <<   (180/180)*PI,   (180/180)*PI,  (150/180)*PI,  1.57,  1.57,  1.57, 
                    (-180/180)*PI,           -PI,  (90/180)*PI, -1.57, -1.57, -1.57;
    // joint_limit <<   (180/180)*PI,   (35/180)*PI,  (164.8/180)*PI,  1.57,  1.57,  1.57, 
    //                 (-135/180)*PI,           -PI,      (5/180)*PI, -1.57, -1.57, -1.57;
    // threshold << 0.2, 0.1, 0.1, 0.1, 0.1, 0.1; 
    // joint_limit << 3.14,     0,  2.8,  1.87,  1.57,  1.57,
    //                 -3.14, -3.14, -0.3, -1.27, -1.57, -1.57;

    A0 << 1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 1, 0,
          0, 0, 0, 1;
    A1 << cos(th[0]), 0, -sin(th[0]), 0,
          sin(th[0]), 0, cos(th[0]), 0,
          0, -1, 0, L1,
          0, 0, 0, 1;
    A2 << cos(th[1]), -sin(th[1]), 0, L2*cos(th[1]),
          sin(th[1]), cos(th[1]), 0, L2*sin(th[1]),
          0, 0, 1, 0, 
          0, 0, 0, 1;
    A3 << cos(th[2]), -sin(th[2]), 0, L3*cos(th[2]), 
          sin(th[2]), cos(th[2]), 0, L3*sin(th[2]), 
          0, 0, 1, 0,
          0, 0, 0, 1;
    A4 << sin(th[3]), 0, cos(th[3]), 0,
          -cos(th[3]), 0, sin(th[3]), 0,
          0, -1, 0, 0,
          0, 0, 0, 1;
    A5 << -sin(th[4]), 0, cos(th[4]), 0,
          cos(th[4]), 0, sin(th[4]), 0,
          0, 1, 0, L5,
          0, 0, 0, 1;
    A6 << -sin(th[5]), -cos(th[5]), 0, -L6*sin(th[5]),
          cos(th[5]), -sin(th[5]), 0, L6*cos(th[5]),
          0, 0, 1, 0, 
          0, 0, 0, 1;
          
    T00 = A0;
    T01 = T00*A1;
    T02 = T01*A2;
    T03 = T02*A3;
    T04 = T03*A4;
    T05 = T04*A5;
    T06 = T05*A6;
  
    a0 << T00(0,2), T00(1,2), T00(2,2);
    a1 << T01(0,2), T01(1,2), T01(2,2);
    a2 << T02(0,2), T02(1,2), T02(2,2);
    a3 << T03(0,2), T03(1,2), T03(2,2);
    a4 << T04(0,2), T04(1,2), T04(2,2);
    a5 << T05(0,2), T05(1,2), T05(2,2);

    P6_P0 << T06(0,3)-T00(0,3), T06(1,3)-T00(1,3), T06(2,3)-T00(2,3);
    P6_P1 << T06(0,3)-T01(0,3), T06(1,3)-T01(1,3), T06(2,3)-T01(2,3);
    P6_P2 << T06(0,3)-T02(0,3), T06(1,3)-T02(1,3), T06(2,3)-T02(2,3);
    P6_P3 << T06(0,3)-T03(0,3), T06(1,3)-T03(1,3), T06(2,3)-T03(2,3);
    P6_P4 << T06(0,3)-T04(0,3), T06(1,3)-T04(1,3), T06(2,3)-T04(2,3);
    P6_P5 << T06(0,3)-T05(0,3), T06(1,3)-T05(1,3), T06(2,3)-T05(2,3);

    J1 << a0.cross(P6_P0), a0;
    J2 << a1.cross(P6_P1), a1;
    J3 << a2.cross(P6_P2), a2;
    J4 << a3.cross(P6_P3), a3;
    J5 << a4.cross(P6_P4), a4;
    J6 << a5.cross(P6_P5), a5;

    Jacobian << J1, J2, J3, J4, J5, J6;

    ee_position << T06(0,3), T06(1,3), T06(2,3);
    
    // if (cnt<1) initial_ee_position << ee_position(0), ee_position(1), ee_position(2);
    if (cnt<1) initial_ee_position << 0.4, 0, 0.3;

    if(cnt_time <= step_time*100)
    { 
      ref_ee_position(0) = initial_ee_position(0) - 0.2*abs(sin(PI/2*(cnt_time/step_time)));
      ref_ee_position(1) = initial_ee_position(1) - 0.3*sin(PI/2*(cnt_time/step_time));
      ref_ee_position(2) = initial_ee_position(2) + 0.2*sin(PI*(cnt_time/step_time));
      ref_ee_quaternion.w() = 1;
      ref_ee_quaternion.x() = 0;
      ref_ee_quaternion.y() = 0;
      ref_ee_quaternion.z() = 0;
    }

    ee_force(0) = gain_p(0) * (ref_ee_position(0) - ee_position(0));
    ee_force(1) = gain_p(1) * (ref_ee_position(1) - ee_position(1));
    ee_force(2) = gain_p(2) * (ref_ee_position(2) - ee_position(2));

    ee_rotation = T06.block<3,3>(0,0);
    ee_rotation_x = ee_rotation.block<3,1>(0,0); 
    ee_rotation_y = ee_rotation.block<3,1>(0,1); 
    ee_rotation_z = ee_rotation.block<3,1>(0,2);

    //for MSG Publisher
    // arm_rbdl.rpy_ee = ee_rotation.eulerAngles(2,1,0);
    // Get the Euler Angle z-y-x rotation matrix
    pi = atan2(ee_rotation(1,0),ee_rotation(0,0));
    theta = atan2(- ee_rotation(2,0), cos(pi)*ee_rotation(0,0) + sin(pi)*ee_rotation(1,0));
    psi = atan2(sin(pi)*ee_rotation(0,2) - cos(pi)*ee_rotation(1,2), -sin(pi)*ee_rotation(0,1) + cos(pi)*ee_rotation(1,1));

    ref_ee_rotation = ref_ee_quaternion.normalized().toRotationMatrix();    

    ref_ee_rotation_x = ref_ee_rotation.block<3,1>(0,0); 
    ref_ee_rotation_y = ref_ee_rotation.block<3,1>(0,1); 
    ref_ee_rotation_z = ref_ee_rotation.block<3,1>(0,2);
    ee_orientation_error = ee_rotation_x.cross(ref_ee_rotation_x) + ee_rotation_y.cross(ref_ee_rotation_y) + ee_rotation_z.cross(ref_ee_rotation_z);
    ee_momentum << gain_w(0) * ee_orientation_error(0), gain_w(1) * ee_orientation_error(1), gain_w(2) * ee_orientation_error(2);

    virtual_spring << ee_force(0), ee_force(1), ee_force(2), ee_momentum(0), ee_momentum(1), ee_momentum(2);

    tau_vs = Jacobian.transpose() * virtual_spring;
    
    RBDL::NonlinearEffects(*arm_rbdl.rbdl_model, arm_rbdl.q, arm_rbdl.q_dot, arm_rbdl.tau_nonlinear, NULL);

    for(uint8_t i = 0; i < 6; i++)
    { 
      tau_gravity_compensation(i) = gain_r(i)*arm_rbdl.tau_nonlinear(i);
    }

    for(uint8_t i=0; i<6; i++)
    {
      if((abs(joint_limit(0,i)-th(i)) <= threshold(i)) && (tau_vs(i) > 0))
      {
        // joint_torque(i) = (abs(joint_limit(0,i)-th(i)))/threshold(i)*tau(i);
        std::cout << "limit_1" << std::endl;
        tau_limit(i) = (1 - cos(PI * abs(joint_limit(0,i)-th(i)) / threshold(i))) / 2 * tau_vs(i);
        joint_torque(i) = tau_limit(i) + tau_gravity_compensation(i);
      }
      else
      {
        joint_torque(i) = tau_vs(i) + tau_gravity_compensation(i);
      }

      if((abs(joint_limit(1,i)+th(i)) <= threshold(i)) && (tau_vs(i) < 0))
      {
        // joint_torque(i) = (abs(joint_limit(1,i)-th(i)))/threshold(i)*tau(i);
        std::cout << "limit_2" << std::endl;
        tau_limit(i) = (1 - cos(PI * abs(joint_limit(1,i)-th(i)) / threshold(i))) / 2 * tau_vs(i);
        joint_torque(i) = tau_limit(i) + tau_gravity_compensation(i);
      }
      else
      {
        joint_torque(i) = tau_vs(i) + tau_gravity_compensation(i);
      }
    }



    cnt++;

    // InverseSolverUsingJacobian(ref_ee_position, ref_ee_rotation);
  }

  // Task Space PD Control || anoter trajectory
  void STArmPlugin::Motion2()
  {
    step_time = 3;
    
    cnt_time = cnt*inner_dt;   

    gain_p = gain_p_task_space;
    gain_w = gain_w_task_space; 
    gain_r << 1, 1, 1, 1, 1, 1; //adjust GC intensity


    threshold << deg, deg, deg, deg, deg, deg;
    joint_limit <<   (180/180)*PI,   (180/180)*PI,  (150/180)*PI,  1.57,  1.57,  1.57, 
                    (-180/180)*PI,           -PI,  (90/180)*PI, -1.57, -1.57, -1.57;
    // joint_limit <<   (180/180)*PI,   (35/180)*PI,  (164.8/180)*PI,  1.57,  1.57,  1.57, 
    //                 (-135/180)*PI,           -PI,      (5/180)*PI, -1.57, -1.57, -1.57;
    // threshold << 0.2, 0.1, 0.1, 0.1, 0.1, 0.1; 
    // joint_limit << 3.14,     0,  2.8,  1.87,  1.57,  1.57,
    //                 -3.14, -3.14, -0.3, -1.27, -1.57, -1.57;

    A0 << 1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 1, 0,
          0, 0, 0, 1;
    A1 << cos(th[0]), 0, -sin(th[0]), 0,
          sin(th[0]), 0, cos(th[0]), 0,
          0, -1, 0, L1,
          0, 0, 0, 1;
    A2 << cos(th[1]), -sin(th[1]), 0, L2*cos(th[1]),
          sin(th[1]), cos(th[1]), 0, L2*sin(th[1]),
          0, 0, 1, 0, 
          0, 0, 0, 1;
    A3 << cos(th[2]), -sin(th[2]), 0, L3*cos(th[2]), 
          sin(th[2]), cos(th[2]), 0, L3*sin(th[2]), 
          0, 0, 1, 0,
          0, 0, 0, 1;
    A4 << sin(th[3]), 0, cos(th[3]), 0,
          -cos(th[3]), 0, sin(th[3]), 0,
          0, -1, 0, 0,
          0, 0, 0, 1;
    A5 << -sin(th[4]), 0, cos(th[4]), 0,
          cos(th[4]), 0, sin(th[4]), 0,
          0, 1, 0, L5,
          0, 0, 0, 1;
    A6 << -sin(th[5]), -cos(th[5]), 0, -L6*sin(th[5]),
          cos(th[5]), -sin(th[5]), 0, L6*cos(th[5]),
          0, 0, 1, 0, 
          0, 0, 0, 1;
          
    T00 = A0;
    T01 = T00*A1;
    T02 = T01*A2;
    T03 = T02*A3;
    T04 = T03*A4;
    T05 = T04*A5;
    T06 = T05*A6;
  
    a0 << T00(0,2), T00(1,2), T00(2,2);
    a1 << T01(0,2), T01(1,2), T01(2,2);
    a2 << T02(0,2), T02(1,2), T02(2,2);
    a3 << T03(0,2), T03(1,2), T03(2,2);
    a4 << T04(0,2), T04(1,2), T04(2,2);
    a5 << T05(0,2), T05(1,2), T05(2,2);

    P6_P0 << T06(0,3)-T00(0,3), T06(1,3)-T00(1,3), T06(2,3)-T00(2,3);
    P6_P1 << T06(0,3)-T01(0,3), T06(1,3)-T01(1,3), T06(2,3)-T01(2,3);
    P6_P2 << T06(0,3)-T02(0,3), T06(1,3)-T02(1,3), T06(2,3)-T02(2,3);
    P6_P3 << T06(0,3)-T03(0,3), T06(1,3)-T03(1,3), T06(2,3)-T03(2,3);
    P6_P4 << T06(0,3)-T04(0,3), T06(1,3)-T04(1,3), T06(2,3)-T04(2,3);
    P6_P5 << T06(0,3)-T05(0,3), T06(1,3)-T05(1,3), T06(2,3)-T05(2,3);

    J1 << a0.cross(P6_P0), a0;
    J2 << a1.cross(P6_P1), a1;
    J3 << a2.cross(P6_P2), a2;
    J4 << a3.cross(P6_P3), a3;
    J5 << a4.cross(P6_P4), a4;
    J6 << a5.cross(P6_P5), a5;

    Jacobian << J1, J2, J3, J4, J5, J6;

    ee_position << T06(0,3), T06(1,3), T06(2,3);
    
    // if (cnt<1) initial_ee_position << ee_position(0), ee_position(1), ee_position(2);
    if (cnt<1) initial_ee_position << 0.4, 0, 0.3;

    if(cnt_time <= step_time*100)
    { 
      ref_ee_position(0) = initial_ee_position(0) - 0.1*0.5*(-1+cos(PI*(cnt_time/step_time)));
      ref_ee_position(1) = initial_ee_position(1)*(sin(0.5*PI*(cnt_time/step_time)));
      ref_ee_position(2) = initial_ee_position(2);
      ref_ee_quaternion.w() = 1;
      ref_ee_quaternion.x() = 0;
      ref_ee_quaternion.y() = 0;
      ref_ee_quaternion.z() = 0;
    }

    ee_force(0) = gain_p(0) * (ref_ee_position(0) - ee_position(0));
    ee_force(1) = gain_p(1) * (ref_ee_position(1) - ee_position(1));
    ee_force(2) = gain_p(2) * (ref_ee_position(2) - ee_position(2));

    ee_rotation = T06.block<3,3>(0,0);
    ee_rotation_x = ee_rotation.block<3,1>(0,0); 
    ee_rotation_y = ee_rotation.block<3,1>(0,1); 
    ee_rotation_z = ee_rotation.block<3,1>(0,2);

    //for MSG Publisher
    arm_rbdl.rpy_ee = ee_rotation.eulerAngles(2,1,0);

    ref_ee_rotation = ref_ee_quaternion.normalized().toRotationMatrix();    

    ref_ee_rotation_x = ref_ee_rotation.block<3,1>(0,0); 
    ref_ee_rotation_y = ref_ee_rotation.block<3,1>(0,1); 
    ref_ee_rotation_z = ref_ee_rotation.block<3,1>(0,2);
    ee_orientation_error = ee_rotation_x.cross(ref_ee_rotation_x) + ee_rotation_y.cross(ref_ee_rotation_y) + ee_rotation_z.cross(ref_ee_rotation_z);
    ee_momentum << gain_w(0) * ee_orientation_error(0), gain_w(1) * ee_orientation_error(1), gain_w(2) * ee_orientation_error(2);

    virtual_spring << ee_force(0), ee_force(1), ee_force(2), ee_momentum(0), ee_momentum(1), ee_momentum(2);

    tau_vs = Jacobian.transpose() * virtual_spring;
    
    RBDL::NonlinearEffects(*arm_rbdl.rbdl_model, arm_rbdl.q, arm_rbdl.q_dot, arm_rbdl.tau_nonlinear, NULL);

    for(uint8_t i = 0; i < 6; i++)
    { 
      tau_gravity_compensation(i) = gain_r(i)*arm_rbdl.tau_nonlinear(i);
    }

    for(uint8_t i=0; i<6; i++)
    {

      if (th(i) > joint_limit(0, i) - threshold(i) && tau(i) > 0 || th(i) < joint_limit(1, i) + threshold(i) && tau(i) < 0)
        joint_torque(i) = tau_gravity_compensation(i);
      else
        joint_torque(i) = tau_vs(i) + tau_gravity_compensation(i);
    }

    cnt++;
  }

  //	HMD Virtual Box follower
  void STArmPlugin::Motion3()
  {
    std::cout << "Checked motion3" << std::endl;

    unit_6 << 1, 0, 0, 0, 0, 0,
              0, 1, 0, 0, 0, 0,
              0, 0, 1, 0, 0, 0,
              0, 0, 0, 1, 0, 0,
              0, 0, 0, 0, 1, 0,
              0, 0, 0, 0, 0, 1;

    gain_p = gain_p_task_space;
    gain_w = gain_w_task_space; 
    gain_r << 1, 1, 1, 1, 1, 1; //adjust GC intensity

    // gain_p << 100, 100, 100;
    gain_d << 5, 5, 5;
    // gain_w << 10, 10, 10;

    // threshold << deg, deg, deg, deg, deg, deg;  // threshold : 문지반, 입구, 한계치
    // joint_limit <<   (180/180)*PI,   (35/180)*PI,  (164.8/180)*PI,  1.57,  1.57,  1.57, 
    //                 (-135/180)*PI,           -PI,      (5/180)*PI, -1.57, -1.57, -1.57;

    threshold << 0.2, 0.1, 0.1, 0.1, 0.1, 0.1; // limit threshold angle
    joint_limit << 3.14,     0,  2.8,  1.87,  1.57,  1.57,
                  -3.14, -3.14, -0.3, -1.27, -1.57, -1.57;
                  
    gain_r << 1, 0.3, 0.5, 0.5, 0.5, 0.5; // affects joint limit control

    // threshold << 0.2, 0.1, 0.1, 0.1, 0.1, 0.1; 
    // joint_limit << 3.14,     0,  2.8,  1.87,  1.57,  1.57,
    //                 -3.14, -3.14, -0.3, -1.27, -1.57, -1.57;
    
    cnt_time = cnt * dt;   

    A0 << 1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 1, 0,
          0, 0, 0, 1;
    A1 << cos(th[0]), 0, -sin(th[0]), 0,
          sin(th[0]), 0, cos(th[0]), 0,
          0, -1, 0, L1,
          0, 0, 0, 1;
    A2 << cos(th[1]), -sin(th[1]), 0, L2*cos(th[1]),
          sin(th[1]), cos(th[1]), 0, L2*sin(th[1]),
          0, 0, 1, 0, 
          0, 0, 0, 1;
    A3 << cos(th[2]), -sin(th[2]), 0, L3*cos(th[2]), 
          sin(th[2]), cos(th[2]), 0, L3*sin(th[2]), 
          0, 0, 1, 0,
          0, 0, 0, 1;
    A4 << sin(th[3]), 0, cos(th[3]), 0,
          -cos(th[3]), 0, sin(th[3]), 0,
          0, -1, 0, 0,
          0, 0, 0, 1;
    A5 << -sin(th[4]), 0, cos(th[4]), 0,
          cos(th[4]), 0, sin(th[4]), 0,
          0, 1, 0, L5,
          0, 0, 0, 1;
    A6 << -sin(th[5]), -cos(th[5]), 0, -L6*sin(th[5]),
          cos(th[5]), -sin(th[5]), 0, L6*cos(th[5]),
          0, 0, 1, 0, 
          0, 0, 0, 1;          
          
    T00 = A0;
    T01 = T00*A1;
    T02 = T01*A2;
    T03 = T02*A3;
    T04 = T03*A4;
    T05 = T04*A5;
    T06 = T05*A6;
  
    a0 << T00(0,2), T00(1,2), T00(2,2);
    a1 << T01(0,2), T01(1,2), T01(2,2);
    a2 << T02(0,2), T02(1,2), T02(2,2);
    a3 << T03(0,2), T03(1,2), T03(2,2);
    a4 << T04(0,2), T04(1,2), T04(2,2);
    a5 << T05(0,2), T05(1,2), T05(2,2);

    P6_P0 << T06(0,3)-T00(0,3), T06(1,3)-T00(1,3), T06(2,3)-T00(2,3);
    P6_P1 << T06(0,3)-T01(0,3), T06(1,3)-T01(1,3), T06(2,3)-T01(2,3);
    P6_P2 << T06(0,3)-T02(0,3), T06(1,3)-T02(1,3), T06(2,3)-T02(2,3);
    P6_P3 << T06(0,3)-T03(0,3), T06(1,3)-T03(1,3), T06(2,3)-T03(2,3);
    P6_P4 << T06(0,3)-T04(0,3), T06(1,3)-T04(1,3), T06(2,3)-T04(2,3);
    P6_P5 << T06(0,3)-T05(0,3), T06(1,3)-T05(1,3), T06(2,3)-T05(2,3);

    J1 << a0.cross(P6_P0), a0;
    J2 << a1.cross(P6_P1), a1;
    J3 << a2.cross(P6_P2), a2;
    J4 << a3.cross(P6_P3), a3;
    J5 << a4.cross(P6_P4), a4;
    J6 << a5.cross(P6_P5), a5;

    Jacobian << J1, J2, J3, J4, J5, J6;

    ee_position << T06(0,3), T06(1,3), T06(2,3);
    if (cnt<1) pre_ee_position = ee_position; 
    ee_velocity = (ee_position - pre_ee_position) / dt;
    pre_ee_position = ee_position;

    ref_ee_position = hmd_position;
    ref_ee_rotation = hmd_quaternion.normalized().toRotationMatrix();

    ee_position_error(0) = ref_ee_position(0) - ee_position(0);
    ee_position_error(1) = ref_ee_position(1) - ee_position(1);
    ee_position_error(2) = ref_ee_position(2) - ee_position(2);

    ee_force(0) = gain_p(0) * (ref_ee_position(0) - ee_position(0)) - gain_d(0) * ee_velocity(0);
    ee_force(1) = gain_p(1) * (ref_ee_position(1) - ee_position(1)) - gain_d(1) * ee_velocity(1);
    ee_force(2) = gain_p(2) * (ref_ee_position(2) - ee_position(2)) - gain_d(2) * ee_velocity(2);

    ee_rotation = T06.block<3,3>(0,0);
    ee_rotation_x = ee_rotation.block<3,1>(0,0); 
    ee_rotation_y = ee_rotation.block<3,1>(0,1); 
    ee_rotation_z = ee_rotation.block<3,1>(0,2);


    ref_ee_rotation_x = ref_ee_rotation.block<3,1>(0,0); 
    ref_ee_rotation_y = ref_ee_rotation.block<3,1>(0,1); 
    ref_ee_rotation_z = ref_ee_rotation.block<3,1>(0,2);

    ee_orientation_error = ee_rotation_x.cross(ref_ee_rotation_x) 
                        + ee_rotation_y.cross(ref_ee_rotation_y) 
                        + ee_rotation_z.cross(ref_ee_rotation_z);
    
    ee_momentum << gain_w(0) * ee_orientation_error(0), 
                  gain_w(1) * ee_orientation_error(1), 
                  gain_w(2) * ee_orientation_error(2);

    virtual_spring << ee_force(0), ee_force(1), ee_force(2), ee_momentum(0), ee_momentum(1), ee_momentum(2);


    // experiment //
    estimation_error << ee_position_error, ee_orientation_error;

    // Error Damped Pseudo inverse(E-DPI)
    Jacobian_tp = Jacobian.transpose();
    estimation_error_innerproduct = 1/2 * estimation_error.dot(estimation_error);
    J_for_dpi = Jacobian*Jacobian_tp + estimation_error_innerproduct*unit_6 + W_term*0.001;
    // J_for_dpi = Jacobian*Jacobian_tp + W_term*0.001;
    J_inverse_for_dpi = J_for_dpi.inverse();
    Jacobian_e_dpi = Jacobian_tp*J_inverse_for_dpi;


    tau = Jacobian_e_dpi * virtual_spring;
    //////////
    // tau =  Jacobian.transpose() * virtual_spring;

    SetRBDLVariables();

    RBDL::NonlinearEffects(*arm_rbdl.rbdl_model, arm_rbdl.q, arm_rbdl.q_dot, arm_rbdl.tau_nonlinear, NULL);
    // RBDL::InverseDynamics(*arm_rbdl.rbdl_model, arm_rbdl.q, arm_rbdl.q_dot, arm_rbdl.q_d_dot, arm_rbdl.tau, NULL);
    for(uint8_t i = 0; i < 6; i++)
    {
      tau_gravity_compensation(i) = arm_rbdl.tau_nonlinear(i);
    }

    for(uint8_t i=0; i<6; i++)
    {
      // if(th(i)>joint_limit(0,i)-threshold(i) && tau(i)>0 || th(i)<joint_limit(1,i)+threshold(i) && tau(i)<0)
      //   joint_torque(i) = gain_r(i)*tau_gravity_compensation(i);
      // else
        joint_torque(i) = tau(i) + gain_r(i)*tau_gravity_compensation(i); 
    }


    // tau_vs = Jacobian.transpose() * virtual_spring;

    // RBDL::NonlinearEffects(*arm_rbdl.rbdl_model, arm_rbdl.q, arm_rbdl.q_dot, arm_rbdl.tau_nonlinear, NULL);
    // // RBDL::InverseDynamics(*arm_rbdl.rbdl_model, arm_rbdl.q, arm_rbdl.q_dot, arm_rbdl.q_d_dot, arm_rbdl.tau, NULL);

    // for(uint8_t i = 0; i < 6; i++)
    // { 
    //   tau_gravity_compensation(i) = gain_r(i)*arm_rbdl.tau_nonlinear(i);
    //   tau(i) =  tau_vs(i) + tau_gravity_compensation(i);
    // }

    

    // for(uint8_t i=0; i<6; i++)
    // {
    //   if((abs(joint_limit(0,i)-th(i)) <= threshold(i)) && (tau(i) > 0))
    //   {
    //     // joint_torque(i) = (abs(joint_limit(0,i)-th(i)))/threshold(i)*tau(i);
    //     joint_torque(i) = (1 - cos(PI * abs(joint_limit(0,i)-th(i)) / threshold(i))) / 2 * tau_vs(i) + tau_gravity_compensation(i);
    //   }
    //   else if((abs(joint_limit(1,i)-th(i)) <= threshold(i)) && (tau(i) < 0))
    //   {
    //     // joint_torque(i) = (abs(joint_limit(1,i)-th(i)))/threshold(i)*tau(i);
    //     joint_torque(i) = (1 - cos(PI * abs(joint_limit(1,i)-th(i)) / threshold(i))) / 2 * tau_vs(i) + tau_gravity_compensation(i);
    //   }
    //   else
    //   {
    //     joint_torque(i) = tau(i);
    //   }
    // }

    cnt++;
  }

  //	IK based Infinity Drawer || Gravity Compensation
  void STArmPlugin::Motion4()
  { 
    step_time = 3;
    
    cnt_time = cnt*inner_dt;   

    gain_r << 1, 1, 1, 1, 1, 1; //adjust GC intensity

    threshold << 0.2, 0.1, 0.1, 0.1, 0.1, 0.1; 
    joint_limit << 3.14,     0,  2.8,  1.87,  1.57,  1.57,
                    -3.14, -3.14, -0.3, -1.27, -1.57, -1.57;

    //for MSG Publisher
    arm_rbdl.ee_pos_act = RBDL::CalcBodyToBaseCoordinates(*arm_rbdl.rbdl_model, arm_rbdl.q, arm_rbdl.gripper_id, RBDLVector3d(0,0,0), true);  //Position 
    arm_rbdl.ee_ori_act = RBDL::CalcBodyWorldOrientation(*arm_rbdl.rbdl_model, arm_rbdl.q, arm_rbdl.gripper_id, true);  //Orientation
    ee_position = arm_rbdl.ee_pos_act;
    arm_rbdl.ee_ori_act_trans = arm_rbdl.ee_ori_act;//.transpose();

    // Get the Euler Angle z-y-x rotation matrix
    pi = atan2(arm_rbdl.ee_ori_act_trans(1,0),arm_rbdl.ee_ori_act_trans(0,0));
    theta = atan2(- arm_rbdl.ee_ori_act_trans(2,0), cos(pi)*arm_rbdl.ee_ori_act_trans(0,0) + sin(pi)*arm_rbdl.ee_ori_act_trans(1,0));
    psi = atan2(sin(pi)*arm_rbdl.ee_ori_act_trans(0,2) - cos(pi)*arm_rbdl.ee_ori_act_trans(1,2), -sin(pi)*arm_rbdl.ee_ori_act_trans(0,1) + cos(pi)*arm_rbdl.ee_ori_act_trans(1,1));

    if (cnt<1) initial_ee_position << 0.4, 0, 0.3;

    if(cnt_time <= step_time*100)
    { 
      ref_ee_position(0) = initial_ee_position(0) - 0.2*abs(sin(PI/2*(cnt_time/step_time)));
      ref_ee_position(1) = initial_ee_position(1) - 0.3*sin(PI/2*(cnt_time/step_time));
      ref_ee_position(2) = initial_ee_position(2) + 0.2*sin(PI*(cnt_time/step_time));
      ref_ee_quaternion.w() = 1;
      ref_ee_quaternion.x() = 0;
      ref_ee_quaternion.y() = 0;
      ref_ee_quaternion.z() = 0;
    }
    ref_ee_rotation = ref_ee_quaternion.normalized().toRotationMatrix();  

    InverseSolverUsingJacobian(ref_ee_position, ref_ee_rotation);

    RBDL::NonlinearEffects(*arm_rbdl.rbdl_model, arm_rbdl.q, arm_rbdl.q_dot, arm_rbdl.tau_nonlinear, NULL);
    for(uint8_t i = 0; i < 6; i++)
    {
      tau_rbdl(i) = arm_rbdl.tau_nonlinear(i);
    }

    for(uint8_t i=0; i<6; i++) 
    {
      tau_viscous_damping[i] = gain_d_joint_space[i] * th_dot[i]; 
      tau_gravity_compensation[i] = tau_rbdl[i] * gain_r[i];
      tau[i] = gain_p_joint_space[i] * (ik_th[i] - th[i]);
    }

    for(uint8_t i=0; i<6; i++){
      if(th(i) > joint_limit(0,i) - threshold(i) && tau(i) > 0 || th(i) < joint_limit(1,i) + threshold(i) && tau(i) < 0)
      {
        joint_torque[i] = tau_gravity_compensation[i] - tau_viscous_damping[i];
      }
      else
      {
        joint_torque[i] = tau_gravity_compensation[i] - tau_viscous_damping[i] + tau[i]; 
      } 
    }

    cnt++;
  }

  //	COM based HMD orientation follower
  void STArmPlugin::Motion5()
  {
    gain_p << 700, 700, 700; 
    gain_w << 5, 5, 5;
    threshold << 0.2, 0.2, 0.2, 0.2, 0.2, 0.2; 
    joint_limit << 3.14,     0,  2.8,  1.87,  1.57,  1.57,
                  -3.14, -3.14, -0.3, -1.27, -1.57, -1.57;
    gain_r << 0.5, 0.3, 0.3, 0.3, 0.3, 0.3;

    step_time = 6;
    
    cnt_time = cnt*inner_dt;   

    A0 << 1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 1, 0,
          0, 0, 0, 1;
    A1 << cos(th[0]), 0, -sin(th[0]), 0,
          sin(th[0]), 0, cos(th[0]), 0,
          0, -1, 0, L1,
          0, 0, 0, 1;
    A2 << cos(th[1]), -sin(th[1]), 0, L2*cos(th[1]),
          sin(th[1]), cos(th[1]), 0, L2*sin(th[1]),
          0, 0, 1, 0, 
          0, 0, 0, 1;
    A3 << cos(th[2]), -sin(th[2]), 0, L3*cos(th[2]), 
          sin(th[2]), cos(th[2]), 0, L3*sin(th[2]), 
          0, 0, 1, 0,
          0, 0, 0, 1;
    A4 << sin(th[3]), 0, cos(th[3]), 0,
          -cos(th[3]), 0, sin(th[3]), 0,
          0, -1, 0, 0,
          0, 0, 0, 1;
    A5 << -sin(th[4]), 0, cos(th[4]), 0,
          cos(th[4]), 0, sin(th[4]), 0,
          0, 1, 0, L5,
          0, 0, 0, 1;
    A6 << -sin(th[5]), -cos(th[5]), 0, -L6*sin(th[5]),
          cos(th[5]), -sin(th[5]), 0, L6*cos(th[5]),
          0, 0, 1, 0, 
          0, 0, 0, 1;          
          
    T00 = A0;
    T01 = T00*A1;
    T02 = T01*A2;
    T03 = T02*A3;
    T04 = T03*A4;
    T05 = T04*A5;
    T06 = T05*A6;

    T12 = T02-T01;
    T23 = T03-T02;
    T45 = T05-T04;
    T56 = T06-T05;
  
    a0 << T00(0,2), T00(1,2), T00(2,2);
    a1 << T01(0,2), T01(1,2), T01(2,2);
    a2 << T02(0,2), T02(1,2), T02(2,2);
    a3 << T03(0,2), T03(1,2), T03(2,2);
    a4 << T04(0,2), T04(1,2), T04(2,2);
    a5 << T05(0,2), T05(1,2), T05(2,2);

    P6_P0 << T06(0,3)-T00(0,3), T06(1,3)-T00(1,3), T06(2,3)-T00(2,3);
    P6_P1 << T06(0,3)-T01(0,3), T06(1,3)-T01(1,3), T06(2,3)-T01(2,3);
    P6_P2 << T06(0,3)-T02(0,3), T06(1,3)-T02(1,3), T06(2,3)-T02(2,3);
    P6_P3 << T06(0,3)-T03(0,3), T06(1,3)-T03(1,3), T06(2,3)-T03(2,3);
    P6_P4 << T06(0,3)-T04(0,3), T06(1,3)-T04(1,3), T06(2,3)-T04(2,3);
    P6_P5 << T06(0,3)-T05(0,3), T06(1,3)-T05(1,3), T06(2,3)-T05(2,3);
    
    J1 << a0.cross(P6_P0), a0;
    J2 << a1.cross(P6_P1), a1;
    J3 << a2.cross(P6_P2), a2;
    J4 << a3.cross(P6_P3), a3;
    J5 << a4.cross(P6_P4), a4;
    J6 << a5.cross(P6_P5), a5;

    Jacobian << J1, J2, J3, J4, J5, J6;

    ee_position << T06(0,3), T06(1,3), T06(2,3);

    //---updated GC---
    // tau_gravity_compensation[0] = 0.0;
    // tau_gravity_compensation[1] = 1.9076*sin(th[1])*sin(th[2]) - 1.9076*cos(th[1])*cos(th[2]) - 2.7704*cos(th[1]) + 0.43376*cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + 0.43376*sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])) + 0.14068*sin(th[5] + 1.5708)*(cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2]))) + 0.14068*cos(th[4] + 1.5708)*cos(th[5] + 1.5708)*(1.0*sin(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) - cos(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])));
    // tau_gravity_compensation[2] = 1.9076*sin(th[1])*sin(th[2]) - 1.9076*cos(th[1])*cos(th[2]) + 0.43376*cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + 0.43376*sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])) + 0.14068*sin(th[5] + 1.5708)*(cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2]))) + 0.14068*cos(th[4] + 1.5708)*cos(th[5] + 1.5708)*(1.0*sin(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) - cos(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])));
    // tau_gravity_compensation[3] = 0.43376*cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + 0.43376*sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])) + 0.14068*sin(th[5] + 1.5708)*(cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2]))) + 0.14068*cos(th[4] + 1.5708)*cos(th[5] + 1.5708)*(1.0*sin(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) - cos(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])));
    // tau_gravity_compensation[4] = 0.14068*cos(th[5] + 1.5708)*sin(th[4] + 1.5708)*(cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])));
    // tau_gravity_compensation[5] = 0.14068*cos(th[5] + 1.5708)*(1.0*sin(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) - cos(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2]))) + 0.14068*cos(th[4] + 1.5708)*sin(th[5] + 1.5708)*(cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])));

    //---updated accurate com---
    shoulder_link_com << joint0_to_L1_com/L1*T01(0,3), joint0_to_L1_com/L1*T01(1,3), joint0_to_L1_com/L1*T01(2,3);
    arm_link_com << T01(0,3)+joint1_to_L2_com/L2*T12(0,3), T01(1,3)+joint1_to_L2_com/L2*T12(1,3), T01(2,3)+joint1_to_L2_com/L2*T12(2,3);
    elbow_link_com << T02(0,3)+joint2_to_L3_com/L3*T23(0,3), T02(1,3)+joint2_to_L3_com/L3*T23(1,3), T02(2,3)+joint2_to_L3_com/L3*T23(2,3);
    forearm_link_com << T04(0,3)+joint3_to_L4_com/L5*T45(0,3), T04(1,3)+joint3_to_L4_com/L5*T45(1,3), T04(2,3)+joint3_to_L4_com/L5*T45(2,3);
    wrist_link_com << T04(0,3)+joint4_to_L5_com/L5*T45(0,3), T04(1,3)+joint4_to_L5_com/L5*T45(1,3), T04(2,3)+joint4_to_L5_com/L5*T45(2,3);
    endeffector_link_com << T06(0,3), T06(1,3), T06(2,3);

    manipulator_com <<  (m_Link1*shoulder_link_com(0) + m_Link2*arm_link_com(0) + m_Link3*elbow_link_com(0) + m_Link4*forearm_link_com(0) + m_Link5*wrist_link_com(0) + m_Link6*endeffector_link_com(0)) / m_Arm,
                        (m_Link1*shoulder_link_com(1) + m_Link2*arm_link_com(1) + m_Link3*elbow_link_com(1) + m_Link4*forearm_link_com(1) + m_Link5*wrist_link_com(1) + m_Link6*endeffector_link_com(1)) / m_Arm,
                        (m_Link1*shoulder_link_com(2) + m_Link2*arm_link_com(2) + m_Link3*elbow_link_com(2) + m_Link4*forearm_link_com(2) + m_Link5*wrist_link_com(2) + m_Link6*endeffector_link_com(2)) / m_Arm;

    C1 << manipulator_com(0), manipulator_com(1), manipulator_com(2);
    C2 << (m_Link2*arm_link_com(0) + m_Link3*elbow_link_com(0) + m_Link4*forearm_link_com(0) + m_Link5*wrist_link_com(0) + m_Link6*endeffector_link_com(0))/M2, (m_Link2*arm_link_com(1) + m_Link3*elbow_link_com(1) + m_Link4*forearm_link_com(1) + m_Link5*wrist_link_com(1) + m_Link6*endeffector_link_com(1))/M2, (m_Link2*arm_link_com(2) + m_Link3*elbow_link_com(2) + m_Link4*forearm_link_com(2) + m_Link5*wrist_link_com(2) + m_Link6*endeffector_link_com(2))/M2;
    C3 << (m_Link3*elbow_link_com(0) + m_Link4*forearm_link_com(0) + m_Link5*wrist_link_com(0) + m_Link6*endeffector_link_com(0))/M3, (m_Link3*elbow_link_com(1) + m_Link4*forearm_link_com(1) + m_Link5*wrist_link_com(1) + m_Link6*endeffector_link_com(1))/M3, (m_Link3*elbow_link_com(2) + m_Link4*forearm_link_com(2) + m_Link5*wrist_link_com(2) + m_Link6*endeffector_link_com(2))/M3;
    C4 << (m_Link4*forearm_link_com(0) + m_Link5*wrist_link_com(0) + m_Link6*endeffector_link_com(0))/M4, (m_Link4*forearm_link_com(1) + m_Link5*wrist_link_com(1) + m_Link6*endeffector_link_com(1))/M4, (m_Link4*forearm_link_com(2) + m_Link5*wrist_link_com(2) + m_Link6*endeffector_link_com(2))/M4;
    C5 << (m_Link5*wrist_link_com(0) + m_Link6*endeffector_link_com(0))/M5, (m_Link5*wrist_link_com(1) + m_Link6*endeffector_link_com(1))/M5, (m_Link5*wrist_link_com(2) + m_Link6*endeffector_link_com(2))/M5;
    C6 << endeffector_link_com(0), endeffector_link_com(1), endeffector_link_com(2);

    C1_P0 << C1(0)-T00(0,3), C1(1)-T00(1,3), C1(2)-T00(2,3);
    C2_P1 << C2(0)-T01(0,3), C2(1)-T01(1,3), C2(2)-T01(2,3);
    C3_P2 << C3(0)-T02(0,3), C3(1)-T02(1,3), C3(2)-T02(2,3);
    C4_P3 << C4(0)-T03(0,3), C4(1)-T03(1,3), C4(2)-T03(2,3);
    C5_P4 << C5(0)-T04(0,3), C5(1)-T04(1,3), C5(2)-T04(2,3);
    C6_P5 << C6(0)-T05(0,3), C6(1)-T05(1,3), C6(2)-T05(2,3);
    
    J1_CoM = M1 / m_Arm * a0.cross(C1_P0);
    J2_CoM = M2 / m_Arm * a1.cross(C2_P1);
    J3_CoM = M3 / m_Arm * a2.cross(C3_P2);
    J4_CoM = M4 / m_Arm * a3.cross(C4_P3);
    J5_CoM = M5 / m_Arm * a4.cross(C5_P4);
    J6_CoM = M6 / m_Arm * a5.cross(C6_P5);

    J_CoM << J1_CoM, J2_CoM, J3_CoM, J4_CoM, J5_CoM, J6_CoM;

    if (cnt<1) initial_com_position << 0.05,0,0.25;
		desired_com_position = initial_com_position;

		for(uint i=0; i<3; i++) virtual_spring_com(i) = gain_p(i) * (desired_com_position(i) - manipulator_com(i));
		
    tau_com = J_CoM.transpose() * virtual_spring_com;
    
    ee_rotation = T06.block<3,3>(0,0);
    ee_rotation_x = ee_rotation.block<3,1>(0,0);
    ee_rotation_y = ee_rotation.block<3,1>(0,1);
    ee_rotation_z = ee_rotation.block<3,1>(0,2);

    ref_ee_rotation = hmd_quaternion.normalized().toRotationMatrix();  
    
    ref_ee_rotation_x = ref_ee_rotation.block<3,1>(0,0);
    ref_ee_rotation_y = ref_ee_rotation.block<3,1>(0,1);
    ref_ee_rotation_z = ref_ee_rotation.block<3,1>(0,2);

    ee_orientation_error = ee_rotation_x.cross(ref_ee_rotation_x) + ee_rotation_y.cross(ref_ee_rotation_y) + ee_rotation_z.cross(ref_ee_rotation_z);
    
		ee_momentum << gain_w(0) * ee_orientation_error(0), gain_w(1) * ee_orientation_error(1), gain_w(2) * ee_orientation_error(2);

    virtual_spring_rotational << 0, 0, 0, ee_momentum(0), ee_momentum(1), ee_momentum(2);
    tau_rotational = Jacobian.transpose() * virtual_spring_rotational;

    // tau_gravity_compensation[0] = 0.0;
    // tau_gravity_compensation[1] = 1.9318*sin(th[1])*sin(th[2]) - 1.9318*cos(th[1])*cos(th[2]) - 2.9498*cos(th[1]) + 0.43025*cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + 0.43025*sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])) + 0.14096*sin(th[5] + 1.5708)*(cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2]))) + 0.14096*cos(th[4] + 1.5708)*cos(th[5] + 1.5708)*(1.0*sin(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) - cos(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])));
    // tau_gravity_compensation[2] = 1.9318*sin(th[1])*sin(th[2]) - 1.9318*cos(th[1])*cos(th[2]) + 0.43025*cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + 0.43025*sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])) + 0.14096*sin(th[5] + 1.5708)*(cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2]))) + 0.14096*cos(th[4] + 1.5708)*cos(th[5] + 1.5708)*(1.0*sin(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) - cos(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])));
    // tau_gravity_compensation[3] = 0.43025*cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + 0.43025*sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])) + 0.14096*sin(th[5] + 1.5708)*(cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2]))) + 0.14096*cos(th[4] + 1.5708)*cos(th[5] + 1.5708)*(1.0*sin(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) - cos(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])));
    // tau_gravity_compensation[4] = 0.14096*cos(th[5] + 1.5708)*sin(th[4] + 1.5708)*(cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])));
    // tau_gravity_compensation[5] = 0.14096*cos(th[5] + 1.5708)*(1.0*sin(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) - cos(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2]))) + 0.14096*cos(th[4] + 1.5708)*sin(th[5] + 1.5708)*(cos(th[3] - 1.5708)*(cos(th[1])*sin(th[2]) + cos(th[2])*sin(th[1])) + sin(th[3] - 1.5708)*(cos(th[1])*cos(th[2]) - 1.0*sin(th[1])*sin(th[2])));
    
    // Nonlinear 토크 계산
    RBDL::NonlinearEffects(*arm_rbdl.rbdl_model, arm_rbdl.q, arm_rbdl.q_dot, arm_rbdl.tau_nonlinear, NULL);
    for(uint8_t i = 0; i < 6; i++)
    {
      tau_gravity_compensation(i) = arm_rbdl.tau_nonlinear(i);
    }
		tau =  tau_com + tau_rotational;// - tau_viscous_damping;
		
    for(int i=0; i<6; i++)
    {               
      if(th(i)>joint_limit(0,i)-threshold(i) && tau(i)>0 || th(i)<joint_limit(1,i)+threshold(i) && tau(i)<0)
        joint_torque(i) = gain_r(i)*tau_gravity_compensation(i);//we can add more damping here
      else
        joint_torque(i) = tau(i) + gain_r(i)*tau_gravity_compensation(i); 
    }
  
    cnt++;
  }

  //  Weight estimation
  void STArmPlugin::Motion6()
  {
    // gain_p << 2000, 200, 200;
    // gain_w << 10, 10, 10;
    // gain_r << 1, 1, 1, 1, 1, 1; //adjust GC intensity

    step_time = 3;
    
    cnt_time = cnt*inner_dt;   

    gain_p = gain_p_task_space;
    gain_w = gain_w_task_space; 
    gain_r << 1, 1, 1, 1, 1, 1; //adjust GC intensity

    threshold << 0.2, 0.1, 0.1, 0.01, 0.01, 0.01; 
    joint_limit << 3.14,     0,  2.8,  1.57,  2.5,  1.57,
                    -3.14, -3.14, -0.0, -1.57, -2.5, -1.57;

    A0 << 1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 1, 0,
          0, 0, 0, 1;
    A1 << cos(th[0]), 0, -sin(th[0]), 0,
          sin(th[0]), 0, cos(th[0]), 0,
          0, -1, 0, L1,
          0, 0, 0, 1;
    A2 << cos(th[1]), -sin(th[1]), 0, L2*cos(th[1]),
          sin(th[1]), cos(th[1]), 0, L2*sin(th[1]),
          0, 0, 1, 0, 
          0, 0, 0, 1;
    A3 << cos(th[2]), -sin(th[2]), 0, L3*cos(th[2]), 
          sin(th[2]), cos(th[2]), 0, L3*sin(th[2]), 
          0, 0, 1, 0,
          0, 0, 0, 1;
    A4 << sin(th[3]), 0, cos(th[3]), 0,
          -cos(th[3]), 0, sin(th[3]), 0,
          0, -1, 0, 0,
          0, 0, 0, 1;
    A5 << -sin(th[4]), 0, cos(th[4]), 0,
          cos(th[4]), 0, sin(th[4]), 0,
          0, 1, 0, L5,
          0, 0, 0, 1;
    A6 << -sin(th[5]), -cos(th[5]), 0, -L6*sin(th[5]),
          cos(th[5]), -sin(th[5]), 0, L6*cos(th[5]),
          0, 0, 1, 0, 
          0, 0, 0, 1;          
          
    T00 = A0;
    T01 = T00*A1;
    T02 = T01*A2;
    T03 = T02*A3;
    T04 = T03*A4;
    T05 = T04*A5;
    T06 = T05*A6;
  
    a0 << T00(0,2), T00(1,2), T00(2,2);
    a1 << T01(0,2), T01(1,2), T01(2,2);
    a2 << T02(0,2), T02(1,2), T02(2,2);
    a3 << T03(0,2), T03(1,2), T03(2,2);
    a4 << T04(0,2), T04(1,2), T04(2,2);
    a5 << T05(0,2), T05(1,2), T05(2,2);

    P6_P0 << T06(0,3)-T00(0,3), T06(1,3)-T00(1,3), T06(2,3)-T00(2,3);
    P6_P1 << T06(0,3)-T01(0,3), T06(1,3)-T01(1,3), T06(2,3)-T01(2,3);
    P6_P2 << T06(0,3)-T02(0,3), T06(1,3)-T02(1,3), T06(2,3)-T02(2,3);
    P6_P3 << T06(0,3)-T03(0,3), T06(1,3)-T03(1,3), T06(2,3)-T03(2,3);
    P6_P4 << T06(0,3)-T04(0,3), T06(1,3)-T04(1,3), T06(2,3)-T04(2,3);
    P6_P5 << T06(0,3)-T05(0,3), T06(1,3)-T05(1,3), T06(2,3)-T05(2,3);

    J1 << a0.cross(P6_P0), a0;
    J2 << a1.cross(P6_P1), a1;
    J3 << a2.cross(P6_P2), a2;
    J4 << a3.cross(P6_P3), a3;
    J5 << a4.cross(P6_P4), a4;
    J6 << a5.cross(P6_P5), a5;

    Jacobian << J1, J2, J3, J4, J5, J6;

    ee_position << T06(0,3), T06(1,3), T06(2,3);
    
    // if (cnt<1) initial_ee_position << ee_position(0), ee_position(1), ee_position(2);
    // if (cnt<1) initial_ee_position << 0.4, 0, 0.3;
    // if(cnt_time <= step_time*5)
    // { 
    //   ref_ee_position(0) = initial_ee_position(0) - 0.2*abs(sin(PI/2*(cnt_time/step_time)));
    //   ref_ee_position(1) = initial_ee_position(1) - 0.3*sin(PI/2*(cnt_time/step_time));
    //   ref_ee_position(2) = initial_ee_position(2) + 0.2*sin(PI*(cnt_time/step_time));
    //   ref_ee_quaternion.w() = 1;
    //   ref_ee_quaternion.x() = 0;
    //   ref_ee_quaternion.y() = 0;
    //   ref_ee_quaternion.z() = 0;
    // }
    // ref_ee_rotation = ref_ee_quaternion.normalized().toRotationMatrix();
    
    ref_ee_position = hmd_position;
    ref_ee_rotation = hmd_quaternion.normalized().toRotationMatrix();

    ee_position_error(0) =  ref_ee_position(0) - ee_position(0);
    ee_position_error(1) =  ref_ee_position(1) - ee_position(1);
    ee_position_error(2) =  ref_ee_position(2) - ee_position(2);

    ee_force << gain_p(0) * ee_position_error(0), 
                gain_p(1) * ee_position_error(1), 
                gain_p(2) * ee_position_error(2);

    ee_rotation = T06.block<3,3>(0,0);
    ee_rotation_x = ee_rotation.block<3,1>(0,0); 
    ee_rotation_y = ee_rotation.block<3,1>(0,1); 
    ee_rotation_z = ee_rotation.block<3,1>(0,2);


    ref_ee_rotation_x = ref_ee_rotation.block<3,1>(0,0);
    ref_ee_rotation_y = ref_ee_rotation.block<3,1>(0,1);
    ref_ee_rotation_z = ref_ee_rotation.block<3,1>(0,2);
    ee_orientation_error = ee_rotation_x.cross(ref_ee_rotation_x) + ee_rotation_y.cross(ref_ee_rotation_y) + ee_rotation_z.cross(ref_ee_rotation_z);
    ee_momentum << gain_w(0) * ee_orientation_error(0), gain_w(1) * ee_orientation_error(1), gain_w(2) * ee_orientation_error(2);

    pose_difference << ee_position_error(0), ee_position_error(1), ee_position_error(2), ee_orientation_error(0), ee_orientation_error(1), ee_orientation_error(2);

    estimated_object_weight = ee_force(2) / 9.81;
    
    if(is_start_estimation)
    {
      estimated_object_weight_difference += estimated_object_weight;
      InitializeRBDLVariablesWithObj(estimated_object_weight_difference);
      is_start_estimation = false;
      std::cout << "RBDL calibrated with obj weight estimation method" << std::endl;
      std::cout << "estimated_object_weight_difference is: " << estimated_object_weight_difference << std::endl;
      std::cout << "last_estimated_object_weight is: " << last_estimated_object_weight << std::endl;
      std::cout << "estimated_object_weight_difference is: " << estimated_object_weight_difference << std::endl;
      last_estimated_object_weight = estimated_object_weight;
    }
    

    virtual_spring << ee_force(0), ee_force(1), ee_force(2), ee_momentum(0), ee_momentum(1), ee_momentum(2);
  
    RBDL::NonlinearEffects(*arm_rbdl.rbdl_model, arm_rbdl.q, arm_rbdl.q_dot, arm_rbdl.tau_nonlinear, NULL);
    for(uint8_t i = 0; i < 6; i++)
    {
      tau_rbdl(i) = arm_rbdl.tau_nonlinear(i);
    }

    tau = Jacobian.transpose() * virtual_spring;

    for(uint8_t i=0; i<6; i++) 
    {
      tau_viscous_damping[i] = gain_d_joint_space[i] * th_dot[i]; 
      tau_gravity_compensation[i] = tau_rbdl[i] * gain_r[i];
    }

    for(uint8_t i=0; i<6; i++){
      if(th(i) > joint_limit(0,i) - threshold(i) && tau(i) > 0 || th(i) < joint_limit(1,i) + threshold(i) && tau(i) < 0)
      {
        joint_torque[i] = tau_gravity_compensation[i] - tau_viscous_damping[i];
      }
      else
      {
        joint_torque[i] = tau_gravity_compensation[i] - tau_viscous_damping[i] + tau[i]; 
      } 
    }

    cnt++;
  }

  //  For CTC || Infinity Drawer
  void STArmPlugin::Motion7()
  {
    dt = 0.001;
    temp_gain_p << 100, 100, 100, 50, 50, 50;
    temp_gain_v << 80, 80, 80, 40, 40, 40;


    threshold << 0.2, 0.2, 0.2, 0.2, 0.2, 0.2; // limit threshold angle
    joint_limit << 3.14,     0,  2.8,  1.87,  1.57,  1.57,
                  -3.14, -3.14, -0.3, -1.27, -1.57, -1.57;
    // gain_r << 1, 0.3, 0.5, 0.5, 0.5, 0.5; // affects joint limit control
    gain_r << 1, 1, 1, 1, 1, 1; // affects joint limit control
    
    // cnt_time = cnt * dt; 
    double pi_ref = 0, theta_ref = 0, psi_ref = 0;

    if (cnt <= 1) initial_ee_position << 0.4, 0, 0.3;

    SetRBDLVariables();

    Calc_Feedback_Pose(arm_rbdl);

    //for infinity Trajectory
    step_time = 3;
    cnt_time = cnt * dt; 

    if(cnt_time <= step_time*100)
    { 
      ref_ee_position(0) = initial_ee_position(0)- 0.2*abs(sin(PI/2*(cnt_time/step_time)));
      ref_ee_position(1) = initial_ee_position(1)- 0.3*sin(PI/2*(cnt_time/step_time));
      ref_ee_position(2) = initial_ee_position(2)+ 0.2*sin(PI*(cnt_time/step_time));
      ref_ee_quaternion.w() = 1;
      ref_ee_quaternion.x() = 0;
      ref_ee_quaternion.y() = 0;
      ref_ee_quaternion.z() = 0;
    }
    ref_ee_rotation = ref_ee_quaternion.normalized().toRotationMatrix();

    // arm_rbdl.rpy_desired = ref_ee_rotation.eulerAngles(2,1,2);
    // pi_ref = arm_rbdl.rpy_desired(0);
    // theta_ref = arm_rbdl.rpy_desired(1);
    // psi_ref = arm_rbdl.rpy_desired(2);

    // z-y-x
    pi_ref = atan2(ref_ee_rotation(1,0),ref_ee_rotation(0,0));
    theta_ref = atan2(-ref_ee_rotation(2,0), cos(pi_ref)*ref_ee_rotation(0,0) + sin(pi_ref)*ref_ee_rotation(1,0));
    psi_ref = atan2(sin(pi_ref)*ref_ee_rotation(0,2) - cos(pi_ref)*ref_ee_rotation(1,2), -sin(pi_ref)*ref_ee_rotation(0,1) + cos(pi_ref)*ref_ee_rotation(1,1));
    
    arm_rbdl.x_desired << ref_ee_position, psi_ref, theta_ref, pi_ref;
    arm_rbdl.x_desired_dot = (arm_rbdl.x_desired - arm_rbdl.x_desired_last) / dt;
    arm_rbdl.x_desired_last = arm_rbdl.x_desired;
    arm_rbdl.x_desired_d_dot = (arm_rbdl.x_desired_dot - arm_rbdl.x_desired_dot_last) / dt;
    arm_rbdl.x_desired_dot_last = arm_rbdl.x_desired_dot; 

    for(int i = 0; i < 6; i++)
    {
      arm_rbdl.x_ctc_d_dot(i) = arm_rbdl.x_desired_d_dot(i) + temp_gain_p(i)*(arm_rbdl.x_desired(i) - arm_rbdl.x_actual(i)) + temp_gain_v(i)*(arm_rbdl.x_desired_dot(i) - arm_rbdl.x_actual_dot(i));
    }


    arm_rbdl.q_d_dot_ctc = arm_rbdl.jacobian_ana_inverse * (arm_rbdl.x_ctc_d_dot - arm_rbdl.jacobian_ana_dot * arm_rbdl.q_dot);


    RBDL::NonlinearEffects(*arm_rbdl.rbdl_model, arm_rbdl.q, arm_rbdl.q_dot, arm_rbdl.tau_nonlinear, NULL); // Nonlinear 토크 생성

    arm_rbdl.inertia_matrix = RBDLMatrixNd::Zero(6,6);// inertia Matrix 0 초기화
    RBDL::CompositeRigidBodyAlgorithm(*arm_rbdl.rbdl_model, arm_rbdl.q, arm_rbdl.inertia_matrix, true); // inertia Matrix 생성
    arm_rbdl.tau_inertia = arm_rbdl.inertia_matrix * arm_rbdl.q_d_dot_ctc;// inertia 토크 생성

    for(uint8_t i=0; i<6; i++)
    {
      // if(th(i)>joint_limit(0,i)-threshold(i) && tau(i)>0 || th(i)<joint_limit(1,i)+threshold(i) && tau(i)<0)
      //   joint_torque(i) = gain_r(i)*tau_gravity_compensation(i);
      // else
        joint_torque(i) = arm_rbdl.tau_inertia(i) + gain_r(i)*arm_rbdl.tau_nonlinear(i);  //CTC 토크
    }

    cnt++;
  }

  // ctc2
  void STArmPlugin::Motion8()
  { 
    dt = 0.001;
    VectorXd temp_gain_p(6);
    temp_gain_p << 1, 1, 1, 1, 1, 1;
    // temp_gain_p << 1, 1, 1, 0, 0, 0;
    VectorXd temp_gain_v(6);
    temp_gain_v << 1, 1, 1, 1, 1, 1;
    // temp_gain_v << 1, 1, 1, 0, 0, 0;

    for(int i=0; i<6; i++) {
      arm_rbdl.ts_p(i, i) = temp_gain_p(i);
      arm_rbdl.ts_v(i, i) = temp_gain_v(i);
    }

    threshold << 0.2, 0.2, 0.2, 0.2, 0.2, 0.2; // limit threshold angle
    joint_limit << 3.14,     0,  2.8,  1.87,  1.57,  1.57,
                  -3.14, -3.14, -0.3, -1.27, -1.57, -1.57;
    // gain_r << 1, 0.3, 0.5, 0.5, 0.5, 0.5; // affects joint limit control
    gain_r << 1, 1, 1, 1, 1, 1; // affects joint limit control

        A0 << 1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 1, 0,
          0, 0, 0, 1;
    A1 << cos(th[0]), 0, -sin(th[0]), 0,
          sin(th[0]), 0, cos(th[0]), 0,
          0, -1, 0, L1,
          0, 0, 0, 1;
    A2 << cos(th[1]), -sin(th[1]), 0, L2*cos(th[1]),
          sin(th[1]), cos(th[1]), 0, L2*sin(th[1]),
          0, 0, 1, 0, 
          0, 0, 0, 1;
    A3 << cos(th[2]), -sin(th[2]), 0, L3*cos(th[2]), 
          sin(th[2]), cos(th[2]), 0, L3*sin(th[2]), 
          0, 0, 1, 0,
          0, 0, 0, 1;
    A4 << sin(th[3]), 0, cos(th[3]), 0,
          -cos(th[3]), 0, sin(th[3]), 0,
          0, -1, 0, 0,
          0, 0, 0, 1;
    A5 << -sin(th[4]), 0, cos(th[4]), 0,
          cos(th[4]), 0, sin(th[4]), 0,
          0, 1, 0, L5,
          0, 0, 0, 1;
    A6 << -sin(th[5]), -cos(th[5]), 0, -L6*sin(th[5]),
          cos(th[5]), -sin(th[5]), 0, L6*cos(th[5]),
          0, 0, 1, 0, 
          0, 0, 0, 1;
          
    T00 = A0;
    T01 = T00*A1;
    T02 = T01*A2;
    T03 = T02*A3;
    T04 = T03*A4;
    T05 = T04*A5;
    T06 = T05*A6;
  
    a0 << T00(0,2), T00(1,2), T00(2,2);
    a1 << T01(0,2), T01(1,2), T01(2,2);
    a2 << T02(0,2), T02(1,2), T02(2,2);
    a3 << T03(0,2), T03(1,2), T03(2,2);
    a4 << T04(0,2), T04(1,2), T04(2,2);
    a5 << T05(0,2), T05(1,2), T05(2,2);

    P6_P0 << T06(0,3)-T00(0,3), T06(1,3)-T00(1,3), T06(2,3)-T00(2,3);
    P6_P1 << T06(0,3)-T01(0,3), T06(1,3)-T01(1,3), T06(2,3)-T01(2,3);
    P6_P2 << T06(0,3)-T02(0,3), T06(1,3)-T02(1,3), T06(2,3)-T02(2,3);
    P6_P3 << T06(0,3)-T03(0,3), T06(1,3)-T03(1,3), T06(2,3)-T03(2,3);
    P6_P4 << T06(0,3)-T04(0,3), T06(1,3)-T04(1,3), T06(2,3)-T04(2,3);
    P6_P5 << T06(0,3)-T05(0,3), T06(1,3)-T05(1,3), T06(2,3)-T05(2,3);

    J1 << a0.cross(P6_P0), a0;
    J2 << a1.cross(P6_P1), a1;
    J3 << a2.cross(P6_P2), a2;
    J4 << a3.cross(P6_P3), a3;
    J5 << a4.cross(P6_P4), a4;
    J6 << a5.cross(P6_P5), a5;

    Jacobian << J1, J2, J3, J4, J5, J6;

    ee_position << T06(0,3), T06(1,3), T06(2,3);
    ee_rotation = T06.block<3,3>(0,0);

    double pi = 0, theta = 0, psi = 0;
    pi = atan2(ee_rotation(1,0),ee_rotation(0,0));
    theta = atan2(-ee_rotation(2,0), cos(pi)*ee_rotation(0,0) + sin(pi)*ee_rotation(1,0));
    psi = atan2(sin(pi)*ee_rotation(0,2) - cos(pi)*ee_rotation(1,2), -sin(pi)*ee_rotation(0,1) + cos(pi)*ee_rotation(1,1));

    arm_rbdl.x_actual << ee_position, psi, theta, pi;

    //for infinity Trajectory
    step_time = 3;
    cnt_time = cnt * dt; 

    if (cnt<1) initial_ee_position << 0.4, 0, 0.3;

    if(cnt_time <= step_time*100)
    { 
      ref_ee_position(0) = initial_ee_position(0);// - 0.2*abs(sin(PI/2*(cnt_time/step_time)));
      ref_ee_position(1) = initial_ee_position(1);// - 0.3*sin(PI/2*(cnt_time/step_time));
      ref_ee_position(2) = initial_ee_position(2);// + 0.2*sin(PI*(cnt_time/step_time));
      ref_ee_quaternion.w() = 1;
      ref_ee_quaternion.x() = 0;
      ref_ee_quaternion.y() = 0;
      ref_ee_quaternion.z() = 0;
    }
    ref_ee_rotation = ref_ee_quaternion.normalized().toRotationMatrix();

    SetRBDLVariables();

    double pi_ref = 0, theta_ref = 0, psi_ref = 0;
    pi_ref = atan2(ref_ee_rotation(1,0),ref_ee_rotation(0,0));
    theta_ref = atan2(-ref_ee_rotation(2,0), cos(pi_ref)*ref_ee_rotation(0,0) + sin(pi_ref)*ref_ee_rotation(1,0));
    psi_ref = atan2(sin(pi_ref)*ref_ee_rotation(0,2) - cos(pi_ref)*ref_ee_rotation(1,2), -sin(pi_ref)*ref_ee_rotation(0,1) + cos(pi_ref)*ref_ee_rotation(1,1));
    
    arm_rbdl.x_desired << ref_ee_position, psi_ref, theta_ref, pi_ref;

    arm_rbdl.geometric_to_analytic << 1, 0, 0, 0, 0, 0,
                                      0, 1, 0, 0, 0, 0, 
                                      0, 0, 1, 0, 0, 0, 
                                      0, 0, 0, cos(pi)/cos(theta), sin(pi)/cos(theta), 0, 
                                      0, 0, 0, -sin(pi), cos(pi), 0, 
                                      0, 0, 0, cos(pi)*tan(theta), sin(pi)*tan(theta), 1;

    arm_rbdl.jacobian_ana = arm_rbdl.geometric_to_analytic * Jacobian;

    arm_rbdl.jacobian_ana_inverse = arm_rbdl.jacobian_ana.inverse();


 
    arm_rbdl.x_error = arm_rbdl.x_desired - arm_rbdl.x_actual;
    arm_rbdl.virtual_spring = arm_rbdl.ts_p * arm_rbdl.x_error;

    arm_rbdl.x_desired_dot = (arm_rbdl.x_desired - arm_rbdl.x_desired_last) / dt;
    arm_rbdl.x_desired_last = arm_rbdl.x_desired;

    arm_rbdl.x_actual_dot = arm_rbdl.jacobian_ana * arm_rbdl.q_dot;
    arm_rbdl.x_error_dot = arm_rbdl.x_desired_dot - arm_rbdl.x_actual_dot;
    arm_rbdl.virtual_damping = arm_rbdl.ts_v * arm_rbdl.x_error_dot;

    arm_rbdl.x_desired_d_dot = (arm_rbdl.x_desired_dot - arm_rbdl.x_desired_dot_last) / dt;
    arm_rbdl.x_desired_dot_last = arm_rbdl.x_desired_dot;

    arm_rbdl.x_ctc_d_dot = arm_rbdl.virtual_spring + arm_rbdl.virtual_damping + arm_rbdl.x_desired_d_dot;

    // arm_rbdl.jacobian_ana_dot = (arm_rbdl.jacobian_ana - arm_rbdl.jacobian_ana_prev) / dt;

    for(int i = 0; i<6; i++)
    {
      for(int j = 0; j<6; j++){
        arm_rbdl.jacobian_ana_dot(i,j) = (arm_rbdl.jacobian_ana(i,j)-arm_rbdl.jacobian_prev(i,j)) / dt;
      }
    }
    arm_rbdl.jacobian_prev = arm_rbdl.jacobian_ana;
    arm_rbdl.q_d_dot_ctc = arm_rbdl.jacobian_ana_inverse * (arm_rbdl.x_ctc_d_dot - arm_rbdl.jacobian_ana_dot * arm_rbdl.q_dot);


    RBDL::NonlinearEffects(*arm_rbdl.rbdl_model, arm_rbdl.q, arm_rbdl.q_dot, arm_rbdl.tau_nonlinear, NULL); // Nonlinear 토크 생성
    arm_rbdl.inertia_matrix = RBDLMatrixNd::Zero(6,6);// inertia Matrix 0 초기화
    RBDL::CompositeRigidBodyAlgorithm(*arm_rbdl.rbdl_model, arm_rbdl.q, arm_rbdl.inertia_matrix, false); // inertia Matrix 생성
    arm_rbdl.tau_inertia = arm_rbdl.inertia_matrix * arm_rbdl.q_d_dot_ctc;// inertia 토크 생성

    
    for(uint8_t i = 0; i < 6; i++)
    {
      tau_gravity_compensation(i) = arm_rbdl.tau_nonlinear(i);
      tau(i) = arm_rbdl.tau_inertia(i);
    }

    for(uint8_t i=0; i<6; i++)
    {
      if(th(i)>joint_limit(0,i)-threshold(i) && tau(i)>0 || th(i)<joint_limit(1,i)+threshold(i) && tau(i)<0)
        joint_torque(i) = gain_r(i)*tau_gravity_compensation(i);
      else
        joint_torque(i) = tau(i) + gain_r(i)*tau_gravity_compensation(i);  //CTC 토크
    }

    cnt++;
  }

  // PID control || No Gravity Compensation || Infinity Drawer
  void STArmPlugin::Motion9()
  {
    step_time = 3;
    
    cnt_time = cnt*inner_dt;   

    gain_p = gain_p_task_space;
    gain_w = gain_w_task_space; 
    gain_r << 1, 1, 1, 1, 1, 1; //adjust GC intensity


    threshold << deg, deg, deg, deg, deg, deg;
    joint_limit <<   (180/180)*PI,   (180/180)*PI,  (150/180)*PI,  1.57,  1.57,  1.57, 
                    (-180/180)*PI,           -PI,  (90/180)*PI, -1.57, -1.57, -1.57;
    // joint_limit <<   (180/180)*PI,   (35/180)*PI,  (164.8/180)*PI,  1.57,  1.57,  1.57, 
    //                 (-135/180)*PI,           -PI,      (5/180)*PI, -1.57, -1.57, -1.57;
    // threshold << 0.2, 0.1, 0.1, 0.1, 0.1, 0.1; 
    // joint_limit << 3.14,     0,  2.8,  1.87,  1.57,  1.57,
    //                 -3.14, -3.14, -0.3, -1.27, -1.57, -1.57;

    A0 << 1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 1, 0,
          0, 0, 0, 1;
    A1 << cos(th[0]), 0, -sin(th[0]), 0,
          sin(th[0]), 0, cos(th[0]), 0,
          0, -1, 0, L1,
          0, 0, 0, 1;
    A2 << cos(th[1]), -sin(th[1]), 0, L2*cos(th[1]),
          sin(th[1]), cos(th[1]), 0, L2*sin(th[1]),
          0, 0, 1, 0, 
          0, 0, 0, 1;
    A3 << cos(th[2]), -sin(th[2]), 0, L3*cos(th[2]), 
          sin(th[2]), cos(th[2]), 0, L3*sin(th[2]), 
          0, 0, 1, 0,
          0, 0, 0, 1;
    A4 << sin(th[3]), 0, cos(th[3]), 0,
          -cos(th[3]), 0, sin(th[3]), 0,
          0, -1, 0, 0,
          0, 0, 0, 1;
    A5 << -sin(th[4]), 0, cos(th[4]), 0,
          cos(th[4]), 0, sin(th[4]), 0,
          0, 1, 0, L5,
          0, 0, 0, 1;
    A6 << -sin(th[5]), -cos(th[5]), 0, -L6*sin(th[5]),
          cos(th[5]), -sin(th[5]), 0, L6*cos(th[5]),
          0, 0, 1, 0, 
          0, 0, 0, 1;
          
    T00 = A0;
    T01 = T00*A1;
    T02 = T01*A2;
    T03 = T02*A3;
    T04 = T03*A4;
    T05 = T04*A5;
    T06 = T05*A6;
  
    a0 << T00(0,2), T00(1,2), T00(2,2);
    a1 << T01(0,2), T01(1,2), T01(2,2);
    a2 << T02(0,2), T02(1,2), T02(2,2);
    a3 << T03(0,2), T03(1,2), T03(2,2);
    a4 << T04(0,2), T04(1,2), T04(2,2);
    a5 << T05(0,2), T05(1,2), T05(2,2);

    P6_P0 << T06(0,3)-T00(0,3), T06(1,3)-T00(1,3), T06(2,3)-T00(2,3);
    P6_P1 << T06(0,3)-T01(0,3), T06(1,3)-T01(1,3), T06(2,3)-T01(2,3);
    P6_P2 << T06(0,3)-T02(0,3), T06(1,3)-T02(1,3), T06(2,3)-T02(2,3);
    P6_P3 << T06(0,3)-T03(0,3), T06(1,3)-T03(1,3), T06(2,3)-T03(2,3);
    P6_P4 << T06(0,3)-T04(0,3), T06(1,3)-T04(1,3), T06(2,3)-T04(2,3);
    P6_P5 << T06(0,3)-T05(0,3), T06(1,3)-T05(1,3), T06(2,3)-T05(2,3);

    J1 << a0.cross(P6_P0), a0;
    J2 << a1.cross(P6_P1), a1;
    J3 << a2.cross(P6_P2), a2;
    J4 << a3.cross(P6_P3), a3;
    J5 << a4.cross(P6_P4), a4;
    J6 << a5.cross(P6_P5), a5;

    Jacobian << J1, J2, J3, J4, J5, J6;

    ee_position << T06(0,3), T06(1,3), T06(2,3);
    
    // if (cnt<1) initial_ee_position << ee_position(0), ee_position(1), ee_position(2);
    if (cnt<1) initial_ee_position << 0.4, 0, 0.3;

    if(cnt_time <= step_time*100)
    { 
      ref_ee_position(0) = initial_ee_position(0) - 0.2*abs(sin(PI/2*(cnt_time/step_time)));
      ref_ee_position(1) = initial_ee_position(1) - 0.3*sin(PI/2*(cnt_time/step_time));
      ref_ee_position(2) = initial_ee_position(2) + 0.2*sin(PI*(cnt_time/step_time));
      ref_ee_quaternion.w() = 1;
      ref_ee_quaternion.x() = 0;
      ref_ee_quaternion.y() = 0;
      ref_ee_quaternion.z() = 0;
    }

    ee_force(0) = gain_p(0) * (ref_ee_position(0) - ee_position(0));
    ee_force(1) = gain_p(1) * (ref_ee_position(1) - ee_position(1));
    ee_force(2) = gain_p(2) * (ref_ee_position(2) - ee_position(2));

    ee_rotation = T06.block<3,3>(0,0);
    ee_rotation_x = ee_rotation.block<3,1>(0,0); 
    ee_rotation_y = ee_rotation.block<3,1>(0,1); 
    ee_rotation_z = ee_rotation.block<3,1>(0,2);

    //for MSG Publisher
    arm_rbdl.rpy_ee = ee_rotation.eulerAngles(2,1,0);

    ref_ee_rotation = ref_ee_quaternion.normalized().toRotationMatrix();    

    ref_ee_rotation_x = ref_ee_rotation.block<3,1>(0,0); 
    ref_ee_rotation_y = ref_ee_rotation.block<3,1>(0,1); 
    ref_ee_rotation_z = ref_ee_rotation.block<3,1>(0,2);
    ee_orientation_error = ee_rotation_x.cross(ref_ee_rotation_x) + ee_rotation_y.cross(ref_ee_rotation_y) + ee_rotation_z.cross(ref_ee_rotation_z);
    ee_momentum << gain_w(0) * ee_orientation_error(0), gain_w(1) * ee_orientation_error(1), gain_w(2) * ee_orientation_error(2);

    virtual_spring << ee_force(0), ee_force(1), ee_force(2), ee_momentum(0), ee_momentum(1), ee_momentum(2);

    tau_vs = Jacobian.transpose() * virtual_spring;
    
    for(uint8_t i=0; i<6; i++)
    {
        joint_torque(i) = tau_vs(i);
    }
  }

  void STArmPlugin::HMDPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
  {
    hmd_position.x() = msg->pose.position.x;
    hmd_position.y() = msg->pose.position.y;
    hmd_position.z() = msg->pose.position.z;

    hmd_quaternion.x() = msg->pose.orientation.x;
    hmd_quaternion.y() = msg->pose.orientation.y;
    hmd_quaternion.z() = msg->pose.orientation.z;
    hmd_quaternion.w() = msg->pose.orientation.w;
  }


  void STArmPlugin::SetRBDLVariables()
  {
    for(uint8_t i = 0; i < 6; i++)
    {
      arm_rbdl.q(i) = th(i);
      arm_rbdl.q_dot(i) = th_dot(i);
      arm_rbdl.q_d_dot(i) = th_d_dot(i);
    }
  }


  void STArmPlugin::SwitchGainJointSpaceP(const std_msgs::Float32MultiArrayConstPtr &msg)
  {
    for(uint8_t i=0; i<NUM_OF_JOINTS_WITH_TOOL; i++)
    {
      gain_p_joint_space(i) = msg->data.at(i);
    }
  }


  void STArmPlugin::SwitchGainJointSpaceD(const std_msgs::Float32MultiArrayConstPtr &msg)
  {
    for(uint8_t i=0; i<NUM_OF_JOINTS_WITH_TOOL; i++)
    {
      gain_d_joint_space(i) = msg->data.at(i);
    }
  }


  void STArmPlugin::SwitchGainTaskSpaceP(const std_msgs::Float32MultiArrayConstPtr &msg)
  {
    for(uint8_t i=0; i<3; i++)
    {
      gain_p_task_space(i) = msg->data.at(i);
    }
  }


  void STArmPlugin::SwitchGainTaskSpaceW(const std_msgs::Float32MultiArrayConstPtr &msg)
  {
    for(uint8_t i=0; i<3; i++)
    {
      gain_w_task_space(i) = msg->data.at(i);
    }
  }


  void STArmPlugin::SwitchGainR(const std_msgs::Float32MultiArrayConstPtr &msg)
  {
    for(uint8_t i=0; i<6; i++)
    {
      gain_r(i) = msg->data.at(i);
    }
  }


  void STArmPlugin::GripperStateCallback(const std_msgs::Float32ConstPtr &msg)
  {
    // float ref_gripper_state = msg->data;
    float ref_gripper_state = Map(msg->data, 0, 2.3, -0.03, 0);
    
    for (uint8_t i=6; i<NUM_OF_JOINTS_WITH_TOOL; i++)
    {
      ref_th[i] = ref_gripper_state;
    }
  }


  void STArmPlugin::GripperControl()
  {
    gain_p_joint_space[6] = 100; 
    gain_p_joint_space[7] = 100; 
    gain_d_joint_space[6] = 1; 
    gain_d_joint_space[7] = 1; 
    for (uint8_t i=6; i<NUM_OF_JOINTS_WITH_TOOL; i++)
    {
      joint_torque[i] = gain_p_joint_space[i] * (ref_th[i] - th[i]) - gain_d_joint_space[i] * th_dot[i];
    }
  }


  float STArmPlugin::Map(float x, float in_min, float in_max, float out_min, float out_max)
  {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }


  void STArmPlugin::RBQ3Motion1()
  {
    traj_time = cnt * inner_dt;
    frequency << 0.2, 0.2, 0.2;
    amplitude << 1, 1, 1;
    horizontal_translation << 0, 1, 0;
    vertical_translation << 0, 0, 0;
    rbq3_base_range_of_motion << 5, 5, 5;
    // rbq3_base_range_of_motion << 0, 0, 0;

    for(uint8_t i=0; i<3; i++)
    {
      rbq3_ref_trajectory[i] = amplitude[i] * sin(2 * PI * frequency[i] * (traj_time - horizontal_translation[i])) + vertical_translation[i];
      rbq3_base_rpy_ref[i] = DEG2RAD * rbq3_base_range_of_motion[i] * rbq3_ref_trajectory[i];
    }

    rbq_base_gain_p = 1000;
    rbq_base_gain_d = 50;

    rbq3_base_torque = rbq_base_gain_p * (rbq3_base_rpy_ref - rbq3_base_rpy) - rbq_base_gain_d * rbq3_base_rpy_dot;

    float quad_js_p = 100;
    float quad_js_d = 0;

    quad_th_ref << 0, 60, -90,
                   0, 60, -90,
                   0, 60, -90,
                   0, 60, -90;
    for(uint8_t i=0; i<12; i++)
    {
      quad_joint_torque[i] = quad_js_p * (quad_th_ref[i] * DEG2RAD - quad_th[i]) - quad_js_d * quad_th_dot[i];
    }
  }


  void STArmPlugin::RBQ3Motion2()
  {
    traj_time = cnt * inner_dt;
    frequency << 0.2, 0.2, 0.2;
    amplitude << 1, 1, 1;
    horizontal_translation << 0, 1, 0;
    vertical_translation << 0, 0, 0;
    // rbq3_base_range_of_motion << 15, 15, 15;
    rbq3_base_range_of_motion << 0, 0, 0;

    for(uint8_t i=0; i<3; i++)
    {
      rbq3_ref_trajectory[i] = amplitude[i] * sin(2 * PI * frequency[i] * (traj_time - horizontal_translation[i])) + vertical_translation[i];
      rbq3_base_rpy_ref[i] = DEG2RAD * rbq3_base_range_of_motion[i] * rbq3_ref_trajectory[i];
    }

    rbq_base_gain_p = 1000;
    rbq_base_gain_d = 50;

    rbq3_base_torque = rbq_base_gain_p * (rbq3_base_rpy_ref - rbq3_base_rpy) - rbq_base_gain_d * rbq3_base_rpy_dot;

    float quad_js_p = 300;
    float quad_js_d = 1;

    Vector3d rr_q, rl_q, fr_q, fl_q;

    Vector3d reference_position_right, reference_position_left;

    reference_position_right << 0.01, -0.05, -0.3;
    reference_position_left << 0.01, 0.05, -0.3;

    rr_q = GetRBQ3RightIK(reference_position_right);
    fr_q = GetRBQ3RightIK(reference_position_right);

    rl_q = GetRBQ3LeftIK(reference_position_left);
    fl_q = GetRBQ3LeftIK(reference_position_left);

    quad_th_ref << rr_q(0), rr_q(1), rr_q(2),
                   rl_q(0), rl_q(1), rl_q(2),
                   fr_q(0), fr_q(1), fr_q(2),
                   fl_q(0), fl_q(1), fl_q(2);

    for(uint8_t i=0; i<12; i++)
    {
      quad_joint_torque(i) = quad_js_p * (quad_th_ref(i) - quad_th(i)) - quad_js_d * quad_th_dot(i);
    }



    // l_HT0 << 1, 0, 0, 0,
    //           0, 1, 0, 0,
    //           0, 0, 1, 0,
    //           0, 0, 0, 1;
    // l_HT1 << cosf(a_th[0]), 0, -sinf(a_th[0]), 0,
    //           sinf(a_th[0]), 0, cosf(a_th[0]), 0,
    //           0, -1, 0, 0,
    //           0, 0, 0, 1;
    // l_HT4 << sinf(a_th[3]), 0, cosf(a_th[3]), 0,
    //           -cosf(a_th[3]), 0, sinf(a_th[3]), 0,
    //           0, -1, 0, 0,
    //           0, 0, 0, 1;
    // l_HT5 << -sinf(a_th[4]), 0, cosf(a_th[4]), 0,
    //           cosf(a_th[4]), 0, sinf(a_th[4]), 0,
    //           0, 1, 0, 0,
    //           0, 0, 0, 1;





  }

  
  Vector3d STArmPlugin::GetRBQ3RightIK(Vector3d position)
  {
    // const float d1 = 0.29785, d2 = 0.055, d3 = 0.110945, d4 = 0.3205, d5 = 0.025, d6 = 0.3395, d3r = -0.110945;
    const float d1 = 0.22445, d2 = 0.07946, d3 = 0.06995, d4 = 0.225, d5 = 0.0, d6 = 0.225, d3r = -0.06995;

    Vector3d q;

    float alpha = atan2(-1 * position(2), position(1));
    float beta = atan2(sqrt(position(1) * position(1) + position(2) * position(2) - d3 * d3), d3r);
    q(0) = beta - alpha;

    float dESquare = position(0) * position(0) + position(1) * position(1) + position(2) * position(2);
    float d43 = sqrt(d4 * d4 + d3 * d3);
    float cosSigma = (dESquare - (d43 * d43 + d6 * d6 + d3 * d3)) / (2 * d43 * d6);
    float sinSigma = -sqrt(1 - (cosSigma * cosSigma));
    float sigma = atan2(d6 * sinSigma, d6 * cosSigma);
    q(2) = sigma;

    alpha = atan2(-1 * position(0), sqrt(position(1) * position(1) + position(2) * position(2) - d3 * d3));
    beta = atan2(d6 * sinSigma, d4 + d6 * cosSigma);
    q(1) = alpha - beta;

    return q;
  }


  Vector3d STArmPlugin::GetRBQ3LeftIK(Vector3d position)
  {
    // const float d1 = 0.29785, d2 = 0.055, d3 = 0.110945, d4 = 0.3205, d5 = 0.025, d6 = 0.3395, d3r = -0.110945;
    const float d1 = 0.22445, d2 = 0.07946, d3 = 0.06995, d4 = 0.225, d5 = 0.0, d6 = 0.225, d3r = -0.06995;

    Vector3d q;

    float alpha = atan2(-1 * position(2), position(1));
    float beta = atan2(sqrt(position(1) * position(1) + position(2) * position(2) - d3 * d3), d3);
    q(0) = beta - alpha;

    float dESquare = position(0) * position(0) + position(1) * position(1) + position(2) * position(2);
    float d43 = sqrt(d4 * d4 + d3 * d3);
    float cosSigma = (dESquare - (d43 * d43 + d6 * d6 + d3 * d3)) / (2 * d43 * d6);
    float sinSigma = -sqrt(1 - (cosSigma * cosSigma));
    float sigma = atan2(d6 * sinSigma, d6 * cosSigma);
    q(2) = sigma;

    alpha = atan2(-1 * position(0), sqrt(position(1) * position(1) + position(2) * position(2) - d3 * d3));
    beta = atan2(d6 * sinSigma, d4 + d6 * cosSigma);
    q(1) = alpha - beta;

    return q;
  }


  void STArmPlugin::SwitchModeRBQ3(const std_msgs::Bool &msg)
  {
    is_move_rbq3 = &msg;
  }


  void STArmPlugin::GetRBQ3Joints()
  {
    this->HRR = this->model->GetJoint("RR_hip_joint");
    this->HRP = this->model->GetJoint("RR_thigh_joint");
    this->HRK = this->model->GetJoint("RR_calf_joint");
    this->HLR = this->model->GetJoint("RL_hip_joint");
    this->HLP = this->model->GetJoint("RL_thigh_joint");
    this->HLK = this->model->GetJoint("RL_calf_joint");
    this->FRR = this->model->GetJoint("FR_hip_joint");
    this->FRP = this->model->GetJoint("FR_thigh_joint");
    this->FRK = this->model->GetJoint("FR_calf_joint");
    this->FLR = this->model->GetJoint("FL_hip_joint");
    this->FLP = this->model->GetJoint("FL_thigh_joint");
    this->FLK = this->model->GetJoint("FL_calf_joint");

    this->rbq3_base_joint = this->model->GetJoint("rbq3_base_joint");
  }


  void STArmPlugin::GetRBQ3JointPosition()
  {
    quad_th[0] = this->HRR->Position(0);
    quad_th[1] = this->HRP->Position(1);
    quad_th[2] = this->HRK->Position(1);
    quad_th[3] = this->HLR->Position(0);
    quad_th[4] = this->HLP->Position(1);
    quad_th[5] = this->HLK->Position(1);
    quad_th[6] = this->FRR->Position(0);
    quad_th[7] = this->FRP->Position(1);
    quad_th[8] = this->FRK->Position(1);
    quad_th[9] = this->FLR->Position(0);
    quad_th[10] = this->FLP->Position(1);
    quad_th[11] = this->FLK->Position(1);

    rbq3_base_rpy[0] = this->rbq3_base_joint->Position(0);
    rbq3_base_rpy[1] = this->rbq3_base_joint->Position(1);
  }


  void STArmPlugin::GetRBQ3JointVelocity()
  {
    // quad_th_dot[0] = this->HRR->GetVelocity(0);
    // quad_th_dot[1] = this->HRP->GetVelocity(1);
    // quad_th_dot[2] = this->HRK->GetVelocity(1);
    // quad_th_dot[3] = this->HLR->GetVelocity(0);
    // quad_th_dot[4] = this->HLP->GetVelocity(1);
    // quad_th_dot[5] = this->HLK->GetVelocity(1);
    // quad_th_dot[6] = this->FRR->GetVelocity(0);
    // quad_th_dot[7] = this->FRP->GetVelocity(1);
    // quad_th_dot[8] = this->FRK->GetVelocity(1);
    // quad_th_dot[9] = this->FLR->GetVelocity(0);
    // quad_th_dot[10] = this->FLP->GetVelocity(1);
    // quad_th_dot[11] = this->FLK->GetVelocity(1);

    for(uint8_t i=0; i<12; i++)
    {
      if(dt > 0.0005) quad_th_dot[i] = (quad_th[i] - quad_last_th[i]) / dt;
      quad_last_th[i] = quad_th[i];
    }

    for(uint8_t i=0; i<12; i++)
    {
      if(abs(quad_th_dot[i]) > JOINT_VEL_LIMIT) quad_th_dot[i] = 0;
    }
    

    rbq3_base_rpy_dot[0] = this->rbq3_base_joint->GetVelocity(0);
    rbq3_base_rpy_dot[1] = this->rbq3_base_joint->GetVelocity(1);
  }


  void STArmPlugin::SetRBQ3JointTorque()
  {
    this->HRR->SetForce(0, quad_joint_torque(0));
    this->HRP->SetForce(1, quad_joint_torque(1));
    this->HRK->SetForce(1, quad_joint_torque(2));
    this->HLR->SetForce(0, quad_joint_torque(3));
    this->HLP->SetForce(1, quad_joint_torque(4));
    this->HLK->SetForce(1, quad_joint_torque(5));
    this->FRR->SetForce(0, quad_joint_torque(6));
    this->FRP->SetForce(1, quad_joint_torque(7));
    this->FRK->SetForce(1, quad_joint_torque(8));
    this->FLR->SetForce(0, quad_joint_torque(9));
    this->FLP->SetForce(1, quad_joint_torque(10));
    this->FLK->SetForce(1, quad_joint_torque(11));

    this->rbq3_base_joint->SetForce(0, rbq3_base_torque(0));
    this->rbq3_base_joint->SetForce(1, rbq3_base_torque(1));
  }


  bool STArmPlugin::InverseSolverUsingJacobian(Vector3d a_target_position, Matrix3d a_target_orientation)
  {
    //solver parameter
    const double lambda = 0.1;
    const int8_t iteration = 80;
    const double tolerance = 0.0001;

    VectorXd l_q = VectorXd::Zero(6);

    for(int8_t i=0; i<6; i++)
    {
      l_q(i) = th(i);
    }

    //delta parameter
    VectorXd pose_difference = VectorXd::Zero(6);
    VectorXd l_delta_q = VectorXd::Zero(6);

    for (int count = 0; count < iteration; count++)
    {
      UpdateJacobian(l_q);

      pose_difference = PoseDifference(a_target_position, a_target_orientation, fk_current_position, fk_current_orientation);

      if (pose_difference.norm() < tolerance)
      {
        // std::cout << "Solved IK" << std::endl;
        std::cout << "Number of iterations: " << count << "     normalized pose difference: " << pose_difference.norm() << std::endl;
        ik_th = l_q;
        return true;
      }
      //get delta angle
      Eigen::ColPivHouseholderQR<MatrixXd> dec(JacobianForIK);
      l_delta_q = lambda * dec.solve(pose_difference);
      for(uint8_t i=0; i<6; i++)
      {
        l_q[i] += l_delta_q[i];
      }
    }
    for(uint8_t i=0; i<6; i++)
    {
      // ik_th(i) = l_q(i);
      ik_current_pose(i) = pose_difference(i);
    }
    return false;
  }


  bool STArmPlugin::InverseSolverUsingSRJacobian(Vector3d a_target_position, Matrix3d a_target_orientation)
  {
    VectorXd l_q = VectorXd::Zero(6);
    VectorXd l_delta_q = VectorXd::Zero(6);

    for(uint8_t i=0; i<6; i++)
    {
      l_q[i] = th[i];
    }

    Matrix4d l_current_pose;

    MatrixXd l_jacobian = MatrixXd::Identity(6, 6);

    //solver parameter
    double lambda = 0.0;
    const double param = 0.002;
    const int8_t iteration = 10;

    const double gamma = 0.5;             //rollback delta

    //sr sovler parameter
    double wn_pos = 1 / 0.3;
    double wn_ang = 1 / (2 * M_PI);
    double pre_Ek = 0.0;
    double new_Ek = 0.0;

    MatrixXd We(6, 6);
    We << wn_pos, 0, 0, 0, 0, 0,
        0, wn_pos, 0, 0, 0, 0,
        0, 0, wn_pos, 0, 0, 0,
        0, 0, 0, wn_ang, 0, 0,
        0, 0, 0, 0, wn_ang, 0,
        0, 0, 0, 0, 0, wn_ang;

    MatrixXd Wn = MatrixXd::Identity(6,6);

    MatrixXd sr_jacobian = MatrixXd::Identity(6, 6);

    //delta parameter
    VectorXd pose_difference = VectorXd::Zero(6);
    VectorXd gerr(6);

    GetJacobians(l_q, l_jacobian, l_current_pose);
    
    pose_difference = PoseDifference(a_target_position, a_target_orientation, l_current_pose);
    pre_Ek = pose_difference.transpose() * We * pose_difference;

    for (int8_t count = 0; count < iteration; count++)
    {
      GetJacobians(l_q, l_jacobian, l_current_pose);

      pose_difference = PoseDifference(a_target_position, a_target_orientation, l_current_pose);
      
      new_Ek = pose_difference.transpose() * We * pose_difference;
      
      lambda = pre_Ek + param;
      
      sr_jacobian = (l_jacobian.transpose() * We * l_jacobian) + (lambda * Wn);     //calculate sr_jacobian (J^T*we*J + lamda*Wn)
      gerr = l_jacobian.transpose() * We * pose_difference;                         //calculate gerr (J^T*we) dx
      Eigen::ColPivHouseholderQR<MatrixXd> dec(sr_jacobian);                        //solving (get dq)
      l_delta_q = dec.solve(gerr);                                                  //(J^T*we) * dx = (J^T*we*J + lamda*Wn) * dq

      l_q = l_q + l_delta_q;

      if (new_Ek < 1E-12)
      {
        std::cout << "Solved IK" << std::endl;
        ik_th = l_q;
        return true;
      }
      else if (new_Ek < pre_Ek)
      {
        pre_Ek = new_Ek;
      }
      else
      {
        l_q = l_q - gamma * l_delta_q;
      }
    }
    ik_th = l_q;
    // std::cout << "Solved IK" << std::endl;
    return false;
  }


  bool STArmPlugin::IK(Vector3d a_target_position, Matrix3d a_target_orientation)
  {
    int it = 0;
    int max_it = 10;
    float tolerance = 0.1;
    float alpha = 0.1;
    float best_norm;
    VectorXd pose_difference = VectorXd::Zero(6);
    VectorXd l_q = VectorXd::Zero(6);
    VectorXd l_best_q = VectorXd::Zero(6);
    VectorXd l_delta_q = VectorXd::Zero(6);
    MatrixXd l_Jacobian = MatrixXd::Zero(6, 6);
    MatrixXd l_Jacobian_inverse = MatrixXd::Zero(6, 6);
    Matrix4d l_current_pose;


    // while ((it == 0 || pose_difference.norm() > tolerance) && it < max_it)
    while ((it == 0 || 1 > tolerance) && it < max_it)
    {
      // GetJacobians(l_q, l_Jacobian, l_current_pose);

      // l_Jacobian_inverse = getDampedPseudoInverse(l_Jacobian, 0);
      l_Jacobian_inverse = l_Jacobian.inverse();

      pose_difference = PoseDifference(a_target_position, a_target_orientation, l_current_pose);

      // l_delta_q = alpha * l_Jacobian_inverse * pose_difference;

      // for(uint8_t i=0; i<6; i++)
      // {
      //   l_q[i] = l_q[i] + l_delta_q[i];
      // }

      // if(it == 0 || pose_difference.norm() < best_norm)
      // {
      //   l_best_q = l_q;
      //   best_norm = pose_difference.norm();
      // }
      it++;
    }
    for(uint8_t i=0; i<6; i++)
    {
      ik_th[i] = l_best_q[i];
    }

    if(it < max_it)
    {
      std::cout << "Did converge, iteration number: " << it << "    pose difference normalized: " << pose_difference.norm() << std::endl;
      return true;
    } 

    std::cout << "Did not converge, iteration number: " << it << "    pose difference normalized: " << (float)pose_difference.norm() << std::endl;

    std::cout << "Here is the matrix J:\n" << l_Jacobian << std::endl;
    std::cout << "\n Here is the matrix J_inv:\n" << l_Jacobian_inverse << std::endl;

    return false;
  }


  VectorXd STArmPlugin::PoseDifference(Vector3d a_desired_position, Matrix3d a_desired_orientation, Vector3d a_present_position, Matrix3d a_present_orientation)
  {
    Vector3d l_position_difference, l_orientation_difference;
    VectorXd l_pose_difference(6);

    l_orientation_difference = Vector3d::Zero(3);

    l_position_difference = PositionDifference(a_desired_position, a_present_position);
    l_orientation_difference = OrientationDifference(a_desired_orientation, a_present_orientation);
    l_pose_difference << l_position_difference(0), l_position_difference(1), l_position_difference(2),
                        l_orientation_difference(0), l_orientation_difference(1), l_orientation_difference(2);

    return l_pose_difference;
  }


  Vector3d STArmPlugin::PositionDifference(Vector3d desired_position, Vector3d present_position)
  {
    Vector3d position_difference;
    position_difference = desired_position - present_position;

    return position_difference;
  }


  Vector3d STArmPlugin::OrientationDifference(Matrix3d desired_orientation, Matrix3d present_orientation)
  {
    Vector3d orientation_difference;
    orientation_difference = present_orientation * MatrixLogarithm(present_orientation.transpose() * desired_orientation);

    return orientation_difference;
  }


  Vector3d STArmPlugin::MatrixLogarithm(Matrix3d rotation_matrix)
  {
    Matrix3d R = rotation_matrix;
    Vector3d l = Vector3d::Zero();
    Vector3d rotation_vector = Vector3d::Zero();

    double theta = 0.0;
    // double diag = 0.0;
    bool diagonal_matrix = R.isDiagonal();

    l << R(2, 1) - R(1, 2),
        R(0, 2) - R(2, 0),
        R(1, 0) - R(0, 1);
    theta = atan2(l.norm(), R(0, 0) + R(1, 1) + R(2, 2) - 1);
    // diag = R.determinant();

    if (R.isIdentity())
    {
      rotation_vector.setZero(3);
      return rotation_vector;
    }
    
    if (diagonal_matrix == true)
    {
      rotation_vector << R(0, 0) + 1, R(1, 1) + 1, R(2, 2) + 1;
      rotation_vector = rotation_vector * M_PI_2;
    }
    else
    {
      rotation_vector = theta * (l / l.norm());
    }
    return rotation_vector;
  }


  Matrix3d STArmPlugin::skewSymmetricMatrix(Vector3d v)
  {
    Matrix3d skew_symmetric_matrix = Matrix3d::Zero();
    skew_symmetric_matrix << 0,     -v(2),      v(1),
                             v(2),      0,     -v(0),
                            -v(1),   v(0),         0;
    return skew_symmetric_matrix;
  }


  MatrixXd STArmPlugin::getDampedPseudoInverse(MatrixXd Jacobian, float lamda)
  {
    MatrixXd a_Jacobian_Transpose = MatrixXd::Zero(6,6);
    MatrixXd a_damped_psudo_inverse_Jacobian = MatrixXd::Zero(6,6);

    a_Jacobian_Transpose = Jacobian.transpose();

    a_damped_psudo_inverse_Jacobian = (a_Jacobian_Transpose * Jacobian + lamda * lamda * MatrixXd::Identity(6,6)).inverse() * a_Jacobian_Transpose;

    return a_damped_psudo_inverse_Jacobian;
  }

  // a argument   // m member    // l local   //p pointer     //r reference   //
  void STArmPlugin::UpdateJacobian(VectorXd a_th)
  {
    Matrix4d l_HT0, l_HT1, l_HT2, l_HT3, l_HT4, l_HT5, l_HT6;
    Matrix4d l_T00, l_T01, l_T02, l_T03, l_T04, l_T05, l_T06;
    Vector3d l_a0, l_a1, l_a2, l_a3, l_a4, l_a5;
    Vector3d l_P6_P0, l_P6_P1, l_P6_P2, l_P6_P3, l_P6_P4, l_P6_P5;
    VectorXd l_J1(6), l_J2(6), l_J3(6), l_J4(6), l_J5(6), l_J6(6); 

    l_HT0 << 1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 1, 0,
          0, 0, 0, 1;
    l_HT1 << cosf(a_th[0]), 0, -sinf(a_th[0]), 0,
          sinf(a_th[0]), 0, cosf(a_th[0]), 0,
          0, -1, 0, L1,
          0, 0, 0, 1;
    l_HT2 << cosf(a_th[1]), -sinf(a_th[1]), 0, L2*cosf(a_th[1]),
          sinf(a_th[1]), cosf(a_th[1]), 0, L2*sinf(a_th[1]),
          0, 0, 1, 0, 
          0, 0, 0, 1;
    l_HT3 << cosf(a_th[2]), -sinf(a_th[2]), 0, L3*cosf(a_th[2]), 
          sinf(a_th[2]), cosf(a_th[2]), 0, L3*sinf(a_th[2]), 
          0, 0, 1, 0,
          0, 0, 0, 1;
    l_HT4 << sinf(a_th[3]), 0, cosf(a_th[3]), 0,
          -cosf(a_th[3]), 0, sinf(a_th[3]), 0,
          0, -1, 0, 0,
          0, 0, 0, 1;
    l_HT5 << -sinf(a_th[4]), 0, cosf(a_th[4]), 0,
          cosf(a_th[4]), 0, sinf(a_th[4]), 0,
          0, 1, 0, L5,
          0, 0, 0, 1;
    l_HT6 << -sinf(a_th[5]), -cosf(a_th[5]), 0, -L6*sinf(a_th[5]),
          cosf(a_th[5]), -sinf(a_th[5]), 0, L6*cosf(a_th[5]),
          0, 0, 1, 0, 
          0, 0, 0, 1;

    l_T00 = l_HT0;
    l_T01 = l_T00 * l_HT1;
    l_T02 = l_T01 * l_HT2;
    l_T03 = l_T02 * l_HT3;
    l_T04 = l_T03 * l_HT4;
    l_T05 = l_T04 * l_HT5;
    l_T06 = l_T05 * l_HT6;


    l_a0 << l_T00(0,2), l_T00(1,2), l_T00(2,2);
    l_a1 << l_T01(0,2), l_T01(1,2), l_T01(2,2);
    l_a2 << l_T02(0,2), l_T02(1,2), l_T02(2,2);
    l_a3 << l_T03(0,2), l_T03(1,2), l_T03(2,2);
    l_a4 << l_T04(0,2), l_T04(1,2), l_T04(2,2);
    l_a5 << l_T05(0,2), l_T05(1,2), l_T05(2,2);

    l_P6_P0 << l_T06(0,3) - l_T00(0,3), l_T06(1,3) - l_T00(1,3), l_T06(2,3) - l_T00(2,3);
    l_P6_P1 << l_T06(0,3) - l_T01(0,3), l_T06(1,3) - l_T01(1,3), l_T06(2,3) - l_T01(2,3);
    l_P6_P2 << l_T06(0,3) - l_T02(0,3), l_T06(1,3) - l_T02(1,3), l_T06(2,3) - l_T02(2,3);
    l_P6_P3 << l_T06(0,3) - l_T03(0,3), l_T06(1,3) - l_T03(1,3), l_T06(2,3) - l_T03(2,3);
    l_P6_P4 << l_T06(0,3) - l_T04(0,3), l_T06(1,3) - l_T04(1,3), l_T06(2,3) - l_T04(2,3);
    l_P6_P5 << l_T06(0,3) - l_T05(0,3), l_T06(1,3) - l_T05(1,3), l_T06(2,3) - l_T05(2,3);

    l_J1 << l_a0.cross(l_P6_P0), l_a0;
    l_J2 << l_a1.cross(l_P6_P1), l_a1;
    l_J3 << l_a2.cross(l_P6_P2), l_a2;
    l_J4 << l_a3.cross(l_P6_P3), l_a3;
    l_J5 << l_a4.cross(l_P6_P4), l_a4;
    l_J6 << l_a5.cross(l_P6_P5), l_a5;

    JacobianForIK << l_J1, l_J2, l_J3, l_J4, l_J5, l_J6;

    // a_Current_Pose = l_T06;

    fk_current_position << l_T06(0,3), l_T06(1,3), l_T06(2,3);
    fk_current_orientation = l_T06.block<3,3>(0,0);

    // ik_current_pose << a_current_position(0), a_current_position(1), a_current_position(2), a_th(0), a_th(1), a_th(2);
  }


  void STArmPlugin::Calc_Feedback_Pose(Arm_RBDL &rbdl)
  {

    double pi = 0, theta = 0, psi = 0;
    arm_rbdl.jacobian_swap = RBDLMatrixNd::Zero(6,6);

    //Get the EE Pose
    arm_rbdl.ee_pos_act = RBDL::CalcBodyToBaseCoordinates(*arm_rbdl.rbdl_model, arm_rbdl.q, arm_rbdl.gripper_id, RBDLVector3d(0,0,0), true);  //Position 
    arm_rbdl.ee_ori_act = RBDL::CalcBodyWorldOrientation(*arm_rbdl.rbdl_model, arm_rbdl.q, arm_rbdl.gripper_id, true);  //Orientation

    ee_position = arm_rbdl.ee_pos_act;

    arm_rbdl.ee_ori_act_trans = arm_rbdl.ee_ori_act;//.transpose();


    // //z - y - z
    // arm_rbdl.rpy_ee = arm_rbdl.ee_ori_act_trans.eulerAngles(2, 1, 2);
    // pi = arm_rbdl.rpy_ee(0);
    // theta = arm_rbdl.rpy_ee(1);
    // psi = arm_rbdl.rpy_ee(2);

    // Get the Euler Angle z-y-x rotation matrix
    pi = atan2(arm_rbdl.ee_ori_act_trans(1,0),arm_rbdl.ee_ori_act_trans(0,0));
    theta = atan2(- arm_rbdl.ee_ori_act_trans(2,0), cos(pi)*arm_rbdl.ee_ori_act_trans(0,0) + sin(pi)*arm_rbdl.ee_ori_act_trans(1,0));
    psi = atan2(sin(pi)*arm_rbdl.ee_ori_act_trans(0,2) - cos(pi)*arm_rbdl.ee_ori_act_trans(1,2), -sin(pi)*arm_rbdl.ee_ori_act_trans(0,1) + cos(pi)*arm_rbdl.ee_ori_act_trans(1,1));

    // Get the Geometric Jacobian
    RBDL::CalcPointJacobian6D(*arm_rbdl.rbdl_model, arm_rbdl.q, arm_rbdl.gripper_id, RBDLVector3d(0,0,0), arm_rbdl.jacobian_swap, true);

    // Change the Row
    for(int j = 0; j < 6; j++)
    {
      for(int i = 0; i < 3; i++)
      {
       arm_rbdl.jacobian(i,j) = arm_rbdl.jacobian_swap(i+3,j);  // linear // 저장한게 방위,위치 순이어서 이를 위치, 방위순으로 재정렬
      }
      for(int i = 3; i < 6; i++)
      {
        arm_rbdl.jacobian(i,j) = arm_rbdl.jacobian_swap(i-3,j);  // angular
      }
    }

    // // Calculate the Analytical Jacobian & Inverse of Analytical Jacobian z-y-z
    // arm_rbdl.geometric_to_analytic << 1, 0, 0, 0, 0, 0,
    //                                   0, 1, 0, 0, 0, 0,
    //                                   0, 0, 1, 0, 0, 0,
    //                                   0, 0, 0, cos(psi) / sin(theta), sin(psi) / sin(theta), 0,
    //                                   0, 0, 0, -sin(psi), cos(psi), 0,
    //                                   0, 0, 0, -cos(psi) * tan(theta), -sin(psi) * tan(theta), 1;

    // Calculate the Analytical Jacobian & Inverse of Analytical Jacobian z-y-x
    arm_rbdl.geometric_to_analytic << 1, 0, 0, 0, 0, 0,
                                      0, 1, 0, 0, 0, 0,
                                      0, 0, 1, 0, 0, 0,
                                      0, 0, 0, cos(pi) / cos(theta), sin(pi) / cos(theta), 0,
                                      0, 0, 0, -sin(pi), cos(pi), 0,
                                      0, 0, 0, cos(pi) * tan(theta), sin(pi) * tan(theta), 1;

    // Analytical Jacobin
    arm_rbdl.jacobian_ana = arm_rbdl.geometric_to_analytic * arm_rbdl.jacobian;

    // Inverse of Analytical Jacobian
    arm_rbdl.jacobian_ana_inverse = arm_rbdl.jacobian_ana.inverse();

    for(int i = 0; i<6; i++)
    {
      for(int j = 0; j<6; j++){
        arm_rbdl.jacobian_ana_dot(i,j) = (arm_rbdl.jacobian_ana(i,j)-arm_rbdl.jacobian_prev(i,j)) / dt;
        arm_rbdl.jacobian_prev(i,j) = arm_rbdl.jacobian_ana(i,j);
      }
    }

    // Storing Actual EE Pose
    arm_rbdl.x_actual << arm_rbdl.ee_pos_act, psi, theta, pi;

    // Calculate Actual EE Velocity
    arm_rbdl.x_actual_dot = arm_rbdl.jacobian_ana * arm_rbdl.q_dot;

  }
}


