#include "spi2can.h"
#include "rt_utils.h"
#include "dynamics.h"
#include "motor_controller.h"
#include "dynamixel.h"
#include "callback.h"

static void *rt_motion_thread(void *arg);
static void *rt_dynamixel_thread(void *arg);
pRBCORE_SHM sharedData;
rmd_motor _DEV_MC[3];
ROBOT_STATE_DATA ros_data;
Dynamics::JMDynamics jm_dynamics;
Motor_Controller motor_ctrl;
Dynamixel _WRIST_MC;
Callback callback;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "st_arm");
    ros::Time::init();
    ros::Rate loop_rate(100);
    ros::NodeHandle node_handle_;

    ros::Publisher st_arm_joint_states_pub_;
    st_arm_joint_states_pub_ = node_handle_.advertise<sensor_msgs::JointState>("st_arm/joint_states", 100);

    ros::Subscriber switch_mode_sub_;
    switch_mode_sub_ = node_handle_.subscribe("st_arm/switch_mode", 10, &Callback::SwitchMode, &callback);

    ros::Subscriber gain_p_sub_;
    gain_p_sub_ = node_handle_.subscribe("st_arm/gain_p", 10, &Callback::SwitchGainP, &callback);

    ros::Subscriber gain_d_sub_;
    gain_d_sub_ = node_handle_.subscribe("st_arm/gain_d", 10, &Callback::SwitchGainD, &callback);

    ros::Subscriber gain_r_sub_;
    gain_r_sub_ = node_handle_.subscribe("st_arm/gain_r", 10, &Callback::SwitchGainR, &callback);

    ros::Subscriber gain_task_space_p_sub_;
    gain_task_space_p_sub_ = node_handle_.subscribe("st_arm/gain_TS_P", 10, &Callback::SwitchGainTaskSpaceP, &callback);

    ros::Subscriber gain_task_space_w_sub_;
    gain_task_space_w_sub_ = node_handle_.subscribe("st_arm/gain_TS_W", 10, &Callback::SwitchGainTaskSpaceW, &callback);

    ros::Subscriber pose_initializer_sub_;
    pose_initializer_sub_ = node_handle_.subscribe("st_arm/init_pose", 10, &Callback::InitializePose, &callback);

    ros::Subscriber gripper_state_sub_;
    gripper_state_sub_ = node_handle_.subscribe("unity/gripper_state", 10, &Callback::GripperCallback, &callback);

    ros::Subscriber virtual_box_pose_sub_;
    virtual_box_pose_sub_ = node_handle_.subscribe("unity/virtual_box_pose", 10, &Callback::HMDTFCallback, &callback);

    spi2can::getInstance();

    sharedData = (pRBCORE_SHM)malloc(sizeof(RBCORE_SHM));

    pthread_t thread_motion;
    pthread_t thread_dynamixel;

    int thread_id_motion = generate_rt_thread(thread_motion, rt_motion_thread, "motion_thread", 3, 95, NULL);
    int thread_id_dynamixel = generate_rt_thread(thread_dynamixel, rt_dynamixel_thread, "dynamixel_thread", 3, 96, NULL);

    while(ros::ok())
    {
        sensor_msgs::JointState msg;
        msg.header.stamp = ros::Time::now();

        std::vector<std::string> joints_name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper"};

        for (uint8_t i = 0; i<7; i ++)
        {
            msg.name.push_back(joints_name.at(i));
            msg.position.push_back(jm_dynamics.th[i]);
            msg.velocity.push_back(jm_dynamics.th_dot_sma_filtered[i]);
            msg.effort.push_back(jm_dynamics.joint_torque[i]);
        }

        for (uint8_t i = 0; i<3; i ++)
        {
            msg.effort.push_back(_DEV_MC[i].GetTorque());
            // msg.position.push_back(jm_dynamics.ref_th[i]);
            // msg.velocity.push_back(jm_dynamics.th_dot[i]);
            // msg.velocity.push_back(jm_dynamics.th_dot_estimated[i]);
        }
        st_arm_joint_states_pub_.publish(msg);
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    _WRIST_MC.~Dynamixel();
    return 0;
}


void *rt_motion_thread(void *arg){
    const long PERIOD_US = RT_MS * 1000;
    struct timespec TIME_NEXT;
    struct timespec TIME_NOW;
    int loop_count = 0;
    int comm_loop_count = 0;
    int comm_loop_count_time_sec = 0;
    bool is_print_comm_frequency = true;

    clock_gettime(CLOCK_REALTIME, &TIME_NEXT);

    bool is_first_loop = true;

    while(true){

        if(is_first_loop){
            motor_ctrl.EnableMotor();
            // motor_ctrl.EnableFilter();
            timespec_add_us(&TIME_NEXT, 4 * 1000 * 1000);
            is_first_loop = false;
            loop_count++;
        }
        else if(loop_count > 1000){
            loop_count++;
            jm_dynamics.Loop();
            // jm_dynamics.SetTheta(motor_ctrl.GetJointTheta());
            // jm_dynamics.SetThetaDotSMAF(motor_ctrl.GetThetaDotSMAF());
            // jm_dynamics.GenerateTorqueManipulationMode();
            // jm_dynamics.GenerateTorqueVisionMode();
            // jm_dynamics.GenerateGripperTorque();
            // motor_ctrl.SetTorque(jm_dynamics.GetTorque());
            // motor_ctrl.EnableFilter();

            if(comm_loop_count > 500 && is_print_comm_frequency) {
                comm_loop_count = 1;
                if(comm_loop_count_time_sec < 120)
                {
                    comm_loop_count_time_sec++;
                    std::cout << "    M1 fbcnt: total: " << _DEV_MC[0].count; std::cout << "  A1: " << _DEV_MC[0].count_A1;
                    std::cout << "    M2 fbcnt: total:" << _DEV_MC[1].count;  std::cout << "  A1: " << _DEV_MC[1].count_A1;
                    std::cout << "    M3 fbcnt: total:" << _DEV_MC[2].count;  std::cout << "  A1: " << _DEV_MC[2].count_A1 << std::endl;
                    // std::cout << "  92: " << _DEV_MC[2].count_92 << std::endl;
                    // std::cout << "    M3 unknown value:  " << _DEV_MC[2].unknown_value << std::endl;
                    for(uint8_t i=0;i<3;i++)
                    {
                        _DEV_MC[i].count = 0;
                        _DEV_MC[i].count_A1 = 0;
                        _DEV_MC[i].count_92 = 0;
                    }
                }
                else is_print_comm_frequency = false;
            }
            if(is_print_comm_frequency) comm_loop_count++;
        }
        else loop_count++;

        clock_gettime(CLOCK_REALTIME, &TIME_NOW);
        timespec_add_us(&TIME_NEXT, PERIOD_US);

        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &TIME_NEXT, NULL);
        if(timespec_cmp(&TIME_NOW, &TIME_NEXT) > 0){
            std::cout << "RT Deadline Miss, main controller :  " << timediff_us(&TIME_NEXT, &TIME_NOW)*0.001<<" ms"<< std::endl;
        }
    }
}


void *rt_dynamixel_thread(void *arg){
    const long PERIOD_US = 2.5 * 1000;
    struct timespec TIME_NEXT;
    struct timespec TIME_NOW;
    struct timespec TIME_TIC;
    struct timespec TIME_TOC;
    int loop_count = 0;
    int dead_miss_cnt = 0;

    clock_gettime(CLOCK_REALTIME, &TIME_NEXT);

    while(true){

        bool isRxTh{true}, isRxThDot{false}, isTxTorque{true};
        _WRIST_MC.Loop(isRxTh, isRxThDot, isTxTorque);

        clock_gettime(CLOCK_REALTIME, &TIME_TIC);
        _WRIST_MC.CalculateEstimatedThetaDot(timediff_us(&TIME_TOC, &TIME_TIC));
        clock_gettime(CLOCK_REALTIME, &TIME_TOC);

        clock_gettime(CLOCK_REALTIME, &TIME_NOW);
        timespec_add_us(&TIME_NEXT, PERIOD_US);
        if(timespec_cmp(&TIME_NOW, &TIME_NEXT) > 0) std::cout << "RT Deadline Miss, Dynamixel " << ++dead_miss_cnt << std::endl;
        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &TIME_NEXT, NULL);
    }
}