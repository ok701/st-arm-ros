#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/PoseStamped.h"
#include "callback.h"
#include "dynamics.h"
#include <vector>
#include <string>

Dynamics::JMDynamics jm_dynamics;
Callback callback;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "st_arm_sim");
    ros::NodeHandle nh;
    ros::Rate loop_rate(100);

    ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("st_arm/joint_states", 100);

    ros::Subscriber sub_mode = nh.subscribe("st_arm/switch_mode", 10, &Callback::SwitchMode, &callback);
    ros::Subscriber sub_gain_p = nh.subscribe("st_arm/gain_p", 10, &Callback::SwitchGainP, &callback);
    ros::Subscriber sub_gain_d = nh.subscribe("st_arm/gain_d", 10, &Callback::SwitchGainD, &callback);
    ros::Subscriber sub_gain_r = nh.subscribe("st_arm/gain_r", 10, &Callback::SwitchGainR, &callback);
    ros::Subscriber sub_ts_p = nh.subscribe("st_arm/gain_TS_P", 10, &Callback::SwitchGainTaskSpaceP, &callback);
    ros::Subscriber sub_ts_w = nh.subscribe("st_arm/gain_TS_W", 10, &Callback::SwitchGainTaskSpaceW, &callback);
    ros::Subscriber sub_init = nh.subscribe("st_arm/init_pose", 10, &Callback::InitializePose, &callback);
    ros::Subscriber sub_grip = nh.subscribe("unity/gripper_state", 10, &Callback::GripperCallback, &callback);
    ros::Subscriber sub_hmd = nh.subscribe("unity/virtual_box_pose", 10, &Callback::HMDTFCallback, &callback);

    std::vector<std::string> names = {"joint1","joint2","joint3","joint4","joint5","joint6","gripper"};

    while (ros::ok())
    {
        jm_dynamics.Loop();

        sensor_msgs::JointState msg;
        msg.header.stamp = ros::Time::now();
        msg.name = names;
        for (int i = 0; i < 7; ++i)
        {
            msg.position.push_back(jm_dynamics.th[i]);
            msg.velocity.push_back(jm_dynamics.th_dot_sma_filtered[i]);
            msg.effort.push_back(jm_dynamics.joint_torque[i]);
        }
        joint_pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
