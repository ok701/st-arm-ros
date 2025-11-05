#include "rmd_motor.h"

rmd_motor::rmd_motor()
{

}

void rmd_motor::UpdateRxData(void) {
    joint_temperature = (int)(torque_data[1]);
    
    int temp_torque = (int)(torque_data[2] | (torque_data[3]<<8));
    joint_torque = temp_torque / torque_to_data;
    
    int temp_speed = (int)(torque_data[4] | (torque_data[5]<<8));
    if(temp_speed > 30000) temp_speed -= 65535;
    joint_velocity = 0.01 * temp_speed * actuator_direction;

    int temp_encoder = (int)(torque_data[6] | (torque_data[7]<<8));
    float temp_theta = temp_encoder / data_to_radian;
    float incremental_theta{0};
    if(initialize_position){
        joint_theta = joint_initial_position;
        motor_theta_last = temp_theta;
        initialize_position = false;
    }
    else{
        incremental_theta = temp_theta - motor_theta_last;
        motor_theta_last = temp_theta;
    }    
    if (incremental_theta > 4) incremental_theta -= 6.28319;
    else if (incremental_theta < -4) incremental_theta += 6.28319;
    joint_theta += incremental_theta / actuator_gear_ratio * actuator_direction;
}

float rmd_motor::GetTheta() 
{
    return joint_theta;
}

// V3 0x92
float rmd_motor::GetThetaV3() 
{
    int temp_enc = (int)(enc_data[4] | (enc_data[5]<<8) | (enc_data[6]<<16) | (enc_data[7]<<24));
    float temp_theta = temp_enc * 0.0174533 * 0.01 * actuator_direction;
    if(initialize_position)
    {
        joint_theta_offset_92 = temp_theta - joint_initial_position; 
        initialize_position = false;
    }
    joint_theta_92 = temp_theta - joint_theta_offset_92;
    return joint_theta_92;
}


float rmd_motor::GetThetaDot() 
{
    return joint_velocity;
}


float rmd_motor::GetTorque()
{
    return joint_torque;
}


