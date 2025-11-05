#ifndef SHARED_MEMORY_H
#define SHARED_MEMORY_H

#include <unistd.h>

#include "rmd_motor.h"
#include "rmd_can.h"

#include "Eigen/Dense"
#include "Eigen/Core"

#define EXTERNAL

#define MAX_COMMAND_DATA        30

#define R2D         57.2957802
#define D2R         0.0174533

#define R2Df         57.2957802
#define D2Rf         0.0174533

///-------------------------------------------------------------------------------
///---------------------Communication Struct--------------------------------------
///-------------------------------------------------------------------------------

// from GUI to Upboard
typedef struct _USER_COMMAND_
{
    int     USER_COMMAND;
    char    USER_PARA_CHAR[MAX_COMMAND_DATA];
    int     USER_PARA_INT[MAX_COMMAND_DATA];
    float   USER_PARA_FLOAT[MAX_COMMAND_DATA];
    double  USER_PARA_DOUBLE[MAX_COMMAND_DATA];

} USER_COMMAND, *pUSER_COMMAND;

// Upboard to GUI
typedef struct _ROBOT_STATE_DATA_ {
    float        custom_variable[20];
} ROBOT_STATE_DATA, *pROBOT_STATE_DATA;

///-------------------------------------------------------------------------


typedef struct _RBCORE_SHM_
{
    ROBOT_STATE_DATA ROBOT_DATA;
    _USER_COMMAND_  COMMAND;
    bool NEWCOMMAND;
    bool LanComm_Status;

    bool rmd_motor_run_flag[12];
    //    int rmd_reference_velocity[12];
    //    int rmd_reference_position[12];
    int rmd_reference_parameter[12];//added
    int rmd_status_temperature[12];
    int rmd_status_torque[12];
    int rmd_status_velocity[12];
    int rmd_status_position[12];
    int COMMAND_NUM;

    int rmd_response[8];

}RBCORE_SHM, *pRBCORE_SHM;

#endif // SHARED_MEMORY_H
