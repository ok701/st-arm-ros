#ifndef RMD_CAN_H
#define RMD_CAN_H

#include <vector>
#include <QVector>
#include <QByteArray>
#include "rmd_motor.h"

typedef struct{
    unsigned char header;
    unsigned char dlc;
    unsigned short id;
    unsigned char data[8];
}ST_CAN;


class rmd_can
{
public:
    rmd_can();

    static pthread_mutex_t mutex_reference[12];

    static ST_CAN  reference_msg[12];   // pos-ref, cur-ref, gain
    static ST_CAN  encoder_msg[12];     //

    static QVector<ST_CAN> general_send_msgs1;
    static QVector<ST_CAN> general_send_msgs2;
    static QVector<ST_CAN> general_recv_msgs;

    static void set_reference_msg(int bno, unsigned char data[8]);
    static void write_general_msg(ST_CAN mb);
};

#endif // RB_CAN_H
