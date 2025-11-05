#ifndef SPI2CAN_H
#define SPI2CAN_H

#include <linux/spi/spidev.h>

#include "rt_utils.h"
#include "rmd_motor.h"
#include "shared_memory.h"

#define MAX_CAN_PER_MSG     7
#define MAX_BYTE_PER_MSG    84

typedef union{
    ST_CAN  can_msg[MAX_CAN_PER_MSG];
    unsigned char data[MAX_BYTE_PER_MSG];
}U_MSG;

class spi2can
{
    spi2can();

    int spi_1_fd;
    int spi_2_fd;

    int init_spi();

public:
    static spi2can * getInstance(){
        static spi2can obj;
        return &obj;
    }

private:

    int         thread_id;
    pthread_t   thread_handler;
    static void *spi2can_thread(void *arg);
};

#endif // SPI2CAN_H
