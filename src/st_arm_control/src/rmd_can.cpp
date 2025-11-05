#include "rmd_can.h"

static rmd_can obj;

extern rmd_motor _DEV_MC[3];

pthread_mutex_t rmd_can::mutex_reference[12];
ST_CAN  rmd_can::reference_msg[12];
ST_CAN  rmd_can::encoder_msg[12];

rmd_can::rmd_can()
{
    // spi1
    reference_msg[0].header = 0x89;     reference_msg[0].id = 0x140;
    reference_msg[1].header = 0x89;     reference_msg[1].id = 0x140;
    
    reference_msg[2].header = 0x89;     reference_msg[2].id = 0x141;
    reference_msg[3].header = 0x89;     reference_msg[3].id = 0x141;
    
    reference_msg[4].header = 0x89;     reference_msg[4].id = 0x142;
    reference_msg[5].header = 0x89;     reference_msg[5].id = 0x142;
    // spi2
    reference_msg[6].header = 0x89;     reference_msg[6].id = 0x142;
    reference_msg[7].header = 0x89;     reference_msg[7].id = 0x142;
    
    reference_msg[8].header = 0x77;     reference_msg[8].id = 0x142;
    reference_msg[9].header = 0x77;     reference_msg[9].id = 0x142;

    // reference_msg[10].header = 0x77;     reference_msg[10].id = 0x148;
    // reference_msg[11].header = 0x77;     reference_msg[11].id = 0x00;

    for(int i=0; i<12; i++){
        pthread_mutex_init(&mutex_reference[i], NULL);

        reference_msg[i].dlc = 8;
        // reference_msg[i].id = 0x140+i;
        // for(int j=0; j<8; j++) reference_msg[i].data[j] = _DEV_MC[i].ref_data[j];
    }
}

void rmd_can::set_reference_msg(int bno, unsigned char data[8]){
    pthread_mutex_lock(&mutex_reference[bno]);
    for(int i=0; i<8; i++){
        reference_msg[bno].data[i] = data[i];
    }
    pthread_mutex_unlock(&mutex_reference[bno]);
}

void rmd_can::write_general_msg(ST_CAN mb) {
    ST_CAN msg = mb;
    int bno = mb.id - 0x140;
    msg.header = 0x77;
    // general_send_msgs1.push_back(msg);
}
