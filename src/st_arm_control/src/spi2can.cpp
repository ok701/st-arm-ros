#include "spi2can.h"
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <string.h>
#include <vector>

#define RMD_COUNT 3

extern pRBCORE_SHM sharedData;
extern rmd_motor _DEV_MC[RMD_COUNT];

#define SPI_SPEED 4000000
unsigned char spi_mode = SPI_MODE_0;
unsigned char spi_bits_per_word = 8;
unsigned char spi_speed = SPI_SPEED;
unsigned char lsb = 0x01;

using namespace std;
spi2can::spi2can()
{
#ifdef EXTERNAL
    spi_1_fd = -1;
    spi_2_fd = -1;

    if(init_spi() != 0){
        cout << "init spi failed" << endl;
    }
    else cout << "init spi succeed" << endl;

    thread_id = generate_rt_thread_hard(thread_handler, spi2can_thread, "spi2can", 2, 98, this);
    cout << "spi2can_thread id: " << thread_id << endl;
#endif
}


void *spi2can::spi2can_thread(void *arg){
    const long PERIOD_US = RT_MS * 1000;
    // const long PERIOD_US = 4 * 1000;
    struct timespec TIME_NEXT;
    struct timespec TIME_NOW;
    unsigned long dead_miss_cnt = 0;

    unsigned int tt = 0;
    spi2can *spi = (spi2can*)arg;


    struct spi_ioc_transfer spi_tr1;
    struct spi_ioc_transfer spi_tr2;
    memset(&spi_tr1, 0, sizeof(struct spi_ioc_transfer));
    memset(&spi_tr2, 0, sizeof(struct spi_ioc_transfer));


    unsigned char tx_1[MAX_BYTE_PER_MSG] = {0,};
    unsigned char tx_2[MAX_BYTE_PER_MSG] = {0,};
    unsigned char rx_1[MAX_BYTE_PER_MSG] = {0,};
    unsigned char rx_2[MAX_BYTE_PER_MSG] = {0,};


    spi_tr1.tx_buf = (unsigned long)tx_1;
    spi_tr1.rx_buf = (unsigned long)rx_1;
    spi_tr1.len = MAX_BYTE_PER_MSG;
    spi_tr1.speed_hz = SPI_SPEED;
    spi_tr1.delay_usecs = 0;
    spi_tr1.bits_per_word = spi_bits_per_word;
    spi_tr1.cs_change = 1;

    spi_tr2.tx_buf = (unsigned long)tx_2;
    spi_tr2.rx_buf = (unsigned long)rx_2;
    spi_tr2.len = MAX_BYTE_PER_MSG;
    spi_tr2.speed_hz = SPI_SPEED;
    spi_tr2.delay_usecs = 0;
    spi_tr2.bits_per_word = spi_bits_per_word;
    spi_tr2.cs_change = 1;

    QByteArray recv_buf1;
    QByteArray recv_buf2;

    int count[12] = {0,};

    usleep(500*1000);

    clock_gettime(CLOCK_REALTIME, &TIME_NEXT);
    while(true){
        for(int j=0; j<8; j++){
            rmd_can::reference_msg[0].data[j] = _DEV_MC[0].ref_data[j];
            rmd_can::reference_msg[1].data[j] = _DEV_MC[0].ref_data2[j];
            rmd_can::reference_msg[2].data[j] = _DEV_MC[1].ref_data[j];
            rmd_can::reference_msg[3].data[j] = _DEV_MC[1].ref_data2[j];
            rmd_can::reference_msg[4].data[j] = _DEV_MC[2].ref_data[j];
            rmd_can::reference_msg[5].data[j] = _DEV_MC[2].ref_data2[j];
        }

        for(int i=0; i<6; i++){
            if(sharedData->rmd_motor_run_flag[i]){
                pthread_mutex_lock(&rmd_can::mutex_reference[i]);
                memcpy(&(tx_1[i*12]), &(rmd_can::reference_msg[i]), 12);
                pthread_mutex_unlock(&rmd_can::mutex_reference[i]);
            }
        }

        for(int j=0; j<8; j++){
            rmd_can::reference_msg[6].data[j] = _DEV_MC[2].ref_data[j];
            rmd_can::reference_msg[7].data[j] = _DEV_MC[2].ref_data2[j];
        }

        for(int i=0; i<6; i++){
            if(sharedData->rmd_motor_run_flag[i+6]){
                pthread_mutex_lock(&rmd_can::mutex_reference[i+6]);
                memcpy(&(tx_2[i*12]), &(rmd_can::reference_msg[i+6]), 12);
                pthread_mutex_unlock(&rmd_can::mutex_reference[i+6]);
            }
        }

        int rv1 = ioctl(spi->spi_1_fd, SPI_IOC_MESSAGE(1), &spi_tr1);
        int rv2 = ioctl(spi->spi_2_fd, SPI_IOC_MESSAGE(1), &spi_tr2);
        
        recv_buf1.append((const char*)rx_1, MAX_BYTE_PER_MSG);
        recv_buf2.append((const char*)rx_2, MAX_BYTE_PER_MSG);

        // HRRLAB - SPI1(CAN CH A, B or 0, 1), CAN Recieve status
        while(recv_buf1.size() >= 12){
            if(uchar(recv_buf1[0]) == 0x89){
                int dlc = recv_buf1[1];
                unsigned int id = ((ushort)(recv_buf1[2]) | (ushort)(recv_buf1[3]<<8));
                unsigned char recv_data1[8];
                for(int i=0; i<8; i++) recv_data1[i] = recv_buf1[4+i];
                recv_buf1.remove(0, 12);

                if(id>=0x140 && id<=0x240+RMD_COUNT){
                    int bno = 0;
                    if(id>=0x240) bno = id-0x240;
                    else bno = id-0x140;
                    _DEV_MC[bno].count++;
                    if(recv_data1[0] == 0xA1){ 
                        for(int j=0; j<dlc; j++) _DEV_MC[bno].torque_data[j] = recv_data1[j];
                        _DEV_MC[bno].UpdateRxData();
                        _DEV_MC[bno].count_A1++;
                    }
                    else if(recv_data1[0] == 0x92){
                        for(int j=0; j<dlc; j++) _DEV_MC[bno].enc_data[j] = recv_data1[j];
                        _DEV_MC[bno].count_92++;
                    } 
                    else
                    {
                        _DEV_MC[bno].unknown_value = (int)recv_data1[0];
                    }
                }
            }else recv_buf1.remove(0, 1);
        }

        // HRRLAB - SPI2(CAN CH C, D or 2, 3), CAN Recieve status
        while(recv_buf2.size() >= 12){
            if(uchar(recv_buf2[0]) == 0x89){
                int dlc = recv_buf2[1];
                unsigned int id = ((ushort)(recv_buf2[2]) | (ushort)(recv_buf2[3]<<8));
                unsigned char recv_data2[8];
                for(int i=0; i<8; i++) recv_data2[i] = recv_buf2[4+i];                
                recv_buf2.remove(0, 12);

                if(id>=0x140 && id<=0x240+RMD_COUNT){
                    int bno = 0;
                    if(id>=0x240) bno = id-0x240;
                    else bno = id-0x140;
                    _DEV_MC[bno].count++;
                    if(recv_data2[0] == 0xA1){ 
                        for(int j=0; j<dlc; j++) _DEV_MC[bno].torque_data[j] = recv_data2[j];
                        _DEV_MC[bno].UpdateRxData();
                        _DEV_MC[bno].count_A1++;
                    }
                    else if(recv_data2[0] == 0x92){
                        for(int j=0; j<dlc; j++) _DEV_MC[bno].enc_data[j] = recv_data2[j];
                        _DEV_MC[bno].count_A1++;
                    } 
                }
            }else recv_buf2.remove(0, 1);
        }
        
        clock_gettime(CLOCK_REALTIME, &TIME_NOW);
        timespec_add_us(&TIME_NEXT, PERIOD_US);
        if(timespec_cmp(&TIME_NOW, &TIME_NEXT) > 0) cout << "RT Deadline Miss, SPI " << ++dead_miss_cnt << endl;
        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &TIME_NEXT, NULL);
    }
}


int spi2can::init_spi(){
    int rv = 0;
    spi_1_fd = open("/dev/spidev1.0", O_RDWR);
    if (spi_1_fd < 0) perror("[ERROR] Couldn't open spidev 1.0");
    spi_2_fd = open("/dev/spidev1.1", O_RDWR);
    if (spi_2_fd < 0) perror("[ERROR] Couldn't open spidev 1.1");

    rv = ioctl(spi_1_fd, SPI_IOC_WR_MODE, &spi_mode);
    if (rv < 0) perror("[ERROR] ioctl spi_ioc_wr_mode (1)");

    rv = ioctl(spi_2_fd, SPI_IOC_WR_MODE, &spi_mode);
    if (rv < 0) perror("[ERROR] ioctl spi_ioc_wr_mode (2)");

    rv = ioctl(spi_1_fd, SPI_IOC_RD_MODE, &spi_mode);
    if (rv < 0) perror("[ERROR] ioctl spi_ioc_rd_mode (1)");

    rv = ioctl(spi_2_fd, SPI_IOC_RD_MODE, &spi_mode);
    if (rv < 0) perror("[ERROR] ioctl spi_ioc_rd_mode (2)");

    rv = ioctl(spi_1_fd, SPI_IOC_WR_BITS_PER_WORD, &spi_bits_per_word);
    if (rv < 0) perror("[ERROR] ioctl spi_ioc_wr_bits_per_word (1)");

    rv = ioctl(spi_2_fd, SPI_IOC_WR_BITS_PER_WORD, &spi_bits_per_word);
    if (rv < 0) perror("[ERROR] ioctl spi_ioc_wr_bits_per_word (2)");

    rv = ioctl(spi_1_fd, SPI_IOC_RD_BITS_PER_WORD, &spi_bits_per_word);
    if (rv < 0) perror("[ERROR] ioctl spi_ioc_rd_bits_per_word (1)");

    rv = ioctl(spi_2_fd, SPI_IOC_RD_BITS_PER_WORD, &spi_bits_per_word);
    if (rv < 0) perror("[ERROR] ioctl spi_ioc_rd_bits_per_word (2)");

    rv = ioctl(spi_1_fd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed);
    if (rv < 0) perror("[ERROR] ioctl spi_ioc_wr_max_speed_hz (1)");
    rv = ioctl(spi_2_fd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed);
    if (rv < 0) perror("[ERROR] ioctl spi_ioc_wr_max_speed_hz (2)");

    rv = ioctl(spi_1_fd, SPI_IOC_RD_MAX_SPEED_HZ, &spi_speed);
    if (rv < 0) perror("[ERROR] ioctl spi_ioc_rd_max_speed_hz (1)");
    rv = ioctl(spi_2_fd, SPI_IOC_RD_MAX_SPEED_HZ, &spi_speed);
    if (rv < 0) perror("[ERROR] ioctl spi_ioc_rd_max_speed_hz (2)");

    rv = ioctl(spi_1_fd, SPI_IOC_RD_LSB_FIRST, &lsb);
    if (rv < 0) perror("[ERROR] ioctl spi_ioc_rd_lsb_first (1)");

    rv = ioctl(spi_2_fd, SPI_IOC_RD_LSB_FIRST, &lsb);
    if (rv < 0) perror("[ERROR] ioctl spi_ioc_rd_lsb_first (2)");
    return rv;
}
