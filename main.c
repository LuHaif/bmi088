#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <stdlib.h>
#include <time.h>
#include "src/spi.h"
#include "src/driver.h"

int main(){

  SPI_Open();
  printf("init rst %d \n",BMI088_init());
  float gyro[3], accel[3], temp;
  struct timespec time;
  struct timespec end_time;
  while(1){
    clock_gettime(0,&time);
    // printf("timespec %d.%d\n",time.tv_sec, time.tv_nsec);
    BMI088_read(gyro, accel, &temp);
    printf("accel  %f  %f  %f\n", accel[0],accel[1],accel[2]);
    printf("gyro  %f  %f  %f\n", gyro[0],gyro[1],gyro[2]);
    printf("temp %f \n", temp);
    clock_gettime(0,&end_time);
    printf("read operation used %f ms\n", ((end_time.tv_sec + end_time.tv_nsec/1e9) - (time.tv_sec + time.tv_nsec/1e9))*1000);
    usleep(10);
    // sleep(1);
  }
}
