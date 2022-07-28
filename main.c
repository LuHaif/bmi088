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
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
int main(){

  SPI_Open();
  int rst = BMI088_init();
  printf("init rst %d \n",rst);
  // if(rst)
  //   return rst;
  float gyro[3], accel[3], temp;
  struct timespec time;
  struct timespec end_time;
  int sock_fd;
  struct sockaddr_in server_addr;
  char recv_buf[1000];
  // char *str;
  int nbytes = 0;
  socklen_t len = 0;

  /* 创建Socket */
  sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (sock_fd < 0)
  {
    printf("客户端Socket创建失败");
    return -1;
  }

  /* 绑定ip和端口 */
  bzero(&server_addr, sizeof(server_addr));
  server_addr.sin_family = AF_INET;
  // server_addr.sin_addr.s_addr = inet_addr(argv[1]);
  server_addr.sin_port = htons(atoi("43897"));//指定端口号
  int optval = 1;
  setsockopt(sock_fd, SOL_SOCKET, SO_BROADCAST, &optval, sizeof(optval));
  inet_pton(AF_INET, "192.168.1.255", &server_addr.sin_addr.s_addr);
  char *str = recv_buf;
  len = sizeof(server_addr);
  float test = 0.0;
  sleep(2);
  while(1){
    clock_gettime(0,&time);
    // printf("timespec %d.%d\n",time.tv_sec, time.tv_nsec);
    BMI088_read(gyro, accel, &temp);
    sprintf(recv_buf, "$%f %f %f %f %f %f;", gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2]);
        // printf("%d \n", strlen(recv_buf));
    sendto(sock_fd,recv_buf,strlen(recv_buf),0,(struct sockaddr *)(&server_addr),len);
    // printf("accel  %+3.6f  %+3.6f  %+3.6f\n", accel[0],accel[1],accel[2]);
    // usleep(1000);
    // printf("gyro  %f  %f  %f\n", gyro[0],gyro[1],gyro[2]);
    // printf("temp %f \n", temp);
    // clock_gettime(0,&end_time);
    // printf("read operation used %f ms\n", ((end_time.tv_sec + end_time.tv_nsec/1e9) - (time.tv_sec + time.tv_nsec/1e9))*1000);
    // usleep(10);
    // usleep(1000000);
    // usleep(50000);
  }
}
