#ifndef _uart_931_h_
#define _uart_931_h_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <assert.h>
#include <termios.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include <sys/types.h>
#include <errno.h>

int uart_open(int fd, const char *pathname);
int uart_set(int fd, int nSpeed, int nBits, char nEvent, int nStop);
int uart_close(int fd);
int send_data(int fd, char *send_buffer, int length);
int recv_data(int fd, char *recv_buffer, int length);
int parse_data(char chr);
void get_data(double *acc, double *gyro, double *quat);

#ifdef __cplusplus
}
#endif

#endif
