#include <iostream>
#include "uart_931.h"
#include <lcm/lcm-cpp.hpp>
#include "lcm_sensor_msgs/Imu.hpp"

static lcm::LCM lc;
static uint32_t seq = 0;

void pub_imu_data(double *a, double *w, double *q)
{
  struct timespec tv;
  lcm_sensor_msgs::Imu msg;
  clock_gettime(CLOCK_MONOTONIC, &tv);
  msg.seq = seq;
  msg.sec = tv.tv_sec;
  msg.nsec = tv.tv_nsec;
  for (uint32_t i = 0; i < 3; i++)
  {
    msg.linear_acceleration[i] = a[i];
    msg.angular_velocity[i] = w[i];
    msg.orientation[i] = q[i];
  }
  msg.orientation[3] = q[3];
  lc.publish("sensor/imu", &msg);
  seq++;
}

int32_t sched_process(int p)
{
  int32_t ret;
  pid_t pid = getpid();
  struct sched_param param;
  param.sched_priority = p;
  ret = sched_setscheduler(pid, SCHED_FIFO, &param);
  if (ret != 0)
  {
    printf("Failed to process sched %d. %s\n", pid, strerror(ret));
  }
  return ret;
}

int main(int argc, char **argv)
{
  sched_process(30);

  if (!lc.good())
  {
    std::cout << "Error: Lcm not good!\n";
    return -1;
  }

  const char *port_name = "/dev/ttyUSB0";
  double baud = 921600;
  int fd;
  fd = uart_open(fd, port_name);
  if (fd == -1)
  {
    printf("Failed to uart open, port %s\n", port_name);
    return -1;
  }
  if (uart_set(fd, baud, 8, 'N', 1) == -1)
  {
    printf("Failed to uart set\n");
    return -1;
  }

  int ret = 0;
  char r_buf[1024];
  bzero(r_buf, 1024);

  int num_pack = 0;
  double a[3], w[3], q[4];

  while (1)
  {
    ret = recv_data(fd, r_buf, 44);
    if (ret == -1)
    {
      printf("Failed to uart read\n");
      uart_close(fd);
      return -2;
    }
    for (int i = 0; i < ret; i++)
    {
      if (parse_data(r_buf[i]) == 9)
      {
        get_data(a, w, q);
        pub_imu_data(a, w, q);
      }
    }
    usleep(20);
  }

  ret = uart_close(fd);
  if (ret == -1)
  {
    printf("\nFailed to uart close\n");
    return -1;
  }
  else
  {
    printf("\nClose port\n");
  }

  return 0;
}
