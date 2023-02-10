#include "uart_931.h"

int uart_open(int fd, const char *pathname)
{
  fd = open(pathname, O_RDWR | O_NOCTTY);
  if (-1 == fd)
  {
    perror("Can't open serial port");
    return (-1);
  }
  else
    printf("Open %s successful\n", pathname);
  if (isatty(STDIN_FILENO) == 0)
    printf("Standard input is not a terminal device\n");
  return fd;
}

int uart_set(int fd, int nSpeed, int nBits, char nEvent, int nStop)
{
  struct termios newtio, oldtio;
  if (tcgetattr(fd, &oldtio) != 0)
  {
    perror("Setup serial 1");
    printf("tcgetattr(fd, &oldtio) -> %d\n", tcgetattr(fd, &oldtio));
    return -1;
  }
  bzero(&newtio, sizeof(newtio));
  newtio.c_cflag |= CLOCAL | CREAD;
  newtio.c_cflag &= ~CSIZE;
  switch (nBits)
  {
  case 7:
    newtio.c_cflag |= CS7;
    break;
  case 8:
    newtio.c_cflag |= CS8;
    break;
  }
  switch (nEvent)
  {
  case 'o':
  case 'O':
    newtio.c_cflag |= PARENB;
    newtio.c_cflag |= PARODD;
    newtio.c_iflag |= (INPCK | ISTRIP);
    break;
  case 'e':
  case 'E':
    newtio.c_iflag |= (INPCK | ISTRIP);
    newtio.c_cflag |= PARENB;
    newtio.c_cflag &= ~PARODD;
    break;
  case 'n':
  case 'N':
    newtio.c_cflag &= ~PARENB;
    break;
  default:
    break;
  }

  switch (nSpeed)
  {
  case 2400:
    cfsetispeed(&newtio, B2400);
    cfsetospeed(&newtio, B2400);
    break;
  case 4800:
    cfsetispeed(&newtio, B4800);
    cfsetospeed(&newtio, B4800);
    break;
  case 9600:
    cfsetispeed(&newtio, B9600);
    cfsetospeed(&newtio, B9600);
    break;
  case 115200:
    cfsetispeed(&newtio, B115200);
    cfsetospeed(&newtio, B115200);
    break;
  case 460800:
    cfsetispeed(&newtio, B460800);
    cfsetospeed(&newtio, B460800);
    break;
  case 921600:
    cfsetispeed(&newtio, B921600);
    cfsetospeed(&newtio, B921600);
    break;
  default:
    cfsetispeed(&newtio, B9600);
    cfsetospeed(&newtio, B9600);
    break;
  }
  if (nStop == 1)
    newtio.c_cflag &= ~CSTOPB;
  else if (nStop == 2)
    newtio.c_cflag |= CSTOPB;
  newtio.c_cc[VTIME] = 0;
  newtio.c_cc[VMIN] = 0;
  tcflush(fd, TCIFLUSH);

  if ((tcsetattr(fd, TCSANOW, &newtio)) != 0)
  {
    perror("Failed to set com port");
    return -1;
  }
  printf("Port baud %d\n", nSpeed);
  return 0;
}

int uart_close(int fd)
{
  assert(fd);
  close(fd);

  return 0;
}

int send_data(int fd, char *send_buffer, int length)
{
  length = write(fd, send_buffer, length * sizeof(unsigned char));
  return length;
}

int recv_data(int fd, char *recv_buffer, int length)
{
  length = read(fd, recv_buffer, length);
  return length;
}

static double a[3], w[3], angle[3], h[3], q[4];

int parse_data(char chr)
{
  static char chrBuf[100];
  static unsigned char chrCnt = 0;
  signed short sData[4];
  unsigned char i;

  time_t now;
  chrBuf[chrCnt++] = chr;
  if (chrCnt < 11)
    return -1;

  if ((chrBuf[0] != 0x55) || ((chrBuf[1] & 0x50) != 0x50))
  {
    // printf("Error: %x %x\r\n", chrBuf[0], chrBuf[1]);
    memcpy(&chrBuf[0], &chrBuf[1], 10);
    chrCnt--;
    return -1;
  }

  int index = 0;

  memcpy(&sData[0], &chrBuf[2], 8);
  switch (chrBuf[1])
  {
  case 0x51:
    index = 1;
    for (i = 0; i < 3; i++)
      a[i] = (double)sData[i] / 32768.0 * 16.0 * 9.81;
    break;
  case 0x52:
    index = 2;
    for (i = 0; i < 3; i++)
      w[i] = (double)sData[i] / 32768.0 * 2000.0 / 57.296;
    break;
  case 0x53:
    index = 3;
    for (i = 0; i < 3; i++)
      angle[i] = (double)sData[i] / 32768.0 * 180.0 / 57.296;
    break;
  case 0x54:
    index = 4;
    for (i = 0; i < 3; i++)
      h[i] = (double)sData[i];
    break;
  case 0x59:
    index = 9;
    for (i = 0; i < 3; i++)
      q[i] = (double)sData[i] / 32768.0;
    break;
  }
  chrCnt = 0;
#if 0
  time(&now);
  printf("\r\nT: %s", asctime(localtime(&now)));
  printf("a: %6.3f %6.3f %6.3f ", a[0], a[1], a[2]);
  printf("w: %7.3f %7.3f %7.3f ", w[0], w[1], w[2]);
  printf("A: %7.3f %7.3f %7.3f ", angle[0], angle[1], angle[2]);
  printf("h: %4.0f %4.0f %4.0f\r\n", h[0], h[1], h[2]);
#endif
  return index;
}

void get_data(double *acc, double *gyro, double *quat)
{
  for (int i = 0; i < 3; i++)
  {
    acc[i] = a[i];
    gyro[i] = w[i];
    quat[i] = q[i];
  }
  quat[3] = q[3];
}

int main_demo(void)
{
  int ret;
  int fd;

  fd = uart_open(fd, "/dev/ttyUSB0"); /*串口号/dev/ttySn,USB口号/dev/ttyUSBn */
  if (fd == -1)
  {
    fprintf(stderr, "uart_open error\n");
    exit(EXIT_FAILURE);
  }

  if (uart_set(fd, 921600, 8, 'N', 1) == -1)
  {
    fprintf(stderr, "uart set failed!\n");
    exit(EXIT_FAILURE);
  }

  char r_buf[1024];
  bzero(r_buf, 1024);

  while (1)
  {
    ret = recv_data(fd, r_buf, 44);
    if (ret == -1)
    {
      fprintf(stderr, "uart read failed!\n");
      exit(EXIT_FAILURE);
    }
    for (int i = 0; i < ret; i++)
    {
      parse_data(r_buf[i]);
    }
    usleep(1000);
  }

  ret = uart_close(fd);
  if (ret == -1)
  {
    fprintf(stderr, "uart_close error\n");
    exit(EXIT_FAILURE);
  }

  exit(EXIT_SUCCESS);
}
