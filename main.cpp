#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>

int filter(int new_value, int value, int a);
int num = 0;
typedef struct optical_flow_data
{
    int16_t		flow_x_integral;
    int16_t		flow_y_integral;
    uint16_t   	integration_timespan;
    uint16_t   	ground_distance;
    uint8_t    	quality;
    uint8_t    	version;
}UpixelsOpticalFlow;

UpixelsOpticalFlow updata;

int16_t flow_parse_char(uint8_t ch);

int set_interface_attribs(int fd, int speed)
{
    struct termios tty;

    if (tcgetattr(fd, &tty) < 0) {
        printf("Error from tcgetattr: %s\n", strerror(errno));
        return -1;
    }

    cfsetospeed(&tty, (speed_t)speed);
    cfsetispeed(&tty, (speed_t)speed);

    tty.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;         /* 8-bit characters */
    tty.c_cflag &= ~PARENB;     /* no parity bit */
    tty.c_cflag &= ~CSTOPB;     /* only need 1 stop bit */
    tty.c_cflag &= ~CRTSCTS;    /* no hardware flowcontrol */

    /* setup for non-canonical mode */
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_oflag &= ~OPOST;

    /* fetch bytes as they become available */
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        printf("Error from tcsetattr: %s\n", strerror(errno));
        return -1;
    }
    return 0;
}

void set_mincount(int fd, int mcount)
{
    struct termios tty;

    if (tcgetattr(fd, &tty) < 0) {
        printf("Error tcgetattr: %s\n", strerror(errno));
        return;
    }

    tty.c_cc[VMIN] = mcount ? 1 : 0;
    tty.c_cc[VTIME] = 5;        /* half second timer */

    if (tcsetattr(fd, TCSANOW, &tty) < 0)
        printf("Error tcsetattr: %s\n", strerror(errno));
}


int main()
{
    char *portname = "/dev/ttyUSB0";
    int fd;
    int wlen;

    fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        printf("Error opening %s: %s\n", portname, strerror(errno));
        return -1;
    }
    /*baudrate 115200, 8 bits, no parity, 1 stop bit */
    set_interface_attribs(fd, B115200);
    //set_mincount(fd, 0);                /* set to pure timed read */

    /* simple output */
    /**wlen = write(fd, "Hello!\n", 7);
    if (wlen != 7) {
        printf("Error from write: %d, %d\n", wlen, errno);
    }
    tcdrain(fd);    /* delay for output **/
    int valuex = -999;
    int valuey = -999;
    int bt = 0;
    int ret;
    /* simple noncanonical input */
    do {
        unsigned char buf[1];
        int rdlen;

        rdlen = read(fd, buf, sizeof(buf) - 1);


        if (rdlen > 0) {
           // std::cout<<buf<<std::endl;
            ret = flow_parse_char((uint8_t)buf[1]);
            if (!ret) {
                short x = 0, y = 0;
                time_t t = time(0);
                struct tm tmTmp;
                char sttemp[32];
                //localtime_s(&tmTmp,&t);
                //asctime_s(sttemp, &tmTmp);
                x = updata.flow_x_integral;
                y = updata.flow_y_integral;
                //timespan = updata.integration_timespan;
                if (valuex != -999) {
                    x = filter(x, valuex, 50);
                }
                if (valuey != -999) {
                    y = filter(y, valuey, 50);
                }
                valuex = x;
                valuey = y;
                /*printf_s("x=%4d , y=%4d\n", x, y);
                printf_s("time is: %s\n", sttemp);*/
                //f << x << " " << y << "\n" << "time:" << sttemp << "\n";
                //printf("num=%d \n", num);
                //printf("time=%d \n", timespan);//这个是光流间隔时间
            }
        } else if (rdlen < 0) {
            printf("Error from read: %d: %s\n", rdlen, strerror(errno));
        }
        /* repeat read to get full message */
    } while (1);
}

int16_t flow_parse_char(uint8_t ch)
{
    num++;
    int16_t ret = 1;
    static int16_t s = 0, p = 0;
    static uint8_t Xor_r = 0x00, Xor_c = 0x00;

    switch (s)
    {
        case 0:
            if (ch == 0xFE)
            {
                s = 1;
                //printf("Got FE\n");
            }
            break;
        case 1:
            if (ch == 0x0A)
            {
                s = 2;
                p = 0;
                Xor_c = 0x00;
                //printf("Got 0A\n");
            }
            else
                s = 0;
            break;
        case 2:
            ((char *)&updata)[p++] = ch;
            Xor_c ^= ch;
            if (p == 10) {
                s = 3;
                p = 0;
            }
            break;
        case 3: //crc
            s = 4;
            Xor_r = ch;
            break;
        case 4://end
            if (ch == 0x55) {
                //printf("Got 0x55\n");
                if (Xor_r == Xor_c) {
                    ret = 0;
                }
                else
                    ret = 2;
            }
            s = 0;
            break;
        default:
            break;
    }
    return ret;
}

int filter(int new_value,int value,int a) {
    return ((100 - a)*value + a * new_value)/100;
}