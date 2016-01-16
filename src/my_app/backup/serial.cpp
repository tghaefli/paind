/****************************************************************************
 *
 *   Copyright (c) 2012-2014 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * Author: Stephan Zuercher stephan.zuercher@stud.hslu.ch
 */

#include <fcntl.h>
#include <termios.h>
#include <systemlib/err.h>
#include <errno.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>

#include <poll.h> /*Nuttix spez include*/

#include "serial.h"

Serial::Serial()
{
    ;
}
int Serial::init(const char *serialPortdevname, int speed)
{
    devName=serialPortdevname;
    serialOpen();
    serialSetSpeed(speed);
    serialClose();

    return 0;
}

int
Serial::serialOpen(bool write)
{
    if(fd!=-1)
        ::close(fd);
    if(write)
        fd = ::open(devName,O_RDWR | O_NOCTTY | O_TRUNC);
    else
        fd = ::open(devName,O_RDONLY | O_NOCTTY | O_TRUNC);
    if(fd==-1)
    {
        warnx("Faild to open Serial port: %s",devName);
        return -1;
    }
    return fd;

}
void
Serial::serialClose()
{
    ::close(fd);
}


int
Serial::serialSetSpeed(int speed)
{
    /* Try to set baud rate */
    struct termios uart_config;
    int termios_state;

    /* Back up the original uart configuration to restore it after exit */
    if ((termios_state = tcgetattr(fd, &uart_config)) < 0)
    {
        warnx("ERR GET CONF %s: %d\n", devName, termios_state);
        close(fd);
        return -1;
    }

    /* Clear ONLCR flag (which appends a CR for every LF) */
    uart_config.c_oflag &= ~ONLCR;

    /* USB serial is indicated by /dev/ttyACM0*/
    if (strcmp(devName, "/dev/ttyACM0") != OK && strcmp(devName, "/dev/ttyACM1") != OK)
    {
        /* Set baud rate */
        if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0)
        {
            warnx("ERR SET BAUD %s: %d\n", devName, termios_state);
            close(fd);
            return -1;
        }
    }

    if ((termios_state = tcsetattr(fd, TCSANOW, &uart_config)) < 0)
    {
        warnx("ERR SET CONF %s\n", devName);
        close(fd);
        return -1;
    }

    warnx("Unknown error in function 'serialSetSpeed'");
    return -1;   //Required to supress warning
}

int
Serial::getData(uint8_t *recv, unsigned len)
{
    ssize_t bytesRead;
    unsigned bytesTotalRead;

    uint8_t* dataPtr = recv;

    bytesTotalRead=0;

    do
    {
        bytesRead=::read(fd, (dataPtr+bytesTotalRead), len-bytesTotalRead);
        if(bytesRead==-1)
        {
            printf("Error reading Serial data. ErrorNo: %d\n",errno);
            exit(-1);
        }
        bytesTotalRead+=bytesRead;
    }
    while(bytesTotalRead<len);
    if(bytesTotalRead!=len)
    {
        printf("Error reading Serial data. Size didnt match");
        exit(-1);
    }

    return (int)bytesRead;
}

void
Serial::printHex(unsigned len)
{
    uint8_t data[len];
    getData(data,len);
    printf("\n reading %d bites\n",len);
    for(unsigned i=0; i<len; i++)
        //printf("%02d: %02x\n",i,data[i]);
        printf("%02x ",data[i]);
}
void
Serial::printAscii(unsigned len)
{
    uint8_t data[len];
    getData(data,len);
    for(unsigned i=0; i<len; i++)
    {
        if(data[i]=='\r')
            data[i]='\n';
        printf("%c",data[i]);
    }
}

void
Serial::sync2CR_LF(void)
{
    ssize_t numBytes;
    unsigned numBytesRead;
    uint8_t buffer[2];

    numBytesRead=0;

    while(numBytesRead<SYNC_TIMEOUT_CNT)
    {
        numBytes = getData(&buffer[1], 1);
        if(numBytes==-1)
        {
            warnx("Error reading Serial data");
            exit(-1);
        }

        numBytesRead+=numBytes;
        if(numBytes==1)
        {
            if( buffer[0]==0x0d && buffer[1]==0x0a)  //found CR _ LF
            {
                //printf("found sync after %lu bits\n",numBytesRead);
                return;
            }
        }
        else
        {
            warnx("Error reading while syncing");
            exit(-1);
        }

        buffer[0]=buffer[1];

    }

    warnx("Error no CR LF sync found");
    exit(-1);


}
int
Serial::putData(uint8_t *trans, unsigned len)
{
    ::write(fd,trans,len);
    return len;
}
int
Serial::isDataAvailable()
{
    pollfd fds[1];
    fds[0].fd = fd;
    fds[0].events = POLLIN;

    int ret = ::poll(fds, sizeof(fds) / sizeof(fds[0]), 500); //timeout 500ms
    if (ret < 0)
        exit(-2);
    return ret;
}
void
Serial::sync()
{
    //not inplemented yet in nuttx
    //::ioctl(fd,TCFLSH,TCIOFLUSH));
}
