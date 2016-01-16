#include "my_serial.h"

/*
 * Serial port
 */
#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */


//#include <nuttx/config.h>
//#include <nuttx/sched.h>


#define PORT_TTYS           "/dev/ttyS6"
#define BAUD                (115200)
#define READ_BUFFER_SIZE    (40)


//static int  fd_serial       =   ERROR;       /* File descriptor for the port */
//static char read_buf[READ_BUFFER_SIZE];

/*
 * 'open_port()' - Open serial port 1.
 *
 * Returns the file descriptor on success or -1 on error.
 */
//int
//open_port(void)
//{
//	/** Flags: read+write / required by POSIX / don't wait for data */
//	//fd_serial = open(PORT_TTYS, O_RDWR);
//	fd_serial = open(PORT_TTYS, O_RDWR | O_NOCTTY | O_NDELAY);
//
//	if (fd_serial == ERROR) {
//		/*
//		 * Could not open the port.
//		 */
//		warnx("open_port: Unable to open /dev/ttyS6");
//	} else {
//		if(configure_port() != ERROR) {
//			warnx("open_port: Unable to config /dev/ttyS6");
//		}
//	}
//	return (fd_serial);
//}


//int
//configure_port(void)
//{
//	struct termios uart_config;
//	int termios_state;
//
//	/* fill the struct for the new configuration */
//	tcgetattr(fd_serial, &uart_config);
//
//	/* clear ONLCR flag (which appends a CR for every LF) */
//	uart_config.c_oflag &= ~ONLCR;
//	/* no parity, one stop bit */
//	uart_config.c_cflag &= ~(CSTOPB | PARENB);
//
//
//
//	/* set baud rate */
//	if ((termios_state = cfsetispeed(&uart_config, BAUD)) < 0) {
//		warnx("ERR: %d (cfsetispeed)\n", termios_state);
//		return ERROR;
//	}
//
//	if ((termios_state = cfsetospeed(&uart_config, BAUD)) < 0) {
//		warnx("ERR: %d (cfsetospeed)\n", termios_state);
//		return ERROR;
//	}
//
//	if ((termios_state = tcsetattr(fd_serial, TCSANOW, &uart_config)) < 0) {
//		warnx("ERR: %d (tcsetattr)\n", termios_state);
//		return ERROR;
//	}
//
//
//	return 0;
//}


//void
//write_port(char* payload, int length)
//{
//	/** Send serial buffer */
//	int n = write(fd_serial, payload, length);
//	if (n < 0)
//		printf("Failed to send");
//}

//
//void
//flush_port(void)
//{
//	/** Flush written, but not send data. */
//	int n = tcflush(fd_serial, TCOFLUSH);
//	if (n < 0)
//		printf("Failed to flush");
//}
//
//
//void
//read_port(char* buffer, int length)
//{
//	/** Copy data into external buffer */
//	int n = read(fd_serial, buffer, length);
//	if (n < 0)
//		printf("Failed to receive");
//}



//int
//close_port(void)
//{
//	return (close(fd_serial));
//}
