#ifndef MY_APP_H
#define MY_APP_H


/*
 * Serial port
 */
#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */


/*
 * For uORB and topics
 */
#include <nuttx/config.h>
#include <nuttx/sched.h>
#include <poll.h>
#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>


/*
 *
 */
#include <systemlib/systemlib.h>
#include <systemlib/err.h>


/*
 *
 */
#include <drivers/drv_hrt.h>
#include <px4_time.h>
#include <px4_config.h>


/*
 * Prototypes
 */
int my_app_main(int argc, char *argv[]);
int serial_task_main(int argc, char *argv[]);


int threads_start(int argc, char *argv[]);
int threads_status(void);
int threads_help(void);
int threads_stop(void);

int uart_config(void);

void parse_buf(const uint8_t b);



#endif /* MYAPP_H */
