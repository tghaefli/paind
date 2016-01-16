//Default includes
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <crc32.h>


#include "my_app.h"
#include "crc.h"


/*
 * Defines
 */
#define deamon_task_stack_size  (4096)  // could be smaller, but set higher to reduce potential nasty problem
#define BUFFER_SIZE_SEND        (1024)
#define BUFFER_SIZE_RCV         (1024)
#define BUFFER_SIZE_CRC         (2)     // 4 bytes - 32 bit - crc32

#define TIMEOUT_SEND_POLL_MS    (200)  // max waiting time for poll function
#define TIMEOUT_RCV_POLL_MS     (200)  // max waiting time for poll function
#define TIMEOUT_RCV_US          (20e3)  // wait after successful poll for more data
#define TIMEOUT_TASK_US         (50e3)  // 50 ms wait

#define PORT_TTYS               "/dev/ttyS6"
#define BAUD                    (460800)            //115200   460800

#define SEND_DATA               1
#define RCV_DATA                0


/*
 * Global Variables
 */
static bool thread_should_exit = false;		/** daemon exit flag **/
static bool thread_running =     false;     /** daemon status flag **/
static bool data_rcv_good =      false;
static int daemon_task =         0;	        /** Handle of daemon task/thread **/

static int fd_serial =           ERROR;

static uint16_t crc_calc =       0;
static uint16_t length_payload = 0;


static char buf_send[BUFFER_SIZE_SEND];
static char buf_rcv[BUFFER_SIZE_RCV];


typedef enum {
	SYNC1 = 1,
	SYNC2,
	SYNC3,
	LENGTH_1,
	LENGTH_2,
	PAYLOAD,
	CHKSUM_GET_1,
	CHKSUM_GET_2,
	CHKSUM_CALC,
} state_t;


/*
 *
 * uORB <--> Serial converter
 *
 */
int
serial_task_main(int argc, char *argv[])
{
	warnx("Thread starting\n");
	thread_running = true;

	int error_counter = 0;
	int poll_ret = 0;
	uint16_t i = 0;

	fd_serial = open(PORT_TTYS, O_RDWR);

	/** open Serial port **/
	if (fd_serial < 0) {
		warnx("Failed to open serial port, fd = %i", fd_serial);
		thread_should_exit = true;
	} else {
		warnx("Serial port opened successful, fd = %i", fd_serial);
	}

	/** configure Serial port **/
	if (uart_config() == ERROR) {
		warnx("Serial port configure failed successful");
		thread_should_exit = true;
	} else {
		warnx("Serial port configured successful");
	}

	warnx("Creating file descriptors");
#if SEND_DATA == 1
	/* poll-file-descriptor for uORB topic(s) */
	struct sensor_combined_s raw;
	int sensor_sub_fd = orb_subscribe(ORB_ID(sensor_combined));
	struct pollfd fds_uorb[]= {
		{ .fd = sensor_sub_fd,   .events = POLLIN },
	};
#endif // SEND_DATA

#if RCV_DATA
	/* poll-file-descriptor for serial interface */
	struct pollfd fds_serial[]= {
		{.fd = fd_serial,   .events = POLLIN },
	};
#endif // RCV_DATA
	warnx("File descriptors created");


	/** For debug purpose **/
	memcpy(buf_rcv,  "a", 1);
	memcpy(buf_send, "a", 1);


	while (!thread_should_exit) {

#if SEND_DATA == 1
		{

			/*
			 *
			 * Get uORB messages and send them to serial
			 *
			 */
			/* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
			poll_ret = poll(fds_uorb, 1, TIMEOUT_SEND_POLL_MS);

			/* handle the poll result */
			if (poll_ret == 0)
			{
				/* this means none of our providers is giving us data */
				warnx("Got no data within %i milli seconds\n", TIMEOUT_SEND_POLL_MS);
			} else if (poll_ret < 0)
			{
				/* this is seriously bad - should be an emergency */
				if (error_counter < 10 || error_counter % 50 == 0) {
					/* use a counter to prevent flooding (and slowing us down) */
					warnx("ERROR return value from poll(): %d\n"
					, poll_ret);
				}
				error_counter++;
			} else {
				if (fds_uorb[0].revents & POLLIN)
				{
					i = 0;

					/* copy sensor raw data into local buffer */
					orb_copy(ORB_ID(sensor_combined), sensor_sub_fd, &raw);

					/** Add Parser item **/
					char parser[3] = "#:_";
					memcpy(&buf_send[i], &parser, SYNC3);
					i += SYNC3;


                    /** Add length **/
                    uint16_t tmp = sizeof(raw);
                    memcpy(&buf_send[i], &tmp, sizeof(tmp));
					i += sizeof(i);
//					warnx("Payload length is %i",sizeof(raw));


                    /** Add payload **/
                    memcpy(&buf_send[i], &raw, sizeof(raw));
                    i += sizeof(raw);
//                  warnx("g1 is %2.4f\n", (double)raw.accelerometer_m_s2[0]);
//                  warnx("g2 is %2.4f\n", (double)raw.accelerometer_m_s2[1]);
//                  warnx("g3 is %2.4f\n", (double)raw.accelerometer_m_s2[2]);


					/** Add checksum **/
					crc_calc = gen_crc16((uint8_t*)buf_send + 5*sizeof(char) , (i - SYNC3 - 2));// crc of payload, sync or length is not included
					memcpy(&buf_send[i], &crc_calc, BUFFER_SIZE_CRC);
					i += BUFFER_SIZE_CRC;


					/** print crc for debug **/
					char tmp2[2];
					sprintf(tmp2,"%d",crc_calc);
//					warnx("crc is %s \n", tmp2);


					/** Send data over serial **/
					write(fd_serial, buf_send, i);
//					warnx("Sending data!\n\n");
				}
			}
		}
#endif // SEND_DATA

#if RCV_DATA == 1
		{
			/*
			 *
			 * Get serial messages and send them to uORB
			 *
			 */
			/* poll for new data */
			poll_ret = poll(fds_serial, 1, TIMEOUT_RCV_POLL_MS);


			/* handle the poll result */
			if (poll_ret == 0)
			{
				/* this means none of our providers is giving us data */
				warnx("Got no data within %i milli seconds\n", TIMEOUT_RCV_POLL_MS);
			} else if (poll_ret < 0)
			{
				/* this is seriously bad - should be an emergency */
				if (error_counter < 10 || error_counter % 50 == 0) {
					/* use a counter to prevent flooding (and slowing us down) */
					warnx("ERROR return value from poll(): %d\n", poll_ret);
				}
				error_counter++;

			} else if (poll_ret > 0)
			{
				/* if we have new data from GPS, go handle it */
				if (fds_serial[0].revents & POLLIN) {
					//warnx("Waiting for Data\n");
					usleep(TIMEOUT_RCV_US);

					char buf_tmp[BUFFER_SIZE_RCV*2];
					/*
					 * We are here because poll says there is some data, so this
					 * won't block even on a blocking device.  If more bytes are
					 * available, we'll go back to poll() again...
					 */

					int ctr = read(fd_serial, buf_tmp, BUFFER_SIZE_RCV);    //only saved data on newline

					for (i = 0; i < ctr; i++) {
						parse_buf(buf_tmp[i]);
					}
					//warnx("Data received");

					if(data_rcv_good) {
                        data_rcv_good = 0;
						warnx("Good data received, writing to uORB");

                        // .....

					}
				}
			}
		}
#endif // RCV_DATA

		// Slow down the whole app
		usleep(TIMEOUT_TASK_US);

	}
	warnx("Thread: exiting.\n");
	thread_running = false;

	close(fd_serial);
	return 0;
}


/**
 * The daemon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_create().
 */
int
my_app_main(int argc, char *argv[])
{
	if (argc < 2) {
		warnx("missing command");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {
		return threads_start(argc, argv);
	}

	if (!strcmp(argv[1], "stop")) {
		return threads_stop();
	}

	if (!strcmp(argv[1], "status")) {
		return threads_status();
	}

	if (!strcmp(argv[1], "help") || !strcmp(argv[1], "--help") || !strcmp(argv[1], "-h")) {
		return threads_help();
	}

	// Wrong input detected
	warnx("my_app: unrecognized command, try --help\n");
	return 1;
}


int
threads_start(int argc, char *argv[])
{
	if (thread_running) {
		warnx("thread is already running\n");
		/* this is not an error */
	} else {
		thread_should_exit = false;
		daemon_task = px4_task_spawn_cmd(
						  "hil_simulation",
						  SCHED_DEFAULT,
						  SCHED_PRIORITY_DEFAULT,
						  deamon_task_stack_size,
						  serial_task_main,
						  (argv) ? (char *const *)&argv[2] : (char *const *)NULL);
	}
	return 0;
}


int
threads_status(void)
{
	if (thread_running) {
		warnx("\tthread is running\n");
	}  else  {
		warnx("\tthread not started\n");
	}
	return 0;
}


int
threads_help(void)
{
	warnx("options:\n");
	printf("\t start: Start the application\n");
	printf("\t stop: Stop the application\n");
	printf("\t status: Request current application status\n");
	return 0;
}


int
threads_stop(void)
{
	thread_should_exit = true;  /** Set static variable */
	warnx("stop");
	return 0;
}


int
uart_config(void)
{
	struct termios uart_config;

	int termios_state;

	/* fill the struct for the new configuration */
	tcgetattr(fd_serial, &uart_config);

	/* clear ONLCR flag (which appends a CR for every LF) */
	uart_config.c_oflag &= ~ONLCR;
	/* no parity, one stop bit */
	uart_config.c_cflag &= ~(CSTOPB | PARENB);

	/* set baud rate */
	if ((termios_state = cfsetispeed(&uart_config, BAUD)) < 0) {
		warnx("ERR: %d (cfsetispeed)\n", termios_state);
		return ERROR;
	}

	if ((termios_state = cfsetospeed(&uart_config, BAUD)) < 0) {
		warnx("ERR: %d (cfsetospeed)\n", termios_state);
		return ERROR;
	}

	if ((termios_state = tcsetattr(fd_serial, TCSANOW, &uart_config)) < 0) {
		warnx("ERR: %d (tcsetattr)\n", termios_state);
		return ERROR;
	}
	return 0;
}

void
parse_buf(const uint8_t b)
{
	static state_t state;
	static int ctr_payload;
	static uint32_t crc_sent;

//	warnx("char is %i", b);

	switch (state) {
	case SYNC1:
//		warnx("SYNC 1");

		if(b == 0x23)           // #
			state = SYNC2;
		break;


	case SYNC2:
//		warnx("SYNC 2");
		if(b == 0x3A)           // :
			state = SYNC3;
		else
			state = SYNC1;
		break;


	case SYNC3:
//		warnx("SYNC 3");
		if(b == 0x5F){          // _
            ctr_payload = 0;    // reset values
            crc_sent    = 0;
            crc_calc    = 0;
            state = LENGTH_1;

		}
		else
			state = SYNC1;
		break;


	case LENGTH_1:
//		warnx("LENGTH_1");
		length_payload += 256*(uint8_t)b;
		state = LENGTH_2;
		break;


	case LENGTH_2:
//		warnx("LENGTH_2");
		length_payload += (uint8_t)b;
//		warnx("length is %i", length_payload);
		state = PAYLOAD;
		break;



	case PAYLOAD:
		if(ctr_payload < length_payload-1) {
			buf_rcv[ctr_payload] = b;
//          warnx("payload char is %c", buf_rcv[ctr_payload]);
            ctr_payload++;
		} else if (ctr_payload == length_payload-1) {           //change state without losing a byte
            buf_rcv[ctr_payload] = b;
//          warnx("payload char is %c", buf_rcv[ctr_payload]);
			state = CHKSUM_GET_1;
		}
		break;


	case CHKSUM_GET_1:
        crc_sent += ((uint8_t)b)<<8;
        state = CHKSUM_GET_2;
        break;


    case CHKSUM_GET_2:
        crc_sent += (uint8_t)b;
        state = CHKSUM_CALC;
//      warnx("crc value is %i", (uint8_t)b);
        break;


	case CHKSUM_CALC:
		/** checksum is calculated for everything payload only **/
		crc_calc = gen_crc16((uint8_t*)buf_rcv , length_payload);

//		warnx("crc sent is %d", crc_sent);
//		warnx("crc calc is %d", crc_calc);

		if(crc_sent == crc_calc) {              //checksum of payload
			warnx("Data received is ok");
			data_rcv_good = 1;
		} else {
			warnx("Data received is bad");
		}
		state = SYNC1;
		break;


	default:
	    state = SYNC1;
		break;
	}
}


