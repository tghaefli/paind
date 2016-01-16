//#include <drivers/drv_hrt.h>

//Default includes
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <px4_config.h>
#include <nuttx/sched.h>
#include <systemlib/systemlib.h>
#include <systemlib/err.h>

#include <px4_time.h>


// For uORB and topics
#include <nuttx/config.h>
#include <poll.h>
#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>


#include "serial.h"
#include "my_app.h"
#include <systemlib/systemlib.h>
#include <uORB/uORB.h>
#include <drivers/drv_gyro.h>
#include <drivers/drv_accel.h>
#include <fcntl.h>
#include <sched.h>



static bool thread_should_exit = false;		/**< daemon exit flag */
static bool thread_running = false;		    /**< daemon status flag */
static int daemon_task;				        /**< Handle of daemon task / thread */

#define deamon_task_stack_size 2000


/**
 * daemon management function.
 */
extern "C" {
    __EXPORT int my_app_main(int argc, char *argv[]);
}

/*
 *  Prototypes
 */

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

static void
usage(const char *reason)
{
    if (reason)
    {
        warnx("%s\n", reason);
    }

    warnx("usage: daemon {start|stop|status} [-p <additional params>]\n\n");
}

/**
 * The daemon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_create().
 */
int my_app_main(int argc, char *argv[])
{
    if (argc < 2)
    {
        usage("missing command");
        return 1;
    }

    if (!strcmp(argv[1], "start"))
    {
        if (thread_running)
        {
            warnx("daemon already running\n");
            /* this is not an error */
            return 0;
        }
        thread_should_exit = false;
        daemon_task = px4_task_spawn_cmd("my_app_daemon",
                                         SCHED_DEFAULT,
                                         SCHED_PRIORITY_DEFAULT,
                                         deamon_task_stack_size,
                                         daemon_thread_main,
                                         (argv) ? (char *const *)&argv[2] : (char *const *)NULL);
        return 0;
    }
    if (!strcmp(argv[1], "stop"))
    {
        thread_should_exit = true;
        return 0;
    }
    if (!strcmp(argv[1], "status"))
    {
        if (thread_running)
        {
            warnx("\trunning\n");
        }
        else
        {
            warnx("\tnot started\n");
        }
        return 0;
    }
    usage("unrecognized command");
    return 1;
}


int daemon_thread_main(int argc, char *argv[])
{
    warnx("starting\n");
    thread_running = true;

    int sensor_combined_sub = orb_subscribe(ORB_ID(sensor_combined));

    //serialOpen();
    usleep(100);
    //Serial::init("/dev/ttyS2",921600);
    usleep(100);

    while (!thread_should_exit)
    {



        /* obtained data for the first file descriptor */
        struct sensor_combined_s raw;
        /* copy sensors raw data into local buffer */

        orb_copy(ORB_ID(sensor_combined), sensor_combined_sub, &raw);

        //hrt_abstime curr_time = hrt_absolute_time();
        //printf("%" PRIu64 "\n",curr_time);

        printf("TEST TEST TEST\n");

        printf("Accelerometer:\t\t%8.4f\t%8.4f\t%8.4f\n",
               (double)raw.accelerometer_m_s2[0],
               (double)raw.accelerometer_m_s2[1],
               (double)raw.accelerometer_m_s2[2]);

        printf("Gyro:\t\t\t%8.4f\t%8.4f\t%8.4f\n",
               (double)raw.gyro_rad_s[0],
               (double)raw.gyro_rad_s[1],
               (double)raw.gyro_rad_s[2]);

        printf("Magnetometer:\t\t%8.4f\t%8.4f\t%8.4f\n",
               (double)raw.magnetometer_ga[0],
               (double)raw.magnetometer_ga[1],
               (double)raw.magnetometer_ga[2]);

        printf("baro_pres_mbar:\t\t%8.4f\n",
               (double)raw.baro_pres_mbar[0]);

        printf("baro_alt_meter:\t\t%8.4f\n",
               (double)raw.baro_alt_meter[0]);

        printf("baro_temp_celcius:\t%8.4f\n\n",
               (double)raw.baro_temp_celcius[0]);


        printf("differential_pressure_pa:\t%8.4f\n",
               (double)raw.differential_pressure_pa[0]);

        printf("differential_pressure_filtered_pa:\t%8.4f\n\n",
               (double)raw.differential_pressure_filtered_pa[0]);

//        int i = 8;
//        outBuffer[i++]='s';
//        memcpy(&outBuffer[i],&time,sizeof(time));
//        i=i+sizeof(time); //i=8
//        out.putData(outBuffer,i);


        //Wait for 1s
        sleep(1);
    }
    warnx("exiting.\n");
    thread_running = false;

    return 0;
}

