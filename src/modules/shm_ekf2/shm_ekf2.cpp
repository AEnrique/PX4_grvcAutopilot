/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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
 * @file shm_ekf2.cpp
 * Application to send topcis data through shared memory
 *
 * @author Antonio Enrique Jiménez Cano <antenr@us.es>
 */

#include <px4_posix.h>
#include <poll.h>

#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_global_position.h>
//#include <uORB/topics/att_control.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <px4_config.h>
#include <px4_tasks.h>

#include <time.h>
#include <uORB/uORB.h>
#include <drivers/drv_hrt.h>

//#include <systemlib/systemlib.h>
#include <systemlib/err.h>
//----------------

#include "/home/pi/Repos/BridgeNavio/libshm_navio/navio_types.h"
#include "/home/pi/Repos/BridgeNavio/libshm_navio/shm_channels.h"
#include "/home/pi/Repos/BridgeNavio/libshm_navio/shm_util.h"

static struct shm_str<shm_attitudePX4EKF2> att_px4ekf2;
static struct shm_str<shm_localPositionPX4EKF2> localposition_px4ekf2;
static struct shm_str<shm_globalPositionPX4EKF2> globalposition_px4ekf2;

static bool thread_should_exit = false;		/**< daemon exit flag */
static bool thread_running = false;		/**< daemon status flag */
static int daemon_task;				/**< Handle of daemon task / thread */
//static struct param_handles ph;

/**
 * management function.
 */
extern "C" __EXPORT int shm_ekf2_main(int argc, char *argv[]);

/**
 * Mainloop of daemon.
 */
int shm_ekf2_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

//void shm_die(char *s);

static void usage(const char *reason)
{
	if (reason) {
		warnx("%s\n", reason);
	}

	warnx("usage: daemon {start|stop|status} [-p <additional params>]\n\n");
}

/*void shm_die(char *s)
{
    	perror(s);
    	exit(1);
}*/

/**
 *
 */
int shm_ekf2_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage("missing command");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			warnx("daemon already running\n");
			/* this is not an error */
			return 0;
		}

		thread_should_exit = false;
		daemon_task = px4_task_spawn_cmd("daemon",
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_DEFAULT,
						 2000,
						 shm_ekf2_thread_main,
						 (argv) ? (char *const *)&argv[2] : (char *const *)NULL);
		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			warnx("\trunning\n");

		} else {
			warnx("\tnot started\n");
		}

		return 0;
	}

	usage("unrecognized command");
	return 1;
}

int shm_ekf2_thread_main(int argc, char *argv[])
{

	warnx("[daemon] starting\n");
	create_shm<shm_attitudePX4EKF2>(&att_px4ekf2,SHM_ATTPX4,SEM_ATTPX4);
	create_shm<shm_localPositionPX4EKF2>(&localposition_px4ekf2,SHM_LOCALPOSPX4,SEM_LOCALPOSPX4);
	create_shm<shm_globalPositionPX4EKF2>(&globalposition_px4ekf2,SHM_GLOBALPOSPX4,SEM_GLOBALPOSPX4);

	shm_localPositionPX4EKF2 local_pos;
	shm_globalPositionPX4EKF2 global_pos;
	shm_attitudePX4EKF2 att;

	//attitudeValues msgATT;
	//global_pos_values msgGLOBALPOS;
	//local_pos_values msgLOCALPOS;

  // -------------------
	/* subscribe to vehicle_attitude topic */
	int vehicle_attitude_sub_fd = orb_subscribe(ORB_ID(vehicle_attitude));
	//int att_control_sub_fd = orb_subscribe(ORB_ID(att_control));
	/* subscribe to vehicle_local_position topic */
	int vehicle_local_position_sub_fd = orb_subscribe(ORB_ID(vehicle_local_position));
	/* subscribe to vehicle_global_position topic */
	int vehicle_global_position_sub_fd = orb_subscribe(ORB_ID(vehicle_global_position));


  /********************************************************************
   ********************************************************************
   *************erase next line to eliminate the limit rate************
   ********************************************************************
   ********************************************************************/
  //orb_set_interval(vehicle_attitude_sub_fd, 200);

  /* one could wait for multiple topics with this technique, just using one here */
  px4_pollfd_struct_t fds[] = {
    { .fd = vehicle_attitude_sub_fd,   .events = POLLIN },
	{ .fd = vehicle_local_position_sub_fd,   .events = POLLIN },
	{ .fd = vehicle_global_position_sub_fd,   .events = POLLIN },
        /* there could be more file descriptors here, in the form like:
         * { .fd = other_sub_fd,   .events = POLLIN },
         */
  };



  int error_counter = 0;

	thread_running = true;

	while (!thread_should_exit) {
    /* wait for sensor update of 2 file descriptors for 4 ms (250Hz) */
    int poll_ret = px4_poll(fds, 3, 4);

    /* handle the poll result */
    if (poll_ret == 0) {
        /* this means none of our providers is giving us data */
        //PX4_ERR("Got no data within a second");

    } else if (poll_ret < 0) {
        /* this is seriously bad - should be an emergency */
        if (error_counter < 10 || error_counter % 50 == 0) {
            /* use a counter to prevent flooding (and slowing us down) */
            PX4_ERR("ERROR return value from poll(): %d", poll_ret);
        }

        error_counter++;


    } else {
				// fds[1] to give priority to att_control update
        if ((fds[0].revents || fds[1].revents || fds[2].revents ) & POLLIN) {

            /* obtained data for the first file descriptor */
            struct vehicle_attitude_s v_attitude;
			//struct att_control_s att_control;
			struct vehicle_local_position_s v_local_pos;
			struct vehicle_global_position_s v_global_pos;

            /* copy sensors raw data into local buffer */
            orb_copy(ORB_ID(vehicle_attitude), vehicle_attitude_sub_fd, &v_attitude);
			//orb_copy(ORB_ID(att_control), att_control_sub_fd, &att_control);

            att._rollspeed = v_attitude.rollspeed;
            att._pitchspeed = v_attitude.pitchspeed;
            att._yawspeed = v_attitude.yawspeed;

            for(int k=0;k<4;k++)
            	att._q[k] = v_attitude.q[k];

            if (v_attitude.timestamp > 0){
            	att._check=1;
            }else{
            	att._check=0;
            }
			if (set_shm<shm_attitudePX4EKF2>(&att_px4ekf2, att) == 0)
						printf("shm_attitudePX4EKF2:Error to write in shared memory direction.\n");

			/* */
			orb_copy(ORB_ID(vehicle_local_position), vehicle_local_position_sub_fd, &v_local_pos);

			local_pos._x = v_local_pos.x;
			local_pos._y = v_local_pos.y;
			local_pos._z = v_local_pos.z;
			local_pos._vx = v_local_pos.vx;
			local_pos._vy = v_local_pos.vy;
			local_pos._vz = v_local_pos.vz;
			local_pos._yaw = v_local_pos.yaw;
			local_pos._ax = v_local_pos.ax;
			local_pos._ay = v_local_pos.ay;
			local_pos._az = v_local_pos.az;
            //if (v_local_pos.timestamp > 0){
            //	local_pos._check=1;
            //}else{
            //	local_pos._check=0;
            //}
			local_pos._check = v_local_pos.xy_valid;
			if (set_shm<shm_localPositionPX4EKF2>(&localposition_px4ekf2, local_pos) == 0)
						printf("shm_localPositionPX4EKF2:Error to write in shared memory direction.\n");


            /* */
			orb_copy(ORB_ID(vehicle_global_position), vehicle_global_position_sub_fd, &v_global_pos);

			global_pos._lat = v_global_pos.lat;
			global_pos._lon = v_global_pos.lon;
			global_pos._alt = v_global_pos.alt;
            global_pos._vel_n = v_global_pos.vel_n;
            global_pos._vel_e = v_global_pos.vel_e;
            global_pos._vel_d = v_global_pos.vel_d;
            global_pos._yaw=v_global_pos.yaw;
            global_pos._eph = v_global_pos.eph;
            global_pos._epv = v_global_pos.epv;
            if (v_global_pos.timestamp > 0){
            	global_pos._check=1;
            }else{
            	global_pos._check=0;
            }
			if (set_shm<shm_globalPositionPX4EKF2>(&globalposition_px4ekf2, global_pos) == 0)
						printf("shm_globalPositionPX4EKF2:Error to write in shared memory direction.\n");





        }
    }
	}

	warnx("[daemon] exiting.\n");
	if (close_shm(&att_px4ekf2) == -1){
		printf("att_px4ekf2:Error closing shared memory\n");
	}
	if (close_shm(&localposition_px4ekf2) == -1){
		printf("att_px4ekf2:Error closing shared memory\n");
	}
	if (close_shm(&globalposition_px4ekf2) == -1){
		printf("att_px4ekf2:Error closing shared memory\n");
	}
	thread_running = false;

	return 0;
}
