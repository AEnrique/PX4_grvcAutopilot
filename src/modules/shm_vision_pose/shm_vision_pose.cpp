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
 * @file control_correction.c
 * Application to update topic control_correction with data received through UDP
 *
 * @author Manuel J. Fernadez <manfergonz@gmail.com>
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <px4.h>
#include <px4_app.h>

#include <px4_config.h>
#include <px4_tasks.h>

#include <time.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <drivers/drv_hrt.h>

#include <systemlib/systemlib.h>
#include <systemlib/err.h>

#include "/home/pi/Repos/BridgeNavio/libshm_navio/navio_types.h"
#include "/home/pi/Repos/BridgeNavio/libshm_navio/shm_channels.h"
#include "/home/pi/Repos/BridgeNavio/libshm_navio/shm_util.h"


static bool thread_should_exit = false;		/**< daemon exit flag */
static bool thread_running = false;		/**< daemon status flag */
static int daemon_task;				/**< Handle of daemon task / thread */
static int shm_type_input;

static struct shm_str<shm_svo> svo;
static struct shm_str<shm_totalStation> totalstation;
static struct shm_str<shm_vicon> vicon;
//static struct param_handles ph;
using namespace px4;
/**
 * management function.
 */
extern "C" __EXPORT int shm_vision_pose_main(int argc, char *argv[]);

// struct param_handles {
// 	float data; //typedef uint16_t	param_t;
// };

/**
 * Initialize all parameter handles and values
 *
 */
//int att_parameters_init(struct param_handles *h);

/**
 * Mainloop of daemon.
 */
int shm_vision_pose_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);


static void usage(const char *reason)
{
	if (reason) {
		warnx("%s\n", reason);
	}

	warnx("usage: daemon {start {svo|totalstation|vicon}|stop|status} [-p <additional params>]\n\n");
}

/**
 *
 */
int shm_vision_pose_main(int argc, char *argv[])
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

		if (argc == 3){

			if (!strcmp(argv[2], "totalstation")){
				shm_type_input = 2;
			}else{
				if (!strcmp(argv[2], "vicon")){
					shm_type_input = 3;
				}else{
					shm_type_input = 1;
				}
			}
		}else{
			shm_type_input = 1;
		}


		thread_should_exit = false;
		daemon_task = px4_task_spawn_cmd("daemon",
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_DEFAULT,
						 2000,
						 shm_vision_pose_thread_main,
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

int shm_vision_pose_thread_main(int argc, char *argv[])
{
	int hz;

	if (shm_type_input == 2){
		warnx("[daemon] totalstation starting\n");
		create_shm<shm_totalStation>(&totalstation,SHM_TOTALSTATION,SEM_TOTALSTATION);
		hz = 20;

	}else{
		if (shm_type_input == 3){
			warnx("[daemon] vicon starting\n");
			create_shm<shm_vicon>(&vicon,SHM_VICON,SEM_VICON);
			hz = 100;

		}else{

			warnx("[daemon] svo starting\n");
			create_shm<shm_svo>(&svo,SHM_SVO,SEM_SVO);
			hz = 50;
		}

	}


	vehicle_local_position_s ev_pos = {};
	vehicle_attitude_s ev_att = {};

	memset(&ev_pos, 0, sizeof(ev_pos));
	memset(&ev_att, 0, sizeof(ev_att));

	orb_advert_t pub_ev_pos = orb_advertise(ORB_ID(vehicle_vision_position), &ev_pos);
	orb_advert_t pub_ev_att = orb_advertise(ORB_ID(vehicle_vision_attitude), &ev_att);


	thread_running = true;

	shm_totalStation totalstation_msg = {0.0,0.0,0.0};
	shm_svo svo_msg = {0.0,0.0,0.0,0.0,0.0,0.0,1.0};
	shm_vicon vicon_msg = {0.0,0.0,0.0,0.0,0.0,0.0,1.0};

	px4::Rate loop_rate(hz);
		while (!thread_should_exit) {
			loop_rate.sleep();
			if (shm_type_input == 2) {
				memcpy(&totalstation_msg, get_shm<shm_totalStation>(&totalstation),sizeof(shm_totalStation));

				ev_pos.x = totalstation_msg._y;
				ev_pos.y = totalstation_msg._x;
				ev_pos.z = -totalstation_msg._z;
				ev_pos.timestamp = hrt_absolute_time();

				ev_att.q[0] = 0.0;
				ev_att.q[1] = 0.0;
				ev_att.q[2] = 0.0;
				ev_att.q[3] = 1.0;
				ev_att.timestamp = hrt_absolute_time();

			}else{
				if (shm_type_input == 3) {
					memcpy(&vicon_msg, get_shm<shm_vicon>(&vicon),sizeof(shm_vicon));

					ev_pos.x = vicon_msg._x;
					ev_pos.y = vicon_msg._y;
					ev_pos.z = vicon_msg._z;
					ev_pos.timestamp = hrt_absolute_time();

					ev_att.q[0] = vicon_msg._qx;
					ev_att.q[1] = vicon_msg._qy;
					ev_att.q[2] = vicon_msg._qz;
					ev_att.q[3] = vicon_msg._qw;
					ev_att.timestamp = hrt_absolute_time();
				}else{

					memcpy(&svo_msg, get_shm<shm_svo>(&svo),sizeof(shm_svo));

					ev_pos.x = svo_msg._x;
					ev_pos.y = svo_msg._y;
					ev_pos.z = svo_msg._z;
					ev_pos.timestamp = hrt_absolute_time();

					ev_att.q[0] = svo_msg._qx;
					ev_att.q[1] = svo_msg._qy;
					ev_att.q[2] = svo_msg._qz;
					ev_att.q[3] = svo_msg._qw;
					ev_att.timestamp = hrt_absolute_time();
				}
			}



		orb_publish(ORB_ID(vehicle_vision_position), pub_ev_pos, &ev_pos);
		orb_publish(ORB_ID(vehicle_vision_attitude), pub_ev_att, &ev_att);
	}

	warnx("[daemon] exiting.\n");

	if (shm_type_input == 2) {
		if (close_shm(&totalstation) == -1)
		         	    printf("TotalStation:Error closing shared memory.\n");
	}else{
		if (shm_type_input == 3) {
			if (close_shm(&vicon) == -1)
			         	    printf("VICON:Error closing shared memory.\n");
		}else{
			if (close_shm(&svo) == -1)
		         	    printf("SVO:Error closing shared memory.\n");
		}
	}

	thread_running = false;

	return 0;
}
