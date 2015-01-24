/****************************************************************************
 *
 *   Copyright (c) 2012, 2013 PX4 Development Team. All rights reserved.
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
 * @file flow_position_estimator_main.cpp
 * Flow position estimator.
 *
 * @author James Goppert <james.goppert@gmail.com>
 */

#include "FlowPositionEstimator.hpp"

#include <nuttx/config.h>
#include <nuttx/sched.h>
#include <unistd.h>
#include <stdio.h>

#include <systemlib/systemlib.h>
#include <systemlib/err.h>

static bool thread_should_exit = false;		/**< flow_position_estimator exit flag */
static bool thread_running = false;		/**< flow_position_estimator status flag */
static int flow_position_estimator_task;				/**< Handle of flow_position_estimator task / thread */

/**
 * flow_position_estimator management function.
 */
extern "C" __EXPORT int flow_position_estimator_main(int argc, char *argv[]);

/**
 * Mainloop of flow_position_estimator.
 */
int flow_position_estimator_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

static void
usage(const char *reason)
{
	if (reason) {
		warnx("%s\n", reason);
	}

	errx(1, "usage: flow_position_estimator {start|stop|status} [-p <additional params>]\n\n");
}

/**
 * The flow_position_estimator app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_create().
 */
int flow_position_estimator_main(int argc, char *argv[])
{
	if (argc < 1) {
		usage("missing command");
	}

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			warnx("flow_position_estimator already running\n");
			/* this is not an error */
			exit(0);
		}

		thread_should_exit = false;
		flow_position_estimator_task = task_spawn_cmd("flow_position_estimator",
					       SCHED_DEFAULT,
					       SCHED_PRIORITY_MAX - 5,
					       4000,
					       flow_position_estimator_thread_main,
					       (argv) ? (char *const *)&argv[2] : (char *const *)NULL);
		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			warnx("\trunning\n");

		} else {
			warnx("\tnot started\n");
		}

		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}

int flow_position_estimator_thread_main(int argc, char *argv[])
{

	warnx("[flow_position_estimator] starting\n");

	FlowPositionEstimator flowEst;

	thread_running = true;

	while (!thread_should_exit) {
		flowEst.update();
	}

	warnx("[flow_position_estimator] exiting.\n");

	thread_running = false;

	return 0;
}
