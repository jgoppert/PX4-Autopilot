/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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

#pragma once

enum fault_t {
	FAULT_NONE = 0,
	FAULT_MINOR,
	FAULT_SEVERE
};

enum {
	RET_OK = 0,
	RET_ERROR
};

enum sensor_t {
	SENSOR_BARO = 0,
	SENSOR_GPS,
	SENSOR_LIDAR,
	SENSOR_FLOW,
	SENSOR_SONAR,
	SENSOR_VISION,
	SENSOR_MOCAP
};

// constants
enum {X_x = 0, X_y, X_z, X_vx, X_vy, X_vz, X_bx, X_by, X_bz, n_x};
enum {U_ax = 0, U_ay, U_az, n_u};
enum {Y_baro_z = 0, n_y_baro};
enum {Y_lidar_z = 0, n_y_lidar};
enum {Y_flow_x = 0, Y_flow_y, n_y_flow};
enum {Y_sonar_z = 0, n_y_sonar};
enum {Y_gps_x = 0, Y_gps_y, Y_gps_z, Y_gps_vx, Y_gps_vy, Y_gps_vz, n_y_gps};
enum {Y_vision_x = 0, Y_vision_y, Y_vision_z, n_y_vision};
enum {Y_mocap_x = 0, Y_mocap_y, Y_mocap_z, n_y_mocap};
enum {poll_flow, poll_sensors, poll_param};
