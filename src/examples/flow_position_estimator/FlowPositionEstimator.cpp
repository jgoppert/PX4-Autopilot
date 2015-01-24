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
 * @file FlowPositionEstimator.cpp
 * Flow Position Estimator
 *
 * @author James Goppert <james.goppert@gmail.com>
 */

#include "FlowPositionEstimator.hpp"
#include <mathlib/mathlib.h>



FlowPositionEstimator::FlowPositionEstimator() :
	SuperBlock(NULL, "FPE"),
	// subscriptions
	_paramUpdate(&getSubscriptions(), ORB_ID(parameter_update),
		     1000), // limit to 1 Hz
	_flow(&getSubscriptions(), ORB_ID(optical_flow), 20),
	_att(&getSubscriptions(), ORB_ID(vehicle_attitude), 20),
	_home(&getSubscriptions(), ORB_ID(home_position), 20),
	// publications
	_pos(&getPublications(), ORB_ID(vehicle_local_position)),
	_posGlobal(&getPublications(), ORB_ID(vehicle_global_position)),
	// time stamp
	_timeStamp(0),
	_mapRef()
{
	_flowPoll.fd = _flow.getHandle();
	_flowPoll.events = POLLIN;
}

void FlowPositionEstimator::update()
{
	// wait for a sensor update, check for exit condition every 100 ms
	if (poll(&_flowPoll, 1, 100) < 0) { return; } // poll error

	uint64_t newTimeStamp = hrt_absolute_time();
	float dt = (newTimeStamp - _timeStamp) / 1.0e6f;
	_timeStamp = newTimeStamp;

	// check for sane values of dt
	// to prevent large control responses
	if (dt > 1.0f || dt < 0) { return; }

	// set dt for all child blocks
	setDt(dt);

	// see what new data exists
	bool paramUpdate = _paramUpdate.updated();
	bool homeUpdate = _home.updated();

	// get new information from subscriptions
	updateSubscriptions();

	if (paramUpdate) { updateParams(); }

	if (homeUpdate) {
		map_projection_init(&_mapRef, _home.lat, _home.lon);
	}

	// angular rotation in x, y axis
	float x_rad = _flow.pixel_flow_x_integral;
	float y_rad = _flow.pixel_flow_y_integral;

	// XXX does flow account for pitch?
	float d = _flow.ground_distance_m;

	// compute velocities in camera frame using ground distance
	// assume camera frame is body frame
	//float dt_flow = _flow.integration_timespan/1.0e6;
	math::Vector<3> v_b;
	v_b(0) = x_rad * d;
	v_b(1) = y_rad * d;
	v_b(2) = 0;

	math::Matrix<3, 3> R_nb(_att.R);
	math::Vector<3> v_n = R_nb * v_b;

	// XXX should handle flow quality

	// local position
	_pos.timestamp = _timeStamp;
	_pos.xy_valid = true;
	_pos.z_valid = true;
	_pos.v_xy_valid = true;
	// could take numer. deriv to est v_z, but would be noisy
	_pos.v_z_valid = false;
	// using euler integration for x, y
	_pos.x = _pos.x + v_n(0) + _home.x;
	_pos.y = _pos.y + v_n(1) + _home.y;
	_pos.z = -d + _home.z;
	_pos.vx = v_n(0);
	_pos.vy = v_n(1);
	_pos.vz = v_n(2);
	_pos.yaw = _att.yaw;
	_pos.xy_global = true;
	_pos.z_global = true;
	_pos.ref_lat = _home.lat;
	_pos.ref_lon = _home.lon;
	_pos.ref_alt = _home.alt;
	_pos.dist_bottom = d;
	_pos.dist_bottom_rate = 0;
	_pos.surface_bottom_timestamp = _timeStamp;
	_pos.dist_bottom_valid = true;
	_pos.eph = 0.001;
	_pos.epv = 0.001;

	// global position
	double proj_lat, proj_lon;
	map_projection_reproject(&_mapRef, _pos.x, _pos.y,
				 &proj_lat, &proj_lon);
	_posGlobal.timestamp = _timeStamp;
	_posGlobal.time_utc_usec = _timeStamp;
	_posGlobal.lat = proj_lat;
	_posGlobal.lon = proj_lon;
	_posGlobal.alt = _pos.ref_alt - _pos.z;
	_posGlobal.vel_n = _pos.vx;
	_posGlobal.vel_e = _pos.vy;
	_posGlobal.vel_d = _pos.vz;
	_posGlobal.yaw = _pos.yaw;
	_posGlobal.eph = _pos.eph;
	_posGlobal.epv = _pos.epv;
	_posGlobal.terrain_alt = 0;
	_posGlobal.terrain_alt_valid = false;

	// update all publications
	updatePublications();
}

FlowPositionEstimator::~FlowPositionEstimator()
{
}
