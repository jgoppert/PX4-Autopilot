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

/**
 * @file Flow.hpp
 * @author James Goppert <james.goppert@gmail.com>
 */

#pragma once
#include "Sensor.hpp"
#include <uORB/topics/optical_flow.h>
#include <uORB/topics/filtered_bottom_flow.h>

class Flow : public Sensor<float, n_x, n_y_flow>
{
public:
	Flow(SuperBlock *parent, const char *name, float timeOut,
	      float initPeriod, float expectedFreq);
	virtual int measure(Vector<float, n_y_flow> &y);
	virtual bool updateAvailable();
	int computeCorrectionData(
		const Vector<float, n_x> &x,
		const Vector<float, n_y_flow> &y,
		Matrix<float, n_y_flow, n_x> &C,
		Matrix<float, n_y_flow, n_y_flow> &R,
		Vector<float, n_y_flow> &r);
	virtual ~Flow() {};
private:
	uORB::Subscription<optical_flow_s> _sub;
	uORB::Publication<filtered_bottom_flow_s> _pub;
	BlockParamFloat  _xy_stddev;
	BlockParamFloat  _board_x_offs;
	BlockParamFloat  _board_y_offs;
	BlockParamFloat  _x_scaler;
	BlockParamFloat  _y_scaler;
	BlockParamInt    _min_q;
	BlockHighPass _gyro_x_high_pass;
	BlockHighPass _gyro_y_high_pass;
};

