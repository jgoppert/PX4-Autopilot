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
 * @file Vision.cpp
 * @author James Goppert <james.goppert@gmail.com>
 */

#include "Vision.hpp"

Vision::Vision(SuperBlock *parent, const char *name, float timeOut,
	     float initPeriod, float expectedFreq) :
	Sensor<float, n_x, n_y_vision>(parent, name,
				      timeOut, initPeriod, expectedFreq),
	_sub(ORB_ID(vision_position_estimate), 0, 0, &getSubscriptions()),
	_xy_stddev(this, "XY"),
	_z_stddev(this, "Z"),
	_no_vision(NULL, "NO_VISION")
{
}

int Vision::measure(Vector<float, n_y_vision> &y)
{
	y(0) = _sub.get().x;
	y(1) = _sub.get().x;
	y(2) = _sub.get().x;
	return RET_OK;
}

bool Vision::updateAvailable()
{
	return _sub.updated();
}

int Vision::computeCorrectionData(
	const Vector<float, n_x> &x,
	const Vector<float, n_y_vision> &y,
	Matrix<float, n_y_vision, n_x> &C,
	Matrix<float, n_y_vision, n_y_vision> &R,
	Vector<float, n_y_vision> &r)
{
	C.setZero();
	C(Y_vision_x, X_x) = 1;
	C(Y_vision_y, X_y) = 1;
	C(Y_vision_z, X_z) = 1;
	R.setZero();
	R(Y_vision_x, Y_vision_x) = _xy_stddev.get()*_xy_stddev.get();
	R(Y_vision_y, Y_vision_y) = _xy_stddev.get()*_xy_stddev.get();
	R(Y_vision_z, Y_vision_z) = _z_stddev.get()*_z_stddev.get();
	r = y - C * x;
	return RET_OK;
}
