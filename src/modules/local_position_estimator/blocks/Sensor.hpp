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

#include <matrix/matrix/Matrix.hpp>
#include <matrix/matrix/filter.hpp>
#include <controllib/blocks.hpp>
#include <controllib/block/BlockParam.hpp>
#include "../constants.hpp"

using namespace control;
using namespace matrix;

template<class Type, size_t M, size_t N>
class Sensor : public control::SuperBlock
{
private:
	fault_t _fault;
	uint32_t _initCount;
	bool _initialized;
	uint64_t _timeStamp;
	float _timeOut;
	uint32_t _initPeriod;
	uint32_t _expectedFreq;
	Vector<Type, N> _y0;
public:

	//---------------------------------------------
	// User must define these functions
	//---------------------------------------------

	/**
	 * Perform a measurement
	 * @y - measurement
	 * @return - valid measurement returns RET_OK
	 */
	virtual int measure(Vector<Type, N> &y) = 0;

	/**
	 * Returns is there is an update waiting
	 * ser must define
	 * @return - true if update waiting
	 */
	virtual bool updateAvailable() = 0;

	/**
	 * Compute correction data
	 * Input:
	 * @x - state
	 * @y - measurement
	 * Output:
	 * @C - measurement matrix
	 * @R - measurement noise covariance matrix
	 * @r - residual
	 */
	virtual int computeCorrectionData(
		const Vector<Type, M> &x,
		const Vector<Type, N> &y,
		Matrix<Type, N, M> &C,
		Matrix<Type, N, N> &R,
		Vector<Type, N> &r) = 0;

	//---------------------------------------------
	// Other functions
	//---------------------------------------------

	/**
	 * Constructor
	 * @timeout Time in seconds for timeout.
	 */
	Sensor(SuperBlock *parent, const char *name, float timeOut,
	       float initPeriod, float expectedFreq) :
		SuperBlock(parent, name),
		_fault(FAULT_NONE),
		_initCount(0),
		_initialized(false),
		_timeStamp(hrt_absolute_time()),
		_timeOut(timeOut),
		_initPeriod(initPeriod),
		_expectedFreq(expectedFreq),
		_y0()
	{
	}

	/**
	 * Responsible for initializing measurement
	 */
	virtual int init()
	{
		if (_initialized) { return RET_ERROR; }

		Vector<Type, N> y;
		int ret = measure(y);

		if (ret == RET_OK) {
			_y0 += y;

		} else {
			_initCount = 0;
			y.setZero();
		}

		if (_initCount > _initPeriod * _expectedFreq) {
			_y0 = _y0 / _initCount;
			_initialized = true;
		}

		return RET_OK;
	}

	/**
	 * Calculate if timed out
	 */
	bool timedOut()
	{
		return (hrt_absolute_time() - _timeStamp) > _timeOut * 1.0e6f;
	}

	virtual int correct(Vector<float, M> &x, Matrix<float, M, M> &P)
	{
		Vector<float, N> y;
		int ret = measure(y);

		if (ret != RET_OK) { return ret; }

		Matrix<float, N, M> C;
		Matrix<float, N, N> R;
		float beta = 0;
		Vector<float, N> r;
		ret = computeCorrectionData(x, y, C, R, r);

		if (ret != RET_OK) { return ret; }

		Vector<float, M> dx;
		Matrix<float, M, M> dP;
		ret = kalman_correct(P, C, R, r, dx, dP, beta);

		if (ret != RET_OK) { return ret; }

		x += dx;
		P += dP;
		return RET_OK;
	}

	/**
	 * Main user interface
	 * @return - returns RET_OK if ok
	 */
	virtual int update(Vector<float, M> &x, Matrix<float, M, M> &P)
	{
		if (updateAvailable()) {
			if (!_initialized) {
				return init();

			} else  {
				return correct(x, P);
			}
		}
		return RET_OK;
	}

	virtual ~Sensor() {};
};

