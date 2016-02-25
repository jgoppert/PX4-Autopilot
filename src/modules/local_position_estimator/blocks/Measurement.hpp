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

#include <matrix/math.hpp>
#include <controllib/blocks.hpp>
#include <controllib/block/BlockParam.hpp>
#include "../constants.hpp"

using namespace control;
using namespace matrix;

template<class T, size_t N>
class Measurement : public control::SuperBlock {
public:
	/**
	 * Constructor
	 * @timeout Time in seconds for timeout.
	 */
	Measurement(SuperBlock * parent, const char * name, float timeOut) :
		SuperBlock(parent, name),
		_fault(FAULT_NONE),
		_timeOut(timeOut),
		_initialized(false),
		_timeStamp(hrt_absolute_time()),
		_initCount(0)
	{
	}

	/**
	 * Perform a measurement
	 */
    Vector<T, N> measure() {};

	/**
	 * Responsible for initializint measurement
	 */
	virtual void init() {};

	/**
	 * Kalman filter based correction
	 */
	virtual void correct() {};

	/**
	 * Calculate if timed out
	 */
	bool timedOut(uint64_t now) {
		return (now - _timeStamp) > _timeOut*1.0e6f;
	}
	virtual ~Measurement() {};

private:
	fault_t _fault;
	float _timeOut;
	bool _initialized;
	uint64_t _timeStamp;
	uint32_t _initCount;
    Vector<T, N> _y0;
};

