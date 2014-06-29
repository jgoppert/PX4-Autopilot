/****************************************************************************
 *
 *   Copyright (c) 2014 PX4 Development Team. All rights reserved.
 *   Author: James Goppert, Scott Yantek
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

#include <controllib/uorb/blocks.hpp>

using namespace control;

class BlockFlappingController : public control::BlockUorbEnabledAutopilot {
public:
	BlockFlappingController() :
		BlockUorbEnabledAutopilot(NULL,"FL"),
		th2v(this, "TH2V"),
		q2v(this, "Q2V"),
		_servoTravel(this, "SRV_TRV"),
		_wingUp(this, "WNG_UP"),
		_wingDown(this, "WNG_DWN"),
		_wingGlide(this, "WNG_GLD"),
		_tDown2Up(this, "T_DWN2UP"),
		_tUp2Glide(this, "T_UP2GLD"),
		_throttleGlide(this, "THR_GLD"),
		_throttle2Frequency(this, "THR2FREQ"),
		_minFrequency(this, "MIN_FREQ"),
		_attPoll(),
		_timeStamp(0)
	{
		_attPoll.fd = _att.getHandle();
		_attPoll.events = POLLIN;
	}
	void update();
private:
	enum {CH_LEFT, CH_RIGHT};
	BlockPI th2v;
	BlockP q2v;
	BlockParamFloat _servoTravel;
	BlockParamFloat _wingUp;
	BlockParamFloat _wingDown;
	BlockParamFloat _wingGlide;
	BlockParamFloat _tDown2Up;
	BlockParamFloat _tUp2Glide;
	BlockParamFloat _throttleGlide;
	BlockParamFloat _throttle2Frequency;
	BlockParamFloat _minFrequency;
	struct pollfd _attPoll;
	uint64_t _timeStamp;
	uint64_t _cycleStartTimeStamp;
	void cycleFrequencyFunction(float throttle, float & cycleFrequency);
	void flappingFunction(float t, float aileron,
			float elevator, float throttle,
			float & wingLeft, float & wingRight);
};
