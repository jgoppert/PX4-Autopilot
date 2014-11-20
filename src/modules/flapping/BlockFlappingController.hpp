/****************************************************************************
 *
 *   Copyright (c) 2014 PX4 Development Team. All rights reserved.
 *   Author: James Goppert, Ben Perseghetti, Scott Yantek
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
#include <uORB/topics/vehicle_vicon_position.h>
#include <uORB/topics/battery_status.h>

#include "learning/GeneticAlgorithm.hpp"

using namespace control;

class BlockFlappingController : public control::BlockUorbEnabledAutopilot {
public:
	BlockFlappingController() :
		BlockUorbEnabledAutopilot(NULL,"FL"),
		_wingFlapState(0),
		_currentlyEvaluating(false),
		_learnStart(0),
		_vicon(&getSubscriptions(), ORB_ID(vehicle_vicon_position), 20),
		_batteryStatus(&getSubscriptions(), ORB_ID(battery_status), 20),
		_servoTravel(this, "SRV_TRV"),
		_wingUp(this, "WNG_UP"),
		_wingDown(this, "WNG_DWN"),
		_wingGlide(this, "WNG_GLD"),
		_tDown2Up(this, "T_DWN2UP"),
		_tUp2Glide(this, "T_UP2GLD"),
		_throttleGlide(this, "THR_GLD"),
		_throttle2Frequency(this, "THR2FREQ"),
		_minFrequency(this, "MIN_FREQ"),
		_lrnTime(this, "LRN_TIME"),
		_mutateProb(this, "MUT_PROB"),
		_reproducingRatio(this, "REP_RATIO"),
		_learnFlapping(this, "LRN_FLAP"),
		_ailMin(this, "AIL_MIN"),
		_ailRange(this, "AIL_RANGE"),
		_elevMin(this, "ELEV_MIN"),
		_elevRange(this, "ELEV_RANGE"),
		_kOmega(this, "K_OMEGA"),
		_wingLeftLowPass(this, "WING_LP"),
		_wingRightLowPass(this, "WING_LP"),
		_powerSum(0),
		_powerCount(0),
		_ga(_populationSize,
			_reproducingRatio.get(),
			_mutateProb.get(),
			_genomeArray,
			_genomeNextArray,
			_fitnessArray),
		_attPoll(),
		_timeStamp(0)
	{
		_attPoll.fd = _att.getHandle();
		_attPoll.events = POLLIN;
	}
	void update();
private:
	float _wingFlapState;
	static const uint16_t _populationSize = 6;
	Genome _genomeArray[_populationSize];
	Genome _genomeNextArray[_populationSize];
	float _fitnessArray[_populationSize];
	bool _currentlyEvaluating;
	uint64_t _learnStart;

	uORB::Subscription<vehicle_vicon_position_s> _vicon;
	uORB::Subscription<battery_status_s> _batteryStatus;

	enum {CH_LEFT, CH_RIGHT};
	BlockParamFloat _servoTravel;
	BlockParamFloat _wingUp;
	BlockParamFloat _wingDown;
	BlockParamFloat _wingGlide;
	BlockParamFloat _tDown2Up;
	BlockParamFloat _tUp2Glide;
	BlockParamFloat _throttleGlide;
	BlockParamFloat _throttle2Frequency;
	BlockParamFloat _minFrequency;
	BlockParamFloat _lrnTime;
	BlockParamFloat _mutateProb;
	BlockParamFloat _reproducingRatio;
	BlockParamFloat _learnFlapping;
	BlockParamFloat _ailMin;
	BlockParamFloat _ailRange;
	BlockParamFloat _elevMin;
	BlockParamFloat _elevRange;

	// frequency scaling
	BlockParamFloat _kOmega;

	BlockLowPass _wingLeftLowPass;
	BlockLowPass _wingRightLowPass;

	float _powerSum;
	uint32_t _powerCount;

	// learning
	GeneticAlgorithm _ga;

	struct pollfd _attPoll;
	uint64_t _timeStamp;
	uint64_t _cycleStartTimeStamp;
	void flappingFunction(float dt, float aileron,
			float elevator, float throttle,
			float delta,
			float & wingLeft, float & wingRight);
};
