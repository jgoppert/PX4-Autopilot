#pragma once

#include <controllib/uorb/blocks.hpp>
#include <uORB/topics/encoders.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/parameter_update.h>

using namespace control;

class BlockEncoderPositionEstimator : public SuperBlock {
public:
	BlockEncoderPositionEstimator();
	void update();
private:
	UOrbSubscription<vehicle_attitude_s> _att;
	UOrbSubscription<parameter_update_s> _param_update;
	UOrbPublication<vehicle_local_position_s> _localPos;
	UOrbPublication<vehicle_global_position_s> _pos;
	UOrbSubscription<encoders_s> _encoders;
	BlockParamFloat _rWheel;
	BlockParamFloat _pulsesPerRev;
	struct pollfd _poll;
	uint64_t _timeStamp;
};
