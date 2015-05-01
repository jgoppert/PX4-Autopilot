#pragma once

// system
#include <poll.h>

// subscription topics
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/encoders.h>
#include <uORB/topics/battery_status.h>

// publication topics
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_global_velocity_setpoint.h>
#include <uORB/topics/actuator_controls.h>

// control blocks
#include <controllib/blocks.hpp>

#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>

using namespace control;

class BlockTurretController : public control::SuperBlock
{
public:
	BlockTurretController();
	void update();
private:
	enum {CH_AZIM, CH_ELV};

	// subscriptions
	uORB::Subscription<vehicle_attitude_s> _att;
	//uORB::Subscription<vehicle_global_position_s> _pos;
	//uORB::Subscription<position_setpoint_triplet_s> _posCmd;
	//uORB::Subscription<vehicle_local_position_s> _lpos;
	uORB::Subscription<vehicle_local_position_setpoint_s> _lposCmd;
	uORB::Subscription<manual_control_setpoint_s> _manual;
	uORB::Subscription<vehicle_status_s> _status;
	uORB::Subscription<parameter_update_s> _param_update;
	//uORB::Subscription<encoders_s> _encoders;
	//uORB::Subscription<battery_status_s> _battery;

	// publications
	uORB::Publication<vehicle_attitude_setpoint_s> _attCmd;
	//uORB::Publication<vehicle_rates_setpoint_s> _ratesCmd;
	//uORB::Publication<vehicle_global_velocity_setpoint_s> _globalVelCmd;
	uORB::Publication<actuator_controls_s> _actuators0;
	uORB::Publication<actuator_controls_s> _actuators1;

	// control blocks
	BlockP _azm2D; // azimuth error to azimuth motor duty cycle
	BlockP _elv2D; // elevation error to elevation motor duty cycle
	BlockP _r2D; // azim rate to azim duty
	BlockP _q2D; // elev rate to elev duty
	BlockLimitSym _azmLimit; // azimuth limit
	BlockLimitSym _elvLimit; // elevation limit

	// parameters
	BlockParamFloat _rMaxTrigger; // max radius in m, to trigger event
	BlockParamFloat _attEMaxTrigger; // max attitude error in deg to trigger event

	// timing
	struct pollfd _attPoll; // attitude polling
	uint64_t _timeStamp; // timestamp for loop timing
};
