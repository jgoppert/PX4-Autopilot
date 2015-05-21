#include "BlockTurretController.hpp"

// px4
#include <geo/geo.h>
#include <drivers/drv_hrt.h>
#include <uORB/topics/vehicle_status.h>

BlockTurretController::BlockTurretController() :
	SuperBlock(NULL, "TRT"),

	// subscriptions
	_att(ORB_ID(vehicle_attitude), 0, &getSubscriptions()),
	//_pos(ORB_ID(vehicle_global_position), 0, &getSubscriptions()),
	_posCmd(ORB_ID(position_setpoint_triplet), 0, &getSubscriptions()),
	//_lpos(ORB_ID(vehicle_local_position), 0, &getSubscriptions()),
	//_lposCmd(ORB_ID(vehicle_local_position_setpoint), 0, &getSubscriptions()),
	_manual(ORB_ID(manual_control_setpoint), 0, &getSubscriptions()),
	_status(ORB_ID(vehicle_status), 0, &getSubscriptions()),
	_param_update(ORB_ID(parameter_update), 0, &getSubscriptions()),
	//_encoders(ORB_ID(encoders), 0, &getSubscriptions()),
	//_battery(ORB_ID(battery_status), 0, &getSubscriptions()),

	// publications
	_attCmd(ORB_ID(vehicle_attitude_setpoint), &getPublications()),
	//_ratesCmd(ORB_ID(vehicle_rates_setpoint), &getPublications()),
	//_globalVelCmd(ORB_ID(vehicle_global_velocity_setpoint), &getPublications()),
	_actuators0(ORB_ID(actuator_controls_0), &getPublications()),
	_actuators1(ORB_ID(actuator_controls_1), &getPublications()),

	// blocks
	_azm2D(this, "AZM_P"),
	_elv2D(this, "ELV_P"),
	_r2D(this, "AZM_D"),
	_q2D(this, "ELV_D"),
	_azmLimit(this, "LIM_AZM"),
	_elvLimit(this, "LIM_ELV"),

	// params
	_rMaxTrigger(this, "TRG_RMAX"),
	_attEMaxTrigger(this, "TRG_AMAX"),

	// timing
	_attPoll(),
	_timeStamp(0)
{
	_attPoll.fd = _att.getHandle();
	_attPoll.events = POLLIN;
}

void BlockTurretController::update()
{
	// wait for a sensor update, check for exit condition every 100 ms
	if (poll(&_attPoll, 1, 100) < 0) { return; } // poll error

	uint64_t newTimeStamp = hrt_absolute_time();
	float dt = (newTimeStamp - _timeStamp) / 1.0e6f;
	_timeStamp = newTimeStamp;

	// check for sane values of dt
	// to prevent large control responses
	if (dt > 1.0f || dt < 0) { return; }

	// set dt for all child blocks
	setDt(dt);

	// check for new updates
	if (_param_update.updated()) { updateParams(); }

	// get new information from subscriptions
	updateSubscriptions();

	// duty cycles
	float azmD = 0;
	float elvD = 0;

	// trigger control
	bool trigger = false;

	if (_status.main_state == vehicle_status_s::MAIN_STATE_MANUAL)  {
		// manually control motor voltage
		elvD = _manual.x;
		azmD = _manual.y;
		if (_manual.z > 0.5f) {
			trigger = true;
		}
		warnx("manual mode");
	} else if (_status.main_state == vehicle_status_s::MAIN_STATE_ALTCTL ||
		_status.main_state == vehicle_status_s::MAIN_STATE_POSCTL ||
		_status.main_state == vehicle_status_s::MAIN_STATE_AUTO_MISSION ||
		_status.main_state == vehicle_status_s::MAIN_STATE_AUTO_LOITER ||
		_status.main_state == vehicle_status_s::MAIN_STATE_AUTO_RTL ||
		_status.main_state == vehicle_status_s::MAIN_STATE_OFFBOARD) {

		// commanded reference positions to track
		float azmR = 0;
		float elvR = 0;

		if (_status.main_state == vehicle_status_s::MAIN_STATE_OFFBOARD) {
			float x = _posCmd.current.x;
			float y = _posCmd.current.y;
			float z = _posCmd.current.z;
			float r = sqrtf(x*x + y*y + z*z);
			// reference
			azmR = atan2f(y, x);
			elvR = -asinf(z/r);
			// error
			float azmE = azmR - _att.yaw;
			float elvE = elvR - _att.pitch;
			float attE = sqrtf(azmE*azmE + elvE*elvE);
			if (attE < _attEMaxTrigger.get() &&
					r < _rMaxTrigger.get() &&
					r > _rMaxTrigger.get()) {
				trigger = true;
			}
			warnx("mode: offboard");
		} else if (_status.main_state == vehicle_status_s::MAIN_STATE_ALTCTL ||
			_status.main_state == vehicle_status_s::MAIN_STATE_POSCTL) {
			elvR = _manual.x;
			azmR = _manual.y;
			warnx("mode: stabilize");
		} else {
			elvR = 0;
			azmR = 0;
			warnx("mode: auto");
		}

		warnx("elvR: %5.2f, azmR: %5.2f", double(elvR), double(azmR));

		// update attitude command to visible to ground station
		_attCmd.roll_body = 0;
		_attCmd.pitch_body = elvR;
		_attCmd.yaw_body = azmR;

		// control loops
		azmD = _azm2D.update(_azmLimit.update(azmR) - _att.yaw) 
			- _r2D.update(_att.yawspeed);
		elvD = _elv2D.update(_elvLimit.update(elvR) - _att.pitch) 
			- _r2D.update(_att.pitchspeed);
	}
	warnx("elvD: %5.2f, azmD: %5.2f", double(elvD), double(azmD));

	// send to motors if armed
	if (_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
		warnx("armed");
		// group 0 (servo pwm output)
		_actuators0.control[0] = azmD;
		_actuators0.control[1] = elvD;
		_actuators0.control[2] = trigger;
		// group 1 (roboclaw)
		_actuators1.control[0] = azmD;
		_actuators1.control[1] = elvD;
	} else {
		warnx("disarmed");
		_actuators0.control[0] = 0;
		_actuators0.control[1] = 0;
		_actuators0.control[2] = 0;
		_actuators1.control[0] = 0;
		_actuators1.control[1] = 0;
	}

	// call custom publication update function
	updatePublications();
}
