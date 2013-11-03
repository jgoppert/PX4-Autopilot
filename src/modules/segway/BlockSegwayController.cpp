#include "BlockSegwayController.hpp"

// px4
#include <geo/geo.h>
#include <drivers/drv_hrt.h>

BlockSegwayController::BlockSegwayController() :
	SuperBlock(NULL,"SEG"),

	// subscriptions
	_att(&getSubscriptions(), ORB_ID(vehicle_attitude), 3),
	_pos(&getSubscriptions() , ORB_ID(vehicle_global_position), 3),
	_posCmd(&getSubscriptions(), ORB_ID(vehicle_global_position_set_triplet), 3),
	_localPos(&getSubscriptions() , ORB_ID(vehicle_local_position), 3),
	_localPosCmd(&getSubscriptions(), ORB_ID(vehicle_local_position_setpoint), 3),
	_manual(&getSubscriptions(), ORB_ID(manual_control_setpoint), 3),
	_status(&getSubscriptions(), ORB_ID(vehicle_status), 3),
	_param_update(&getSubscriptions(), ORB_ID(parameter_update), 1000), // limit to 1 Hz
	_encoders(&getSubscriptions(), ORB_ID(encoders), 10), // limit to 100 Hz

	// publications
	_attCmd(&getPublications(), ORB_ID(vehicle_attitude_setpoint)),
	_ratesCmd(&getPublications(), ORB_ID(vehicle_rates_setpoint)),
	_globalVelCmd(&getPublications(), ORB_ID(vehicle_global_velocity_setpoint)),
	_actuators(&getPublications(), ORB_ID(actuator_controls_1)),

	_yaw2r(this, "YAW2R"),
	_r2v(this, "R2V"),
	_th2v(this, "TH2V"),
	_q2v(this, "Q2V"),
	_x2vel(this, "X2VEL"),
	_vel2th(this, "VEL2TH"),
	_thLimit(this, "TH_LIM"),
	_velLimit(this, "VEL_LIM"),
	_thStop(this, "TH_STOP"),
	_sysIdAmp(this, "SYSID_AMP"),
	_sysIdFreq(this, "SYSID_FREQ"),
	_attPoll(),
	_timeStamp(0)
{
	orb_set_interval(_att.getHandle(), 10); // set attitude update rate to 100 Hz (period 10 ms)
	_attPoll.fd = _att.getHandle();
	_attPoll.events = POLLIN;
}


void BlockSegwayController::update() {
	// wait for a sensor update, check for exit condition every 100 ms
	if (poll(&_attPoll, 1, 100) < 0) return; // poll error

	uint64_t newTimeStamp = hrt_absolute_time();
	float dt = (newTimeStamp - _timeStamp) / 1.0e6f;
	_timeStamp = newTimeStamp;

	// check for sane values of dt
	// to prevent large control responses
	if (dt > 1.0f || dt < 0) return;

	// set dt for all child blocks
	setDt(dt);

	// check for new updates
	if (_param_update.updated()) updateParams();

	// get new information from subscriptions
	updateSubscriptions();

	// default all output to zero unless handled by mode
	for (unsigned i = 2; i < NUM_ACTUATOR_CONTROLS; i++)
		_actuators.control[i] = 0.0f;

	// only update guidance in auto mode
	if (_status.main_state == MAIN_STATE_AUTO) {
		// update guidance
	}

	// commands for inner stabilization loop
	float thCmd = 0; // pitch command
	float rCmd = 0; // yaw rate command
	float yawCmd = 0; // always point north for now, can use localPosCmd.yaw later
	float velCmd = 0; // velocity command

	// syste id
	float t = _timeStamp/1.0e6;

	// modes that track position
	if (_status.main_state == MAIN_STATE_AUTO || 
		_status.main_state == MAIN_STATE_EASY || 
		_status.main_state == MAIN_STATE_SEATBELT) {

		// the position to track
		float localPosCmdX = 0;

		// auto mode follows waypoints
		if ( _status.main_state == MAIN_STATE_AUTO) {
			localPosCmdX = _localPosCmd.x;
		// system id with square wave
		} else if (_status.main_state == MAIN_STATE_EASY) {
			float sineWave = sinf(2*M_PI_F*_sysIdFreq.get()*t);
			float squareWave = 0;
			if (sineWave > 0) {
				squareWave = _sysIdAmp.get();
			} else {
				squareWave = -_sysIdAmp.get();
			}
			localPosCmdX = squareWave;
		// system id with sine wave
		} else if (_status.main_state == MAIN_STATE_SEATBELT) {
			float sineWave = sinf(2*M_PI_F*_sysIdFreq.get()*t);
			localPosCmdX = sineWave;
		}

		// track the position command
		velCmd = _velLimit.update(_x2vel.update(localPosCmdX - _localPos.x));
		// negative sign since need to lean in negative pitch to move forward
		thCmd = -_thLimit.update(_vel2th.update(velCmd - _localPos.vx));
		float yawError = yawCmd - _att.yaw;
		// wrap yaw error to between -180 and 180
		if (yawError > M_PI_F/2) yawError = yawError - 2*M_PI_F;
		if (yawError < -M_PI_F/2) yawError = yawError + 2*M_PI_F;
		rCmd = _yaw2r.update(yawError);

	// manual mode
	} else if (_status.main_state == MAIN_STATE_MANUAL) {
		rCmd = _manual.yaw;
		thCmd = _manual.pitch;
		velCmd = 0;
	}

	// compute control for pitch
	float controlPitch = _th2v.update(thCmd - _att.pitch)
		- _q2v.update(_att.pitchspeed);

	// compute control for yaw
	float controlYaw = _r2v.update(rCmd - _att.yawspeed);

	// attitude set point
	_attCmd.timestamp = _timeStamp;
	_attCmd.pitch_body = thCmd;
	_attCmd.roll_body = 0;
	_attCmd.yaw_body = yawCmd;
	_attCmd.R_valid = false;
	_attCmd.q_d_valid = false;
	_attCmd.q_e_valid = false;
	_attCmd.thrust = 0;
	_attCmd.roll_reset_integral = false;
	_attCmd.update();

	// rates set point
	_ratesCmd.timestamp = _timeStamp;
	_ratesCmd.roll = 0;
	_ratesCmd.pitch = 0;
	_ratesCmd.yaw = rCmd;
	_ratesCmd.thrust = 0;
	_ratesCmd.update();

	// global velocity set point
	_globalVelCmd.vx = velCmd;
	_globalVelCmd.vy = 0;
	_globalVelCmd.vz = 0;
	_globalVelCmd.update();

	// send outputs if armed and pitch less
	// than shut off pitch
	if (_status.arming_state == ARMING_STATE_ARMED &&
			fabsf(_att.pitch) < _thStop.get() ) {
		// controls
		_actuators.timestamp = _timeStamp;
		_actuators.control[0] = 0; // roll
		_actuators.control[1] = controlPitch; // pitch
		_actuators.control[2] = controlYaw; // yaw
		_actuators.control[3] = 0; // thrust
		_actuators.update();
	} else {
		// controls
		_actuators.timestamp = _timeStamp;
		_actuators.control[0] = 0; // roll
		_actuators.control[1] = 0; // pitch
		_actuators.control[2] = 0; // yaw
		_actuators.control[3] = 0; // thrust
		_actuators.update();
	}
}

