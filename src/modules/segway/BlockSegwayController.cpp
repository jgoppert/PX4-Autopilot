#include "BlockSegwayController.hpp"

// px4
#include <geo/geo.h>
#include <drivers/drv_hrt.h>
#include <uORB/topics/vehicle_status.h>

BlockSegwayController::BlockSegwayController() :
	SuperBlock(NULL, "SEG"),

	// subscriptions
	_att(ORB_ID(vehicle_attitude), 0, 0, &getSubscriptions()),
	_pos(ORB_ID(vehicle_global_position), 0, 0, &getSubscriptions()),
	_posCmd(ORB_ID(position_setpoint_triplet), 0, 0, &getSubscriptions()),
	_localPos(ORB_ID(vehicle_local_position), 0, 0, &getSubscriptions()),
	_localPosCmd(ORB_ID(vehicle_local_position_setpoint), 0, 0, &getSubscriptions()),
	_manual(ORB_ID(manual_control_setpoint), 0, 0, &getSubscriptions()),
	_status(ORB_ID(vehicle_status), 0, 0, &getSubscriptions()),
	_param_update(ORB_ID(parameter_update), 0, 0, &getSubscriptions()),
	_encoders(ORB_ID(encoders), 0, 0, &getSubscriptions()),
	_battery(ORB_ID(battery_status), 0, 0, &getSubscriptions()),

	// publications
	_attCmd(ORB_ID(vehicle_attitude_setpoint), -1, &getPublications()),
	_ratesCmd(ORB_ID(vehicle_rates_setpoint), -1,  &getPublications()),
	_globalVelCmd(ORB_ID(vehicle_global_velocity_setpoint), -1, &getPublications()),
	_actuators(ORB_ID(actuator_controls_1), -1, &getPublications()),

	_yaw2r(this, "YAW2R"),
	_r2v(this, "R2V"),
	_th2v(this, "TH2V"),
	_q2v(this, "Q2V"),
	_x2vel(this, "X2VEL"),
	_vel2th(this, "VEL2TH"),
	_thLimit(this, "TH_LIM"),
	_velLimit(this, "VEL_LIM"),
	_thStop(this, "TH_STOP"),
	_trimPitch(this, "TRIM_PITCH", false),
	_pulsesPerRev(this, "ENCP_PPR", false),
	//_mgl(this, "MGL"),
	//_J(this, "J"),
	//_k_emf(this, "K_EMF"),
	//_k_damp(this, "K_DAMP"),
	//_wn_theta(this, "WN_THETA"),
	//_zeta_theta(this, "ZETA_THETA"),
	//_bemf(this, "BEMF"),
	_sysIdEnable(this, "SYSID_ENABLE"),
	_sysIdAmp(this, "SYSID_AMP"),
	_sysIdFreq(this, "SYSID_FREQ"),
	_attPoll(),
	_timeStamp(0),
	_thCmd(0),
	_rCmd(0),
	_yawCmd(0),
	_velCmd(0),
	_xCmd(0),
	_controlPitch(0),
	_controlYaw(0)
{
	_attPoll.fd = _att.getHandle();
	_attPoll.events = POLLIN;
}

void BlockSegwayController::update()
{
	// wait for a sensor update, check for exit condition every 100 ms
	if (px4_poll(&_attPoll, 1, 100) < 0) { return; } // poll error

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

	// handle modes
	if (_sysIdEnable.get() > 0) { // sysid
		handleSysIdModes();
	} else { // normal modes
		handleNormalModes();
	}

	// call custom publication update function
	updatePublications();
}

void BlockSegwayController::handleNormalModes()
{
	setControlsToZero();
	if (_status.get().main_state == vehicle_status_s::MAIN_STATE_MANUAL) {
		// user controls vel cmd and yaw rate cmd
		_thCmd = -_thLimit.getMax()*_manual.get().x; // note negative, since neg pitch goes fwd
		_rCmd = _manual.get().y;
	} else if (_status.get().main_state == vehicle_status_s::MAIN_STATE_ALTCTL) {
		// user controls vel cmd and yaw rate cmd
		_velCmd = _manual.get().x * _velLimit.getMax();
		velCmd2PitchCmd();
		_rCmd = _manual.get().y;
	} else if (_status.get().main_state == vehicle_status_s::MAIN_STATE_ACRO) {
		// user controls th cmd and yaw rate cmd
		_thCmd = -_thLimit.getMax()*_manual.get().x; // note negative, since neg pitch goes fwd
		_rCmd = _manual.get().y;
	} else if (_status.get().main_state == vehicle_status_s::MAIN_STATE_POSCTL) {
		// user controls pos cmd and yaw rate cmd
		_xCmd = 0.5f * _manual.get().x;
		xCmd2VelocityCmd();
		velCmd2PitchCmd();
		_rCmd = _manual.get().y;
	} else if (_status.get().main_state == vehicle_status_s::MAIN_STATE_AUTO_MISSION) {
		_xCmd = _localPosCmd.get().x;
		xCmd2VelocityCmd();
		velCmd2PitchCmd();
		_yawCmd = _localPosCmd.get().yaw;
		yawCmd2YawRateCmd();
	} else if (_status.get().main_state == vehicle_status_s::MAIN_STATE_AUTO_LOITER) {
		_xCmd = _localPosCmd.get().x;
		// TODO check if local pos cmd is set to loiter pos.
		xCmd2VelocityCmd();
		velCmd2PitchCmd();
		_yawCmd = _localPosCmd.get().yaw;
		yawCmd2YawRateCmd();
	} else if (_status.get().main_state == vehicle_status_s::MAIN_STATE_AUTO_RTL) {
		_xCmd = 0;
		xCmd2VelocityCmd();
		velCmd2PitchCmd();
		_yawCmd = _localPosCmd.get().yaw;
		yawCmd2YawRateCmd();
	} else if (_status.get().main_state == vehicle_status_s::MAIN_STATE_OFFBOARD) {
	}

	// compute angles and rates
	float th = _att.get().pitch -_trimPitch.get();
	//float th_dot = _att.pitchspeed;
	//float r = _att.yawspeed;
	//float alpha_dot_left = _encoders.velocity[0]/_pulsesPerRev.get();
	//float alpha_dot_right = _encoders.velocity[1]/_pulsesPerRev.get();
	//float alpha_dot_mean = (alpha_dot_left + alpha_dot_right)/2;
	//float alpha_dot_diff = (alpha_dot_left - alpha_dot_right);

	// constants
	//float k_emf = _k_emf.get();
	//float k_damp = _k_damp.get();
	//float wn_theta = _wn_theta.get();
	//float zeta_theta = _zeta_theta.get();
	//float J = _J.get();
	//float mgl = _mgl.get();

	// dynamic inversion
	//float th_ddot_d = -2*zeta_theta*wn_theta*th_dot - wn_theta*wn_theta*(th - _thCmd);
	//float V_pitch = -J*th_ddot_d/(2*k_emf) - mgl*sinf(th)/(2*k_emf) + alpha_dot_mean*k_damp/k_emf;
	//float V_yaw = _r2v.update(_rCmd - r) + alpha_dot_diff*k_damp/k_emf;

	//float inv_dynamics_yaw = _bemf.get()*
	//	(alpha_dot_left - alpha_dot_right)/2;
	//float inv_dynamics_pitch = _mgl.get()*sinf(th)
	//	+ _bemf.get()*(alpha_dot_left + alpha_dot_right)/2;

	// compute control for pitch
	_controlPitch = _th2v.update(_thCmd - th)
		- _q2v.update(_att.get().pitchspeed); // - inv_dynamics_pitch;

	// compute control for yaw
	_controlYaw = _r2v.update(_rCmd - _att.get().yawspeed); // - inv_dynamics_yaw;

	// output scaling by manual throttle
	_controlPitch *= _manual.get().z;
	_controlYaw *= _manual.get().z;
}

void BlockSegwayController::handleSysIdModes()
{
	setControlsToZero();
	float sineWave = _sysIdAmp.get() *
		sinf(2.0f * M_PI_F * _sysIdFreq.get() * _timeStamp / 1.0e6f);
	float squareWave = 0;
	if (sineWave > 0) {
		squareWave = _sysIdAmp.get();
	} else {
		squareWave = -_sysIdAmp.get();
	}
	if (_status.get().main_state == vehicle_status_s::MAIN_STATE_MANUAL) {
		_controlPitch = _manual.get().x;
	} else if (_status.get().main_state == vehicle_status_s::MAIN_STATE_ALTCTL) {
		_controlPitch = sineWave;
	} else if (_status.get().main_state == vehicle_status_s::MAIN_STATE_POSCTL) {
		_controlPitch = squareWave;
	}
	// output scaling by manual throttle
	_controlPitch *= _manual.get().z;
	_controlYaw *= _manual.get().z;
}

void BlockSegwayController::updatePublications() {
	// attitude set point
	_attCmd.get().timestamp = _timeStamp;
	_attCmd.get().pitch_body = _thCmd;
	_attCmd.get().roll_body = 0;
	_attCmd.get().yaw_body = _yawCmd;
	_attCmd.get().R_valid = false;
	_attCmd.get().q_d_valid = false;
	_attCmd.get().q_e_valid = false;
	_attCmd.get().thrust = 0;
	_attCmd.get().roll_reset_integral = false;
	_attCmd.update();

	// rates set point
	_ratesCmd.get().timestamp = _timeStamp;
	_ratesCmd.get().roll = 0;
	_ratesCmd.get().pitch = 0;
	_ratesCmd.get().yaw = _rCmd;
	_ratesCmd.get().thrust = 0;
	_ratesCmd.update();

	// global velocity set point
	_globalVelCmd.get().vx = _velCmd;
	_globalVelCmd.get().vy = 0;
	_globalVelCmd.get().vz = 0;
	_globalVelCmd.update();

	// normalize output using battery voltage,
	// keeps performance same as battery voltage decreases
	float V_batt = _battery.get().voltage_v;
	float dutyPitch = _controlPitch*(12.0f/V_batt);
	float dutyYaw = _controlYaw*(12.0f/V_batt);

	// send outputs if armed and pitch less
	// than shut off pitch
	if (_status.get().arming_state == vehicle_status_s::ARMING_STATE_ARMED &&
	    fabsf(_att.get().pitch) < _thStop.get()) {
		// controls
		_actuators.get().timestamp = _timeStamp;
		_actuators.get().control[0] = dutyPitch - dutyYaw;
		_actuators.get().control[1] = dutyPitch + dutyYaw;
		_actuators.get().control[2] = 0;
		_actuators.get().control[3] = 0;
		_actuators.update();

	} else {
		// controls
		_actuators.get().timestamp = _timeStamp;
		_actuators.get().control[0] = 0;
		_actuators.get().control[1] = 0;
		_actuators.get().control[2] = 0;
		_actuators.get().control[3] = 0;
		_actuators.update();
	}
}

void BlockSegwayController::xCmd2VelocityCmd()
{
	_velCmd = _velLimit.update(_x2vel.update(_xCmd - _localPos.get().x));
}

void BlockSegwayController::yawCmd2YawRateCmd()
{
	float yawError = _yawCmd - _att.get().yaw;

	// wrap yaw error to between -180 and 180
	if (yawError > M_PI_F / 2) { yawError = yawError - 2 * M_PI_F; }

	if (yawError < -M_PI_F / 2) { yawError = yawError + 2 * M_PI_F; }

	_rCmd = _yaw2r.update(yawError);
}

void BlockSegwayController::velCmd2PitchCmd()
{
	// negative sign since need to lean in negative pitch to move forward
	_thCmd = -_thLimit.update(_vel2th.update(_velCmd - _localPos.get().vx));
}

void BlockSegwayController::setControlsToZero() {
	// initialize all controls to zero
	_thCmd = 0; // pitch command
	_rCmd = 0; // yaw rate command
	_yawCmd = 0; // always point north for now, can use localPosCmd.yaw later
	_velCmd = 0; // velocity command
	_xCmd = 0; // position command
	_controlPitch = 0;
	_controlYaw = 0;
}
