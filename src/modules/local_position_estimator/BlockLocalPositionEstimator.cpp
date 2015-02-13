#include "BlockLocalPositionEstimator.hpp"

BlockLocalPositionEstimator::BlockLocalPositionEstimator() :
	// this block has no parent, and has name LPE
	SuperBlock(NULL,"LPE"), 

	// subscriptions, set rate, add to list
	_status(ORB_ID(vehicle_status), 500, &getSubscriptions()),
	_armed(ORB_ID(actuator_armed), 500, &getSubscriptions()),
	_control_mode(ORB_ID(vehicle_control_mode), 500, &getSubscriptions()),
	_att(ORB_ID(vehicle_attitude), 20, &getSubscriptions()),
	_att_sp(ORB_ID(vehicle_attitude_setpoint), 20, &getSubscriptions()),
	_flow(ORB_ID(optical_flow), 20, &getSubscriptions()),
	_sensor(ORB_ID(optical_flow), 20, &getSubscriptions()),
	_range_finder(ORB_ID(sensor_range_finder), 20, &getSubscriptions()),
	_param_update(ORB_ID(parameter_update), 500, &getSubscriptions()),
	_manual(ORB_ID(manual_control_setpoint), 500, &getSubscriptions()),

	// publications
	_pos(ORB_ID(vehicle_local_position), &getPublications()),

	// misc
	th2v(this, "TH2V"),
	q2v(this, "Q2V"),
	_flowPoll(),
	_timeStamp(0),

	// kf matrices
	_A(), _B(), _C_pos(), _C_vel()
{
	// setup event triggering based on new flow messages to integrate
	_flowPoll.fd = _flow.getHandle();
	_flowPoll.events = POLLIN;

	// derivative of position is velocity
	_A(X_px, X_vx) = 1;
	_A(X_py, X_vy) = 1;
	_A(X_pz, X_vz) = 1;

	// derivative of velocity is accelerometer bias + acceleration
	_A(X_vx, X_bx) = 1;
	_A(X_vy, X_by) = 1;
	_A(X_vz, X_bz) = 1;

	_B(X_vx, U_ax) = 1;
	_B(X_vy, U_ay) = 1;
	_B(X_vz, U_az) = 1;

	// position measurement
	_C_pos(Y_pos_px, X_px) = 1;
	_C_pos(Y_pos_py, X_py) = 1;
	_C_pos(Y_pos_pz, X_pz) = 1;

	// velocity measurement
	_C_vel(Y_vel_vx, X_vx) = 1;
	_C_vel(Y_vel_vy, X_vy) = 1;
	_C_vel(Y_vel_vz, X_vz) = 1;
}

BlockLocalPositionEstimator::~BlockLocalPositionEstimator() {};

void BlockLocalPositionEstimator::update() {
	// wait for a sensor update, check for exit condition every 100 ms
	if (poll(&_flowPoll, 1, 100) < 0) return; // poll error

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

	// test position
	_pos.x = 1;
	_pos.y = 2;
	_pos.z = 3;

	// update all publications
	updatePublications();
}

void BlockLocalPositionEstimator::predict() {
}

void BlockLocalPositionEstimator::update_flow() {
}

void BlockLocalPositionEstimator::update_baro() {
}

void BlockLocalPositionEstimator::update_lidar() {
}

void BlockLocalPositionEstimator::update_sonar() {
}
