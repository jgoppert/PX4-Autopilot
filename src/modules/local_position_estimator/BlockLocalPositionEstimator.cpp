#include "BlockLocalPositionEstimator.hpp"

BlockLocalPositionEstimator::BlockLocalPositionEstimator() :
	// this block has no parent, and has name LPE
	SuperBlock(NULL,"LPE"), 

	// subscriptions, set rate, add to list
	// TODO topic speed limiting?
	_status(ORB_ID(vehicle_status), 0, &getSubscriptions()),
	_armed(ORB_ID(actuator_armed), 0, &getSubscriptions()),
	_control_mode(ORB_ID(vehicle_control_mode), 0, &getSubscriptions()),
	_att(ORB_ID(vehicle_attitude), 0, &getSubscriptions()),
	_att_sp(ORB_ID(vehicle_attitude_setpoint), 0, &getSubscriptions()),
	_flow(ORB_ID(optical_flow), 0, &getSubscriptions()),
	_sensor(ORB_ID(optical_flow), 0, &getSubscriptions()),
	_range_finder(ORB_ID(sensor_range_finder), 0, &getSubscriptions()),
	_param_update(ORB_ID(parameter_update), 0, &getSubscriptions()),
	_manual(ORB_ID(manual_control_setpoint), 0, &getSubscriptions()),

	// publications
	_pos(ORB_ID(vehicle_local_position), &getPublications()),
	_filtered_flow(ORB_ID(filtered_bottom_flow), &getPublications()),

	// misc
	th2v(this, "TH2V"),
	q2v(this, "Q2V"),
	_polls(),
	_timeStamp(0),
	_time_last_flow(0),
	_loop_perf(),
	_err_perf(),

	// kf matrices
	_A(), _B(), _Q(),
	_C_flow(), _R_flow(), _R_accel(),
	_R_lidar(),
	_x(), _u(), _P()
{
	// setup event triggering based on new flow messages to integrate
	_polls[POLL_FLOW].fd = _flow.getHandle();
	_polls[POLL_FLOW].events = POLLIN;

	_polls[POLL_PARAM].fd = _param_update.getHandle();
	_polls[POLL_PARAM].events = POLLIN;

	_polls[POLL_SENSORS].fd = _sensor.getHandle();
	_polls[POLL_SENSORS].events = POLLIN;

	// derivative of position is velocity
	_A(X_px, X_vx) = 1;
	_A(X_py, X_vy) = 1;
	_A(X_pz, X_vz) = 1;

	// derivative of velocity is accelerometer bias + acceleration
	//_A(X_vx, X_bx) = 1;
	//_A(X_vy, X_by) = 1;
	//_A(X_vz, X_bz) = 1;

	_B(X_vx, U_ax) = 1;
	_B(X_vy, U_ay) = 1;
	_B(X_vz, U_az) = 1;

	// flow measurement matrix
	_C_flow(Y_flow_vx, X_vx) = 1;
	_C_flow(Y_flow_vy, X_vy) = 1;
	_C_flow(Y_flow_z, X_pz) = 1;

	// sonar measurement matrix

	// initialize P to identity
	_P.identity();
	_P *= 0.01;

	// sensor noise standard devations
	float flow_v_stddev = 20.0e-2f;
	float flow_z_stddev = 2.0e-2f;
	float lidar_z_stddev = 5.0e-2f;
	float accel_xy_stddev = 1.0e-3f;
	float accel_z_stddev = 1.0e-2f;

	// initialize measurement noise
	_R_flow(Y_flow_vx, Y_flow_vx) = flow_v_stddev*flow_v_stddev;
	_R_flow(Y_flow_vy, Y_flow_vy) = flow_v_stddev*flow_v_stddev;
	_R_flow(Y_flow_z, Y_flow_z) = flow_z_stddev*flow_z_stddev;

	_R_lidar = lidar_z_stddev*lidar_z_stddev;

	// initialize process noise
	_R_accel(U_ax, U_ax) = accel_xy_stddev*accel_xy_stddev;
	_R_accel(U_ay, U_ay) = accel_xy_stddev*accel_xy_stddev;
	_R_accel(U_az, U_az) = accel_z_stddev*accel_z_stddev;
	_Q = _B*_R_accel*_B.transposed();

	// perf counters
	_loop_perf = perf_alloc(PC_ELAPSED,
			"flow_position_estimator_runtime");
	_interval_perf = perf_alloc(PC_INTERVAL,
			"flow_position_estimator_interval");
	_err_perf = perf_alloc(PC_COUNT, "flow_position_estimator_err");
}

BlockLocalPositionEstimator::~BlockLocalPositionEstimator() {};

void BlockLocalPositionEstimator::update() {

	// wait for a sensor update, check for exit condition every 100 ms
	int ret = poll(_polls, 2, 500);
	if (ret < 0) {
		/* poll error, count it in perf */
		perf_count(_err_perf);
		return;
	}

	uint64_t newTimeStamp = hrt_absolute_time();
	float dt = (newTimeStamp - _timeStamp) / 1.0e6f;
	_timeStamp = newTimeStamp;

	// check for sane values of dt
	// to prevent large control responses
	if (dt > 1.0f || dt < 0) return;

	// set dt for all child blocks
	setDt(dt);

	// see which updates are available
	bool flow_updated = _flow.updated();
	bool params_updated = _flow.updated();

	// get new data
	updateSubscriptions();

	// update parameters
	if (params_updated) updateParams();

	// do prediction
	//_u = math::Vector<3>(_sensor.accelerometer_m_s2);
	//_u(2) += 9.81f; // add g
	_u = math::Vector<3>({0,0,0});
	predict();


	// update flow
	if (flow_updated) { 
		_flow.update();
		perf_begin(_loop_perf);
		update_flow();
		perf_count(_interval_perf);
		perf_end(_loop_perf);
	}



	// publish local position
	_pos.x = _x(X_px);
	_pos.y = _x(X_py);
	_pos.z = _x(X_pz);
	_pos.vx = _x(X_vx);
	_pos.vy = _x(X_vy);
	_pos.vz = _x(X_vz);
	_pos.xy_valid = true;
	_pos.z_valid = true;
	_pos.v_xy_valid = true;
	_pos.v_z_valid = true;
	_pos.timestamp = _timeStamp;
	_pos.dist_bottom_valid = false;
	_pos.eph = 0;
	_pos.epv = 0;
	if(isfinite(_pos.x) && isfinite(_pos.y) && isfinite(_pos.z)
			&& isfinite(_pos.vx) && isfinite(_pos.vy)) {
		_pos.update();
	}

	// publish filtered flow
	if(isfinite(_filtered_flow.sumx) && isfinite(_filtered_flow.sumy) &&
		isfinite(_filtered_flow.vx) && isfinite(_filtered_flow.vy)) {
		_filtered_flow.update();
	}


}

void BlockLocalPositionEstimator::predict() {
	// continuous time kalman filter prediction
	_x += (_A*_x + _B*_u)*getDt();
	_P += (_A*_P + _P*_A.transposed() + _Q)*getDt();
}

void BlockLocalPositionEstimator::update_flow() {
	float flow_speed[3] = {0.0f, 0.0f, 0.0f};
	float speed[3] = {0.0f, 0.0f, 0.0f};
	float global_speed[3] = {0.0f, 0.0f, 0.0f};

	/* rotation matrix for transformation of optical flow speed vectors */
	static const int8_t rotM_flow_sensor[3][3] =   {
		{  0, -1, 0 },
		{ 1, 0, 0 },
		{  0, 0, 1 }}; // 90deg rotated

	/* calc dt between flow timestamps */
	/* ignore first flow msg */
	if (_time_last_flow == 0) {
		_time_last_flow = _flow.timestamp;
		return;
	}
	float dt = (_flow.timestamp - _time_last_flow) * 1.0e-6f ;
	_time_last_flow = _flow.timestamp;

	// calculate velocity over ground
	// TODO, use z estimate instead of flow raw sonar
	if (_flow.integration_timespan > 0) {
		flow_speed[0] = _flow.pixel_flow_x_integral / (_flow.integration_timespan / 1e6f) * _flow.ground_distance_m;
		flow_speed[1] = _flow.pixel_flow_y_integral / (_flow.integration_timespan / 1e6f) * _flow.ground_distance_m;
	} else {
		flow_speed[0] = 0;
		flow_speed[1] = 0;
	}
	flow_speed[2] = 0.0f;

	/* convert to bodyframe velocity */
	for(uint8_t i = 0; i < 3; i++) {
		float sum = 0.0f;
		for(uint8_t j = 0; j < 3; j++) {
			sum = sum + flow_speed[j] * rotM_flow_sensor[j][i];
		}
		speed[i] = sum;
	}

	/* update filtered flow */
	_filtered_flow.sumx += speed[0] * dt;
	_filtered_flow.sumy += speed[1] * dt;
	_filtered_flow.vx = speed[0];
	_filtered_flow.vy = speed[1];

	// TODO add yaw rotation correction (with distance to vehicle zero)

	/* convert to globalframe velocity
	 * -> local position is currently not used for position control
	 */
	for(uint8_t i = 0; i < 3; i++) {
		float sum = 0.0f;
		for(uint8_t j = 0; j < 3; j++) {
			sum = sum + speed[j] * PX4_R(_att.R, i, j);
		}
		global_speed[i] = sum;
	}

	// measurement 
	math::Vector<3> y_flow;
	y_flow(0) = global_speed[0];
	y_flow(1) = global_speed[1];
	y_flow(2) = _flow.ground_distance_m;

	// residual
	math::Matrix<n_y_flow, n_y_flow> S_I = (_C_flow*_P*_C_flow.transposed() + _R_flow).inversed();
	math::Vector<3> r = y_flow - _C_flow*_x;

	// fault detection
	float beta = sqrtf(r*(S_I*r));

	// kalman filter correction if no fault
	if (beta < 10) {
		math::Matrix<n_x, n_y_flow> K = _P*_C_flow.transposed()*S_I;
		_x += K*r;
		_P -= K*_C_flow*_P;
	}
}

void BlockLocalPositionEstimator::update_baro() {
}

void BlockLocalPositionEstimator::update_lidar() {
}
