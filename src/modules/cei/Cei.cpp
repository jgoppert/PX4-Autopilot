#include "Cei.hpp"
#include "gen/att_lgpekf/casadi_att_lgpekf.h"
#include <cstdlib>
#include <drivers/drv_hrt.h>
#include <string.h>
#include <assert.h>


Cei::Cei() :
	SuperBlock(nullptr, "CEI"),
	ModuleParams(nullptr),
	_perf_elapsed(),
	_initialized(false),

	// blocks
	_mag_stats(this, ""),
	_accel_stats(this, ""),

	// casadi functions
	_mrp_shadow(mrp_shadow_functions()),
	_mrp_to_quat(mrp_to_quat_functions()),
	_quat_to_euler(quat_to_euler_functions()),
	_predict_W(predict_W_functions()),
	_x_predict(x_predict_functions()),
	_correct_accel(correct_accel_functions()),
	_correct_mag(correct_mag_functions()),
	_init(init_functions()),

	// subscriptions
	_sub_param_update(ORB_ID(parameter_update), 1000 / 2, 0, &getSubscriptions()),
	_sub_sensor(ORB_ID(sensor_combined), 1000 / 200, 0, &getSubscriptions()),
	_sub_mag(ORB_ID(vehicle_magnetometer), 1000 / 50, 0, &getSubscriptions()),
	_sub_gps(ORB_ID(vehicle_gps_position), 1000 / 10, 0, &getSubscriptions()),

	// publications
	_pub_lpos(ORB_ID(vehicle_local_position), -1, &getPublications()),
	_pub_gpos(ORB_ID(vehicle_global_position), -1, &getPublications()),
	_pub_att(ORB_ID(vehicle_attitude), -1, &getPublications()),
	_pub_est(ORB_ID(estimator_status), -1, &getPublications()),
	_pub_innov(ORB_ID(ekf2_innovations), -1, &getPublications()),

	// misc
	_polls()
{
	// counters
	_perf_elapsed = perf_alloc(PC_ELAPSED, "cei_elapsed");

	_polls[POLL_PARAM].fd = _sub_param_update.getHandle();
	_polls[POLL_PARAM].events = POLLIN;

	_polls[POLL_SENSORS].fd = _sub_sensor.getHandle();
	_polls[POLL_SENSORS].events = POLLIN;

	_x.setZero();
	_W.setAll(0.1);
	status();
}

Cei::~Cei()
{
	perf_free(_perf_elapsed);
}

void Cei::update()
{
	perf_begin(_perf_elapsed);

	// wait for a sensor update, check for exit condition every 100 ms
	int ret = px4_poll(_polls, n_poll, 100);

	if (ret < 0) {
		PX4_WARN("poll failed");
		return;
	}

	uint64_t now = hrt_absolute_time();
	float dt = (now - _timeStamp) / 1.0e6f;

	// abort if not enough time elapsed
	if (dt < 1e-3f) {
		return;
	}

	_timeStamp =  now;

	// check for sane update rate
	if (dt > 0.1f) {
		PX4_WARN("update rate slow: %12.5f", double(dt));
		return;
	}

	// set dt for all child blocks
	setDt(dt);

	// get updates
	bool mag_updated = _sub_mag.updated();
	bool accel_updated = _sub_sensor.updated();

	// update all subsription data
	updateSubscriptions();

	// parameters
	const float w_att = 0.1;
	const float decl = 0.1;
	const float w_mag = 0.1;

	// predict
	if (!_initialized) {

		_mag_stats.update(matrix::Vector3f(_sub_mag.get().magnetometer_ga));
		_accel_stats.update(matrix::Vector3f(_sub_sensor.get().accelerometer_m_s2));

		if (_mag_stats.getCount() > 10 and _accel_stats.getCount() > 10) {

			/* init:(g_b[3],B_b[3],decl)->(init_valid,x0[6]) */
			float valid = 0;
			float x1[6] = {0};
			matrix::Vector3f y_accel = _accel_stats.getMean();
			matrix::Vector3f y_mag = _mag_stats.getMean();

			_init.arg(0, y_accel.data());
			_init.arg(1, y_mag.data());
			_init.arg(2, &decl);
			_init.res(0, &valid);
			_init.res(1, x1);
			_init.eval();

			if (int(valid)) {
				_initialized = true;
				PX4_INFO("initialized");

			} else {
				PX4_INFO("initialization failed: %f", double(valid));
				_accel_stats.getMean().print();
				_mag_stats.getMean().print();
				matrix::Vector<float, 6>(x1).print();
			}
		}

	} else {

		// prediction
		{
			const float *omega_b = _sub_sensor.get().gyro_rad;

			/* x_predict:(x0[6],omega_b[3],dt)->(x1[6]) */
			float x1[n_x] = {0};
			_x_predict.arg(0, _x.data());
			_x_predict.arg(1, omega_b);
			_x_predict.arg(2, &dt);
			_x_predict.res(0, x1);
			_x_predict.eval();

			/* predict_W:(x_h[6],W0[6x6,21nz],w_att,omega_b[3],dt)->(W1[6x6,21nz]) */
			float W1[n_W] = {0};
			_predict_W.arg(0, _x.data());
			_predict_W.arg(1, _W.data());
			_predict_W.arg(2, &w_att);
			_predict_W.arg(3, omega_b);
			_predict_W.arg(4, &dt);
			_predict_W.res(0, W1);
			_predict_W.eval();

			correct_if_finite(x1, W1, "predict");
		}

		// correct mag
		if (mag_updated) {
			/* correct_mag:(x_h[6],W[6x6,21nz],y_b[3],decl,w_mag)->(x_mag[6],W_mag[6x6,21nz]) */
			const float *y_b = _sub_mag.get().magnetometer_ga;
			float x1[n_x] = {0};
			float W1[n_W] = {0};
			_correct_mag.arg(0, _x.data());
			_correct_mag.arg(1, _W.data());
			_correct_mag.arg(2, y_b);
			_correct_mag.arg(3, &decl);
			_correct_mag.arg(4, &w_mag);
			_correct_mag.res(0, x1);
			_correct_mag.res(1, W1);
			_correct_mag.eval();
			correct_if_finite(x1, W1, "mag");
		}

		// correct accel
		if (accel_updated) {
			/* correct_mag:(x_h[6],W[6x6,21nz],y_b[3],decl,w_mag)->(x_mag[6],W_mag[6x6,21nz]) */
			const float *y_b = _sub_sensor.get().accelerometer_m_s2;
			float x1[n_x] = {0};
			float W1[n_W] = {0};
			_correct_accel.arg(0, _x.data());
			_correct_accel.arg(1, _W.data());
			_correct_accel.arg(2, y_b);
			_correct_accel.arg(3, &decl);
			_correct_accel.arg(4, &w_mag);
			_correct_accel.res(0, x1);
			_correct_accel.res(1, W1);
			_correct_accel.eval();
			correct_if_finite(x1, W1, "accel");
		}
	}

	// publish local position
	if (true) {
		vehicle_local_position_s &lpos = _pub_lpos.get();
		lpos.ax = 0.1;
		lpos.ay = 0.1;
		lpos.az = 0.1;
		lpos.delta_vxy[0] = 0;
		lpos.delta_vxy[1] = 0;
		lpos.delta_vz = 0;
		lpos.delta_xy[0] = 0;
		lpos.delta_xy[1] = 0;
		lpos.delta_z = 0;
		lpos.dist_bottom = 0.1;
		lpos.dist_bottom_rate = 0.1;
		lpos.dist_bottom_valid = true;
		lpos.eph = 0.1;
		lpos.epv = 0.1;
		lpos.evh = 0.1;
		lpos.evv = 0.1;
		lpos.limit_hagl = false;
		lpos.ref_alt = 0.1;
		lpos.ref_lat = 0.1;
		lpos.ref_lon = 0.1;
		lpos.ref_timestamp = now;
		lpos.timestamp = now;
		lpos.vx = 0.01; //_x[X_vx];
		lpos.vxy_max = 0.1;
		lpos.vxy_reset_counter = 0;
		lpos.vy = 0; //_x[X_vy];
		lpos.vz = 0; //_x[X_vz];
		lpos.vz_reset_counter = 0;
		lpos.v_xy_valid = true;
		lpos.v_z_valid = true;
		lpos.x = 0; //_x[X_x];
		lpos.xy_global = true;
		lpos.xy_reset_counter = 0;
		lpos.xy_valid = true;
		lpos.y = 0; //_x[X_y];
		lpos.yaw = 0.1;
		lpos.z = 0; //_x[X_z];
		lpos.z_deriv = 0.1;
		lpos.z_global = true;
		lpos.z_reset_counter = 0;
		lpos.z_valid = true;
		_pub_lpos.update();
	}

	// publish vehicle_attitude
	if (true) {
		vehicle_attitude_s &att = _pub_att.get();
		att.delta_q_reset[0] = 0;
		att.delta_q_reset[1] = 0;
		att.delta_q_reset[2] = 0;
		att.delta_q_reset[3] = 0;
		att.pitchspeed = 0.1;
		att.q[0] = 1;
		att.q[1] = 0;
		att.q[2] = 0;
		att.q[3] = 0;
		att.quat_reset_counter = 0;
		att.rollspeed = 0.1;
		att.timestamp = now;
		att.yawspeed = 0.1;
		_pub_att.update();
	}

	// publish estimator status
	if (true) {
		estimator_status_s &est = _pub_est.get();
		est.beta_test_ratio = 0;
		est.control_mode_flags = 0;

		for (int i = 0; i < 24; i++) {
			est.covariances[i] = 0;
		}

		est.filter_fault_flags = 0;
		est.gps_check_fail_flags = 0;
		est.hagl_test_ratio = 0;
		est.health_flags = 0;
		est.hgt_test_ratio = 0;
		est.innovation_check_flags = 0;
		est.mag_test_ratio = 0;
		est.nan_flags = 0;
		est.n_states = 10;
		est.pos_horiz_accuracy = 1;
		est.pos_test_ratio = 0;
		est.pos_vert_accuracy = 1;
		est.pre_flt_fail = false;
		est.solution_status_flags = 0;

		for (int i = 0; i < 24; i++) {
			est.states[i] = 0;
		}

		est.tas_test_ratio = 0;
		est.timeout_flags = 0;
		est.timestamp = now;
		est.time_slip = 0;
		est.vel_test_ratio = 0;

		for (int i = 0; i < 3; i++) {
			est.vibe[i] = 0;
		}

		_pub_est.update();
	}

	// innovations
	if (true) {
		ekf2_innovations_s &innov = _pub_innov.get();
		innov.airspeed_innov = 0;
		innov.airspeed_innov_var = 1;

		for (int i = 0; i < 2; i++) {
			innov.aux_vel_innov[i] = 0;
		}

		innov.beta_innov = 0;
		innov.beta_innov_var = 1;

		for (int i = 0; i < 2; i++) {
			innov.drag_innov[i] = 0;
			innov.drag_innov_var[i] = 0;
		}

		for (int i = 0; i < 2; i++) {
			innov.flow_innov[i] = 0;
			innov.flow_innov_var[i] = 1;
		}

		innov.hagl_innov = 0;
		innov.hagl_innov_var = 1;
		innov.heading_innov = 0;
		innov.heading_innov_var = 1;

		for (int i = 0; i < 3; i++) {
			innov.mag_innov[i] = 0;
			innov.mag_innov_var[i] = 1;
			innov.output_tracking_error[i] = 0;
		}

		innov.timestamp = 0;

		for (int i = 0; i < 6; i++) {
			innov.vel_pos_innov[i] = 0;
			innov.vel_pos_innov_var[i] = 1;
		}

		_pub_innov.update();
	}

	perf_end(_perf_elapsed);
}

void Cei::status()
{
	PX4_INFO("initialized: %d", _initialized);

	for (int i = 0; i < 6; i++) {
		PX4_INFO("x[%5d] = %10.4f", i, double(_x(i)));
	}

	for (int i = 0; i < 21; i++) {
		PX4_INFO("W[%5d] = %10.4f", i, double(_W(i)));
	}

	perf_print_counter(_perf_elapsed);
}
