#include "Cei.hpp"
#include "ekf/casadi_sekf.h"
#include <cstdlib>
#include <drivers/drv_hrt.h>
#include <string.h>
#include <assert.h>


Cei::Cei() :
	SuperBlock(nullptr, "CEI"),
	ModuleParams(nullptr),
	_perf_elapsed(),

    // casadi functions
    _correct(correct_functions()),
    _predict_covariance(P_dot_functions()),
    _predict_state(x_dot_functions()),
    _init_covariance(P_init_functions()),

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

	std::memset(_x, 0, sizeof _x);
	std::memset(_PU, 0, sizeof _PU);

    _init_covariance.res(0, _PU);
    _init_covariance.eval();

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
	_timeStamp =  now;

	// check for sane update rate
	if (dt > 0.1f || dt < 1e-3f) {
		PX4_WARN("update rate out of range: %12.5f", double(dt));
		return;
	}

	// set dt for all child blocks
	setDt(dt);

	if (_sub_sensor.updated()) {
		_sub_sensor.update();
	}

	if (_sub_param_update.updated()) {
		_sub_param_update.update();
	}

	// predict
	if (true) {
		{
			/* ekf_state_derivative:(x[6],u[3])->(dx[6]) */
			const float u[3] = {0, 0, 0};
			float dx[6] = {0};
			_predict_state.arg(0, _x);
			_predict_state.arg(1, u);
			_predict_state.res(0, dx);
			_predict_state.eval();

			if (array_finite(dx, 6)) {
				for (int i = 0; i < 6; i++) {
					_x[i] += dx[i] * dt;
				}

			} else {
				PX4_WARN("non finite state prediction");
			}
		}
		{
			const float u[3] = {0, 0, 0};
            float dPU[n_P] = {0};

            const float Q_diag[6] = {1, 1, 1, 1, 1, 1};
			_predict_covariance.arg(0, _x);
			_predict_covariance.arg(1, u);
			_predict_covariance.arg(2, _PU);
            _predict_covariance.arg(3, Q_diag);
			_predict_covariance.res(0, dPU);
			_predict_covariance.eval();

            if (array_finite(dPU, n_P)) {
                for (size_t i = 0; i < n_P; i++) {
					_PU[i] += dPU[i] * dt;
				}

			} else {
				PX4_WARN("non finite covariance prediction");
				PX4_WARN("x: %10.4f %10.4f %10.4f %10.4f %10.4f %10.4f", double(_x[0]), double(_x[1]), double(_x[2]), double(_x[3]),
					 double(_x[4]), double(_x[5]));
				PX4_WARN("u: %10.4f %10.4f %10.4f", double(u[0]), double(u[1]), double(u[2]));

                for (size_t i = 0; i < n_P; i++) {
					PX4_WARN("PU[%ld] = %10.4f", i, double(_PU[i]));
				}

                for (size_t i = 0; i < n_P; i++) {
					PX4_WARN("dPU[%ld] = %10.4f", i, double(dPU[i]));
				}
			}
		}
	}

	// correct mag
	if (_sub_mag.updated()) {
		const float y[3] = {1, 1, 1};
		float x1[6] = {0};
        float P1[n_P] = {0};
		const float sigma_v[3] = {1, 1, 1};
		_correct.arg(0, _x);
		_correct.arg(1, y);
		_correct.arg(2, _PU);
		_correct.arg(3, sigma_v);
		_correct.res(0, x1);
		_correct.res(1, P1);
		_correct.eval();
		bool correct = true;

        if (!array_finite(P1, n_P)) {
			PX4_WARN("non finite correction covariance");
			correct = false;
		}

		if (!array_finite(_x, 6)) {
			PX4_WARN("non finite correction state");
			correct = false;
		}

		if (correct) {
            for (size_t i = 0; i < n_P; i++) {
				_PU[i] = P1[i];
			}

            for (size_t i = 0; i < 6; i++) {
				_x[i] = x1[i];
			}

		} else {
			PX4_WARN("non finite correction");
		}

		//PX4_INFO("correct mag");
	}

	// publish local position
	if (_sub_mag.updated()) {
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
		lpos.vx = _x[X_vx];
		lpos.vxy_max = 0.1;
		lpos.vxy_reset_counter = 0;
		lpos.vy = _x[X_vy];
		lpos.vz = _x[X_vz];
		lpos.vz_reset_counter = 0;
		lpos.v_xy_valid = true;
		lpos.v_z_valid = true;
		lpos.x = _x[X_x];
		lpos.xy_global = true;
		lpos.xy_reset_counter = 0;
		lpos.xy_valid = true;
		lpos.y = _x[X_y];
		lpos.yaw = 0.1;
		lpos.z = _x[X_z];
		lpos.z_deriv = 0.1;
		lpos.z_global = true;
		lpos.z_reset_counter = 0;
		lpos.z_valid = true;
		_pub_lpos.update();
	}

	// publish vehicle_attitude
	if (_sub_mag.updated()) {
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
	if (_sub_mag.updated()) {
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
	if (_sub_mag.updated()) {
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

	_sub_mag.update();
	perf_end(_perf_elapsed);
}

void Cei::status()
{
	PX4_INFO("p   : %10.4f, %10.4f, %10.4f", x_d(X_x), x_d(X_y), x_d(X_z));
	PX4_INFO("p_v : %10.4f, %10.4f, %10.4f", var_d(X_x), var_d(X_y), var_d(X_z));
	PX4_INFO("v   : %10.4f, %10.4f, %10.4f", x_d(X_vx), x_d(X_vy), x_d(X_vz));
	PX4_INFO("v_v : %10.4f, %10.4f, %10.4f", var_d(X_vx), var_d(X_vy), var_d(X_vz)); \

    for (size_t i = 0; i < n_P; i++) {
		PX4_INFO("P[%5d] = %10.4f", i, double(_PU[i]));
	}

	perf_print_counter(_perf_elapsed);
}
