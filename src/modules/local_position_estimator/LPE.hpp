/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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
#include <mathlib/mathlib.h>
#include <systemlib/perf_counter.h>
#include <lib/geo/geo.h>

#ifdef USE_MATRIX_LIB
#include "matrix/Matrix.hpp"
using namespace matrix;
#else
#include <Eigen/Eigen>
using namespace Eigen;
#endif

#include "blocks/Sensor.hpp"
#include "blocks/Lidar.hpp"

// uORB Subscriptions
#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/optical_flow.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vision_position_estimate.h>
#include <uORB/topics/att_pos_mocap.h>

// uORB Publications
#include <uORB/Publication.hpp>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/filtered_bottom_flow.h>
#include <uORB/topics/estimator_status.h>

#include "constants.hpp"

#define CBRK_NO_VISION_KEY	328754

using namespace control;

class LPE : public control::SuperBlock
{
//
// The purpose of this estimator is to provide a robust solution for
// indoor flight.
//
// dynamics:
//
//	x(+) = A * x(-) + B * u(+)
//	y_i = C_i*x
//
// kalman filter
//
//	E[xx'] = P
//	E[uu'] = W
//	E[y_iy_i'] = R_i
//
//	prediction
//		x(+|-) = A*x(-|-) + B*u(+)
//		P(+|-) = A*P(-|-)*A' + B*W*B'
//
//	correction
//		x(+|+) =  x(+|-) + K_i * (y_i - H_i * x(+|-) )
//
//
// input:
// 	ax, ay, az (acceleration NED)
//
// states:
// 	px, py, pz , ( position NED)
// 	vx, vy, vz ( vel NED),
// 	bx, by, bz ( TODO accelerometer bias)
// 	tz (TODO terrain altitude)
//
// measurements:
//
// 	sonar: pz (measured d*cos(phi)*cos(theta))
//
// 	baro: pz
//
// 	flow: vx, vy (flow is in body x, y frame)
//
// 	gps: px, py, pz, vx, vy, vz (flow is in body x, y frame)
//
// 	lidar: px (actual measured d*cos(phi)*cos(theta))
//
// 	vision: px, py, pz, vx, vy, vz
//
// 	mocap: px, py, pz
//
public:

	Lidar _lidar;

	LPE();
	void update();
	virtual ~LPE();

private:
	// prevent copy and assignment
	LPE(const LPE &);
	LPE operator=(const LPE &);

	// methods
	// ----------------------------
	void initP();

	// predict the next state
	void predict();

	// correct the state prediction with a measurement
	void correctBaro();
	void correctGps();
	void correctLidar();
	void correctFlow();
	void correctSonar();
	void correctVision();
	void correctMocap();

	// sensor timeout checks
	void checkTimeouts();

	// sensor initialization
	void updateHome();
	void initBaro();
	void initGps();
	void initLidar();
	void initSonar();
	void initFlow();
	void initVision();
	void initMocap();

	// publications
	void publishLocalPos();
	void publishGlobalPos();
	void publishFilteredFlow();
	void publishEstimatorStatus();

	// attributes
	// ----------------------------

	// subscriptions
	uORB::Subscription<vehicle_status_s> _sub_status;
	uORB::Subscription<actuator_armed_s> _sub_armed;
	uORB::Subscription<vehicle_control_mode_s> _sub_control_mode;
	uORB::Subscription<vehicle_attitude_s> _sub_att;
	uORB::Subscription<vehicle_attitude_setpoint_s> _sub_att_sp;
	uORB::Subscription<optical_flow_s> _sub_flow;
	uORB::Subscription<sensor_combined_s> _sub_sensor;
	uORB::Subscription<parameter_update_s> _sub_param_update;
	uORB::Subscription<manual_control_setpoint_s> _sub_manual;
	uORB::Subscription<home_position_s> _sub_home;
	uORB::Subscription<vehicle_gps_position_s> _sub_gps;
	uORB::Subscription<vision_position_estimate_s> _sub_vision_pos;
	uORB::Subscription<att_pos_mocap_s> _sub_mocap;
	uORB::Subscription<distance_sensor_s> *_distance_subs[ORB_MULTI_MAX_INSTANCES];
	uORB::Subscription<distance_sensor_s> *_sub_lidar;
	uORB::Subscription<distance_sensor_s> *_sub_sonar;

	// publications
	uORB::Publication<vehicle_local_position_s> _pub_lpos;
	uORB::Publication<vehicle_global_position_s> _pub_gpos;
	uORB::Publication<filtered_bottom_flow_s> _pub_filtered_flow;
	uORB::Publication<estimator_status_s> _pub_est_status;

	// map projection
	struct map_projection_reference_s _map_ref;

	// parameters
	BlockParamInt  _integrate;

	BlockParamFloat  _flow_xy_stddev;
	BlockParamFloat  _sonar_z_stddev;

	BlockParamFloat  _lidar_z_stddev;

	BlockParamFloat  _accel_xy_stddev;
	BlockParamFloat  _accel_z_stddev;

	BlockParamFloat  _baro_stddev;

	BlockParamFloat  _gps_xy_stddev;
	BlockParamFloat  _gps_z_stddev;

	BlockParamFloat  _gps_vxy_stddev;
	BlockParamFloat  _gps_vz_stddev;

	BlockParamFloat  _gps_eph_max;

	BlockParamFloat  _vision_xy_stddev;
	BlockParamFloat  _vision_z_stddev;
	BlockParamInt    _no_vision;
	BlockParamFloat  _beta_max;

	BlockParamFloat  _mocap_p_stddev;

	BlockParamFloat  _flow_board_x_offs;
	BlockParamFloat  _flow_board_y_offs;

	BlockParamFloat  _flow_x_scaler;
	BlockParamFloat  _flow_y_scaler;
	BlockParamInt    _flow_min_q;

	// process noise
	BlockParamFloat  _pn_p_noise_power;
	BlockParamFloat  _pn_v_noise_power;
	BlockParamFloat  _pn_b_noise_power;

	// flow gyro
	BlockHighPass _flow_gyro_x_high_pass;
	BlockHighPass _flow_gyro_y_high_pass;

	// misc
	struct pollfd _polls[3];
	uint64_t _timeStamp;
	uint64_t _time_last_xy;
	uint64_t _time_last_z;
	uint64_t _time_last_flow;
	uint64_t _time_last_baro;
	uint64_t _time_last_gps;
	uint64_t _time_last_lidar;
	uint64_t _time_last_sonar;
	uint64_t _time_last_vision_p;
	uint64_t _time_last_mocap;
	int 	 _mavlink_fd;

	// initialization flags
	bool _baroInitialized;
	bool _gpsInitialized;
	bool _lidarInitialized;
	bool _sonarInitialized;
	bool _flowInitialized;
	bool _visionInitialized;
	bool _mocapInitialized;

	// init counts
	int _baroInitCount;
	int _gpsInitCount;
	int _lidarInitCount;
	int _sonarInitCount;
	int _flowInitCount;
	int _visionInitCount;
	int _mocapInitCount;

	// reference altitudes
	float _altHome;
	bool _altHomeInitialized;
	float _baroAltHome;
	float _gpsAltHome;
	float _lidarAltHome;
	float _sonarAltHome;
	float _flowAltHome;
	Vector3f _visionHome;
	Vector3f _mocapHome;

	// flow integration
	float _flowX;
	float _flowY;
	float _flowMeanQual;

	// referene lat/lon
	double _gpsLatHome;
	double _gpsLonHome;

	// status
	bool _canEstimateXY;
	bool _canEstimateZ;
	bool _xyTimeout;
	bool _zTimeout;

	// sensor faults
	fault_t _baroFault;
	fault_t _gpsFault;
	fault_t _lidarFault;
	fault_t _flowFault;
	fault_t _sonarFault;
	fault_t _visionFault;
	fault_t _mocapFault;

	// timeouts
	bool _baroTimeout;
	bool _gpsTimeout;
	bool _flowTimeout;
	bool _lidarTimeout;
	bool _sonarTimeout;
	bool _visionTimeout;
	bool _mocapTimeout;

	// performance counters
	perf_counter_t _loop_perf;
	perf_counter_t _interval_perf;
	perf_counter_t _err_perf;

	// state space
	Vector<float, n_x>  _x; // state vector
	Vector<float, n_u>  _u; // input vector
	Matrix<float, n_x, n_x>  _P; // state covariance matrix
};
