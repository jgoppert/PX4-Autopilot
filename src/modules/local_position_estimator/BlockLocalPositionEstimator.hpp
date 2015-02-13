#pragma once

#include <controllib/uorb/blocks.hpp>

// uORB Subscriptions
#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/optical_flow.h>
#include <uORB/topics/sensor_combined.h>
#include <drivers/drv_range_finder.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <mathlib/mathlib.h>

// uORB Publications
#include <uORB/Publication.hpp>
#include <uORB/topics/vehicle_local_position.h>

using namespace control;

class BlockLocalPositionEstimator : public control::SuperBlock {
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
//
// measurements:
//
// 	sonar: pz (TODO should account for roll/pitch)
//
// 	baro: pz (TODO should account for roll/pitch)
//
// 	flow: vx, vy (flow is in body x, y frame)
//
public:
	BlockLocalPositionEstimator();
	void update();
	virtual ~BlockLocalPositionEstimator();
private:
	// methods
	
	// predict the next state
	void predict();

	// update the state prediction wtih a measurement
	void update_flow();
	void update_baro();
	void update_lidar();
	void update_sonar();

	// subscriptions
	uORB::Subscription<vehicle_status_s> _status;
	uORB::Subscription<actuator_armed_s> _armed;
	uORB::Subscription<vehicle_control_mode_s> _control_mode;
	uORB::Subscription<vehicle_attitude_s> _att;
	uORB::Subscription<vehicle_attitude_setpoint_s> _att_sp;
	uORB::Subscription<optical_flow_s> _flow;
	uORB::Subscription<sensor_combined_s> _sensor;
	uORB::Subscription<range_finder_report> _range_finder;
	uORB::Subscription<parameter_update_s> _param_update;
	uORB::Subscription<manual_control_setpoint_s> _manual;

	// publications
	uORB::Publication<vehicle_local_position_s> _pos;

	// matrices for KF
	static const uint8_t n_X = 9;
	static const uint8_t n_U = 3; // 3 accelerations
	static const uint8_t n_Y_pos = 3;
	static const uint8_t n_Y_vel = 3;

	enum {X_px=0, X_py, X_pz, X_vx, X_vy, X_vz, X_bx, X_by, X_bz};
	enum {U_ax=0, U_ay, U_az};
	enum {Y_pos_px=0, Y_pos_py, Y_pos_pz};
	enum {Y_vel_vx=0, Y_vel_vy, Y_vel_vz};

	enum {CH_LEFT, CH_RIGHT};
	BlockPI th2v;
	BlockP q2v;
	struct pollfd _flowPoll;
	uint64_t _timeStamp;

	math::Matrix<n_X, n_X>  _A;
	math::Matrix<n_X, n_U>  _B;
	math::Matrix<n_Y_pos, n_X>  _C_pos; // position measurement
	math::Matrix<n_Y_pos, n_X>  _C_vel; // velocity measurement

	math::Vector<n_X>  _X;
	math::Vector<n_U>  _U;
	math::Vector<n_U>  _Y;
};

