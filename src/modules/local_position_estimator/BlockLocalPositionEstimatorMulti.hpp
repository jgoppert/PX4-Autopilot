#pragma once


#include <controllib/uorb/blocks.hpp>
#include <mathlib/mathlib.h>
#include <systemlib/perf_counter.h>
#include <lib/geo/geo.h>

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
#include <uORB/topics/vision_speed_estimate.h>
#include <uORB/topics/vehicle_vicon_position.h>

// uORB Publications
#include <uORB/Publication.hpp>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/filtered_bottom_flow.h>

#define CBRK_NO_VISION_KEY	328754

#include "ros/node_handle.hpp"

#include <px4.h>

using namespace control;


class BlockLocalPositionEstimatorMulti : public control::SuperBlock {
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
// 	vicon: px, py, pz
//
public:
	BlockLocalPositionEstimatorMulti();
	void update();
	virtual ~BlockLocalPositionEstimatorMulti();

private:
	// prevent copy and assignment
	BlockLocalPositionEstimatorMulti(const BlockLocalPositionEstimatorMulti &);
	BlockLocalPositionEstimatorMulti operator=(const BlockLocalPositionEstimatorMulti&);

	// constants
	static const uint8_t n_x = 6;
	static const uint8_t n_u = 3; // 3 accelerations
	static const uint8_t n_y_flow = 2;
	static const uint8_t n_y_sonar = 1;
	static const uint8_t n_y_baro = 1;
	static const uint8_t n_y_lidar = 1;
	static const uint8_t n_y_gps = 6;
	static const uint8_t n_y_vision_pos = 3;
	static const uint8_t n_y_vision_vel = 3;
	static const uint8_t n_y_vicon = 3;
	enum {X_x=0, X_y, X_z, X_vx, X_vy, X_vz}; //, X_bx, X_by, X_bz};
	enum {U_ax=0, U_ay, U_az};
	enum {Y_baro_z=0};
	enum {Y_lidar_z=0};
	enum {Y_flow_x=0, Y_flow_y};
	enum {Y_sonar_z=0};
	enum {Y_gps_x=0, Y_gps_y, Y_gps_z, Y_gps_vx, Y_gps_vy, Y_gps_vz};
	enum {Y_vision_x=0, Y_vision_y, Y_vision_z, Y_vision_vx, Y_vision_vy, Y_vision_vz};
	enum {Y_vicon_x=0, Y_vicon_y, Y_vicon_z};
	enum {POLL_FLOW, POLL_SENSORS, POLL_PARAM};

	// callback functions
	// ----------------------------
	void handleAttitude(const vehicle_attitude_s & msg);
	void handleOpticalFlow(const optical_flow_s & msg);
	void handleSensorCombined(const sensor_combined_s & msg);
	void handleDistanceSensor(const distance_sensor_s & msg);
	void handleParamUpdate(const parameter_update_s & msg);
	void handleHome(const home_position_s & msg);
	void handleGPS(const vehicle_gps_position_s & msg);
	void handleVisionPosition(const vision_position_estimate_s & msg);
	void handleVisionVelocity(const vision_speed_estimate_s & msg);
	void handleVicon(const vehicle_vicon_position_s & msg);

	// methods
	// ----------------------------

	// predict the next state
	void predict(bool canEstimateXY, bool canEstimateZ);

	// correct the state prediction wtih a measurement
	void correctBaro(const sensor_combined_s & msg);
	void correctGps(const vehicle_gps_position_s & msg);
	void correctLidar(const distance_sensor_s & msg);
	void correctFlow(const optical_flow_s & msg);
	void correctSonar(const distance_sensor_s & msg);
	void correctVisionPos(const vision_position_estimate_s & msg);
	void correctVisionVel(const vision_speed_estimate_s & msg);
	void correctVicon(const vehicle_vicon_position_s & msg);

	// sensor initialization
	void initBaro(const sensor_combined_s & msg);
	void initGps(const vehicle_gps_position_s & msg);
	void initLidar(const distance_sensor_s & msg);
	void initSonar(const distance_sensor_s & msg);
	void initFlow(const optical_flow_s & msg);
	void initVisionPos(const vision_position_estimate_s & msg);
	void initVisionVel(const vision_speed_estimate_s & msg);
	void initVicon(const vehicle_vicon_position_s & msg);

	// publications
	void publishLocalPos(bool z_valid, bool xy_valid);
	void publishGlobalPos(bool dead_reckoning);
	void publishFilteredFlow();
	
	// attributes
	// ----------------------------
	
	// test
	ros::NodeHandle _nh;

	ros::Subscriber _sub_att;
	ros::Subscriber _sub_flow;
	ros::Subscriber _sub_sensor;
	ros::Subscriber _sub_distance;
	ros::Subscriber _sub_param_update;
	ros::Subscriber _sub_home;
	ros::Subscriber _sub_gps;
	ros::Subscriber _sub_vision_pos;
	ros::Subscriber _sub_vision_vel;
	ros::Subscriber _sub_vicon;

	// publications
	ros::Publisher _pub_lpos;
	ros::Publisher _pub_gpos;
	ros::Publisher _pub_filtered_flow;

	// local message copies
	vehicle_attitude_s _att;
	home_position_s _home;
	vehicle_gps_position_s _gps;
	sensor_combined_s _sensor_combined;

	// map projection
	struct map_projection_reference_s _map_ref;

	// parameters
	BlockParamInt  _integrate;

	BlockParamFloat  _flow_xy_stddev;
	BlockParamFloat  _sonar_z_stddev;

	BlockParamFloat  _lidar_z_stddev;

	BlockParamFloat  _accel_xy_noise_power;
	BlockParamFloat  _accel_z_noise_power;

	BlockParamFloat  _baro_stddev;

	BlockParamFloat  _gps_xy_stddev;
	BlockParamFloat  _gps_z_stddev;

	BlockParamFloat  _gps_vxy_stddev;
	BlockParamFloat  _gps_vz_stddev;

	BlockParamFloat  _vision_xy_stddev;
	BlockParamFloat  _vision_z_stddev;
	BlockParamFloat  _vision_vxy_stddev;
	BlockParamFloat  _vision_vz_stddev;
	BlockParamInt    _no_vision;
	BlockParamFloat  _beta_max;

	BlockParamFloat  _vicon_p_stddev;

	// process noise
	BlockParamFloat  _pn_p_noise_power;
	BlockParamFloat  _pn_v_noise_power;

	// misc
	struct pollfd _polls[3];
	uint64_t _timeStamp;
	uint64_t _time_last_xy;
	uint64_t _time_last_flow;
	uint64_t _time_last_baro;
	uint64_t _time_last_gps;
	uint64_t _time_last_lidar;
	uint64_t _time_last_sonar;
	uint64_t _time_last_vision_p;
	uint64_t _time_last_vision_v;
	uint64_t _time_last_vicon;
	float 	 _altHome;
	int 	 _mavlink_fd;
	
	// initialization flags
	bool _baroInitialized;
	bool _gpsInitialized;
	bool _lidarInitialized;
	bool _sonarInitialized;
	bool _flowInitialized;
	bool _visionPosInitialized;
	bool _visionVelInitialized;
	bool _viconInitialized;

	// init counts
	int _baroInitCount;
	int _gpsInitCount;
	int _lidarInitCount;
	int _sonarInitCount;
	int _flowInitCount;
	int _visionPosInitCount;
	int _visionVelInitCount;
	int _viconInitCount;

	// reference altitudes
	float _baroAltHome;
	float _gpsAltHome;	
	float _lidarAltHome;
	float _sonarAltHome;
	float _flowAltHome;
	math::Vector<3> _visionHome;
	math::Vector<3> _visionBaseVel;
	math::Vector<3> _viconHome;

	// flow integration
	float _flowX;
	float _flowY;
	float _flowMeanQual;

	// referene lat/lon
	double _gpsLatHome;
	double _gpsLonHome;

	// sensor faults
	int _baroFault;
	int _gpsFault;
	int _lidarFault;
	int _flowFault;
	int _sonarFault;
	int _visionPosFault;
	int _visionVelFault;
	int _viconFault;
	
	bool _visionPosTimeout;
	bool _visionVelTimeout;
	bool _viconTimeout;

	perf_counter_t _loop_perf;
	perf_counter_t _interval_perf;
	perf_counter_t _err_perf;

	// state space
	math::Vector<n_x>  _x; // state vector
	math::Vector<n_u>  _u; // input vector
	math::Matrix<n_x, n_x>  _P; // state covariance matrix
};
