#include "BlockLocalPositionEstimator.hpp"
#include <mavlink/mavlink_log.h>
#include <fcntl.h>
#include <nuttx/math.h>
#include <systemlib/err.h>

static const int 		MIN_FLOW_QUALITY = 100;
static const int 		REQ_INIT_COUNT = 100;

static const uint32_t 		VISION_POSITION_TIMEOUT = 500000;
static const uint32_t 		VISION_VELOCITY_TIMEOUT = 500000;
static const uint32_t 		VICON_TIMEOUT = 200000;	 

static const uint32_t 		XY_SRC_TIMEOUT = 2000000;

BlockLocalPositionEstimator::BlockLocalPositionEstimator() :
	// this block has no parent, and has name LPE
	SuperBlock(NULL,"LPE"),

	// subscriptions, set rate, add to list
	// TODO topic speed limiting?
	_sub_status(ORB_ID(vehicle_status), 0, 0, &getSubscriptions()),
	_sub_armed(ORB_ID(actuator_armed), 0, 0, &getSubscriptions()),
	_sub_control_mode(ORB_ID(vehicle_control_mode),
			0, 0, &getSubscriptions()),
	_sub_att(ORB_ID(vehicle_attitude), 0, 0, &getSubscriptions()),
	_sub_att_sp(ORB_ID(vehicle_attitude_setpoint),
			0, 0, &getSubscriptions()),
	_sub_flow(ORB_ID(optical_flow), 0, 0, &getSubscriptions()),
	_sub_sensor(ORB_ID(sensor_combined), 0, 0, &getSubscriptions()),
	_sub_distance(ORB_ID(distance_sensor),
			0, 0, &getSubscriptions()),
	_sub_param_update(ORB_ID(parameter_update), 0, 0, &getSubscriptions()),
	_sub_manual(ORB_ID(manual_control_setpoint), 0, 0, &getSubscriptions()),
	_sub_home(ORB_ID(home_position), 0, 0, &getSubscriptions()),
	_sub_gps(ORB_ID(vehicle_gps_position), 0, 0, &getSubscriptions()),
	_sub_vision_pos(ORB_ID(vision_position_estimate), 0, 0, &getSubscriptions()),
	_sub_vision_vel(ORB_ID(vision_speed_estimate), 0, 0, &getSubscriptions()),
	_sub_vicon(ORB_ID(vehicle_vicon_position), 0, 0, &getSubscriptions()),

	// publications
	_pub_lpos(ORB_ID(vehicle_local_position), -1, &getPublications()),
	_pub_gpos(ORB_ID(vehicle_global_position), -1, &getPublications()),
	_pub_filtered_flow(ORB_ID(filtered_bottom_flow), -1, &getPublications()),

	// map projection
	_map_ref(),

	// block parameters
	_integrate(this, "INTEGRATE"),
	_flow_xy_stddev(this, "FLW_XY"),
	_sonar_z_stddev(this, "SNR_Z"),
	_lidar_z_stddev(this, "LDR_Z"),
	_accel_xy_stddev(this, "ACC_XY"),
	_accel_z_stddev(this, "ACC_Z"),
	_baro_stddev(this, "BAR_Z"),
	_gps_xy_stddev(this, "GPS_XY"),
	_gps_z_stddev(this, "GPS_Z"),
	_gps_vxy_stddev(this, "GPS_VXY"),
	_gps_vz_stddev(this, "GPS_VZ"),
	_vision_p_stddev(this, "VIS_P"),
	_vision_v_stddev(this, "VIS_V"),
	_vicon_p_stddev(this, "VIC_P"),
	_pn_p_stddev(this, "PN_P"),
	_pn_v_stddev(this, "PN_V"),

	// misc
	_polls(),
	_timeStamp(hrt_absolute_time()),
	_time_last_xy(0),
	_time_last_flow(0),
	_time_last_baro(0),
	_time_last_gps(0),
	_time_last_lidar(0),
	_time_last_sonar(0),
	_time_last_vision_p(0),
	_time_last_vision_v(0),	
	_time_last_vicon(0),
	_altHome(0),

	// mavlink log
	_mavlink_fd(open(MAVLINK_LOG_DEVICE, 0)),

	// initialization flags
	_baroInitialized(false),
	_gpsInitialized(false),
	_lidarInitialized(false),
	_sonarInitialized(false),
	_flowInitialized(false),
	_visionPosInitialized(false),
	_visionVelInitialized(false),
	_viconInitialized(false),

	// init counts
	_baroInitCount(0),
	_gpsInitCount(0),
	_lidarInitCount(0),
	_sonarInitCount(0),
	_flowInitCount(0),
	_visionPosInitCount(0),
	_visionVelInitCount(0),
	_viconInitCount(0),

	// reference altitudes
	_baroAltHome(0),
	_gpsAltHome(0),
	_lidarAltHome(0),
	_sonarAltHome(0),
	_visionHome(),
	_visionBaseVel(),
	_viconHome(),

	// flow integration
	_flowX(0),
	_flowY(0),
	_flowMeanQual(0),

	// reference lat/lon
	_gpsLatHome(0),
	_gpsLonHome(0),

	// faults
	_baroFault(0),
	_gpsFault(0),
	_lidarFault(0),
	_flowFault(0),
	_sonarFault(0),
	_visionFault(0),
	_viconFault(0),

	//timeouts
	_visionPosTimeout(true),
	_visionVelTimeout(true),
	_viconTimeout(true),

	// loop performance
	_loop_perf(),
	_interval_perf(),
	_err_perf(),

	// kf matrices
	_x(), _u(), _P()
{
	// setup event triggering based on new flow messages to integrate
	_polls[POLL_FLOW].fd = _sub_flow.getHandle();
	_polls[POLL_FLOW].events = POLLIN;

	_polls[POLL_PARAM].fd = _sub_param_update.getHandle();
	_polls[POLL_PARAM].events = POLLIN;

	_polls[POLL_SENSORS].fd = _sub_sensor.getHandle();
	_polls[POLL_SENSORS].events = POLLIN;

	// initialize P to identity*0.1
	_P.identity();
	_P *= 0.1;

	// perf counters
	_loop_perf = perf_alloc(PC_ELAPSED,
			"local_position_estimator_runtime");
	_interval_perf = perf_alloc(PC_INTERVAL,
			"local_position_estimator_interval");
	_err_perf = perf_alloc(PC_COUNT, "local_position_estimator_err");

	// map
	_map_ref.init_done = false;

	// intialize parameter dependent matrices
	updateParams();
}

BlockLocalPositionEstimator::~BlockLocalPositionEstimator() {
}

void BlockLocalPositionEstimator::update() {

	// wait for a sensor update, check for exit condition every 100 ms
	int ret = poll(_polls, 2, 100);
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
	bool flowUpdated = _sub_flow.updated();
	bool paramsUpdated = _sub_param_update.updated();
	bool baroUpdated = _sub_sensor.updated();
	bool lidarUpdated = false;
	bool sonarUpdated = false;
	if (_sub_distance.updated()) {
		if (_sub_distance.get().type == distance_sensor_s::MAV_DISTANCE_SENSOR_LASER) {
			lidarUpdated = true;		
		}
		if (_sub_distance.get().type == distance_sensor_s::MAV_DISTANCE_SENSOR_ULTRASOUND) {
			sonarUpdated = true;		
		}
	}
	bool gpsUpdated = _sub_gps.updated();
	bool homeUpdated = _sub_home.updated();
	bool visionPosUpdated = _sub_vision_pos.updated();
	bool visionVelUpdated = _sub_vision_vel.updated();
	bool viconUpdated = _sub_vicon.updated();

	// get new data
	updateSubscriptions();

	// update parameters
	if (paramsUpdated) updateParams();

	// update home position projection
	if (homeUpdated) updateHome();

	// check for timeouts on external sources
	if((hrt_absolute_time() - _time_last_vision_p > VISION_POSITION_TIMEOUT) && _visionPosInitialized)
	{
		if(!_visionPosTimeout)
		{
			_visionPosTimeout = true;
			mavlink_log_info(_mavlink_fd, "[lpe] vision position timeout ");
			warnx("[lpe] vision position timeout ");
		}
	} else {
		_visionPosTimeout = false;
	}

	if((hrt_absolute_time() - _time_last_vision_v > VISION_VELOCITY_TIMEOUT) && _visionVelInitialized)
	{
		if(!_visionVelTimeout)
		{
			_visionVelTimeout = true;
			mavlink_log_info(_mavlink_fd, "[lpe] vision velocity timeout ");
			warnx("[lpe] vision velocity timeout ");
		}
	} else {
		_visionVelTimeout = false;
	}

	if((hrt_absolute_time() -_time_last_vicon > VICON_TIMEOUT) && _viconInitialized)
	{
		if(!_viconTimeout)
		{
			_viconTimeout = true;
			mavlink_log_info(_mavlink_fd, "[lpe] vicon timeout ");
			warnx("[lpe] vicon timeout ");
		}
	} else {
		_viconTimeout = false;
	}

	// determine if we should start estimating
	bool canEstimateZ = 
		_baroInitialized;	
	bool canEstimateXY = 
		(_gpsInitialized && !_gpsFault) ||
 		(_flowInitialized && !_flowFault) ||
 		(_visionPosInitialized && !_visionPosTimeout && !_visionFault) ||
 		(_viconInitialized && !_viconTimeout && !_viconFault);

	if(canEstimateXY) {
		_time_last_xy = hrt_absolute_time();	
	}
		
	// if we have no lat, lon initialized projection at 0,0
	if (canEstimateXY && !_map_ref.init_done) {
		map_projection_init(&_map_ref, 0, 0);
	}

	// do prediction if we have a reasonable set of
	// initialized sensors
	if (canEstimateZ) {
		predict();
	}

	// sensor corrections/ initializations
	if (gpsUpdated) {
		if (!_gpsInitialized) {
			initGps();
		} else {
			correctGps();
		}
	}
	if (baroUpdated) {
		if (!_baroInitialized) {
			initBaro();
		} else {
			correctBaro();
		}
	}
	if (lidarUpdated) {
		if (!_lidarInitialized) {
			initLidar();
		} else {
			correctLidar();
		}
	}
	if (sonarUpdated) {
		if (!_sonarInitialized) {
			initSonar();
		} else {
			correctSonar();
		}
	}
	if (flowUpdated) {
		if (!_flowInitialized) {
			initFlow();
		} else {
			perf_begin(_loop_perf);// TODO
			correctFlow();	
			perf_count(_interval_perf);
			perf_end(_loop_perf);
		}
	}
	if (visionPosUpdated) {
		if (!_visionPosInitialized) {
			initVisionPos();
		}
		else if (visionVelUpdated && !_visionVelInitialized) {
			initVisionVel();
		}
		else {
			correctVision();
		}
	}
	if (viconUpdated) {
		if (!_viconInitialized) {
			initVicon();
		} else {
			correctVicon();
		}
	}

	if (!(hrt_absolute_time() - _time_last_xy > XY_SRC_TIMEOUT)) {
		// update all publications if possible
		publishLocalPos(true, true);
		publishGlobalPos();
		publishFilteredFlow();
	}
	else {
		// publish only Z estimate
		publishLocalPos(true, false);
	}
}

void BlockLocalPositionEstimator::updateHome() {
	double lat = _sub_home.get().lat;
	double lon = _sub_home.get().lon;
	float alt = _sub_home.get().alt;
	mavlink_log_info(_mavlink_fd, "[lpe] home: lat %5.0f, lon %5.0f, alt %5.0f", lat, lon, double(alt));
	warnx("[lpe] home: lat %5.0f, lon %5.0f, alt %5.0f", lat, lon, double(alt));
	map_projection_init(&_map_ref, lat, lon);
	float delta_alt = alt - _altHome;
	_altHome = alt;
	_gpsAltHome += delta_alt;
	_baroAltHome +=  delta_alt;
	_lidarAltHome +=  delta_alt;
	_sonarAltHome +=  delta_alt;
}

void BlockLocalPositionEstimator::initBaro() {
	// collect baro data
	if (!_baroInitialized &&
		(_sub_sensor.get().baro_timestamp != _time_last_baro)) {
		_time_last_baro = _sub_sensor.get().baro_timestamp;
		_baroAltHome += _sub_sensor.get().baro_alt_meter;
		if (_baroInitCount++ > REQ_INIT_COUNT) {
			_baroAltHome /= _baroInitCount;
			mavlink_log_info(_mavlink_fd,
				"[lpe] baro offs: %d m", (int)_baroAltHome);
			warnx("[lpe] baro offs: %d m", (int)_baroAltHome);
			_baroInitialized = true;
		}
	}
}


void BlockLocalPositionEstimator::initGps() {
	// collect gps data
	if (!_gpsInitialized && _sub_gps.get().fix_type > 2) {
		double lat = _sub_gps.get().lat*1e-7;
		double lon = _sub_gps.get().lon*1e-7;
		float alt = _sub_gps.get().alt*1e-3f;
		// increament sums for mean
		_gpsLatHome += lat;
		_gpsLonHome += lon;
		_gpsAltHome += alt;
		_time_last_gps = _sub_gps.get().timestamp_position;
		if (_gpsInitCount++ > REQ_INIT_COUNT) {
			_gpsLatHome /= _gpsInitCount;
			_gpsLonHome /= _gpsInitCount;
			_gpsAltHome /= _gpsInitCount;
			map_projection_init(&_map_ref, lat, lon);
			mavlink_log_info(_mavlink_fd, "[lpe] gps init: "
					"lat %d, lon %d, alt %d m",
					int(_gpsLatHome), int(_gpsLonHome), int(_gpsAltHome));
			warnx("[lpe] gps init: lat %d, lon %d, alt %d m",
					int(_gpsLatHome), int(_gpsLonHome), int(_gpsAltHome));
			_gpsInitialized = true;
		}
	}
}

void BlockLocalPositionEstimator::initLidar() {
	
	if(_sub_distance.get().type != distance_sensor_s::MAV_DISTANCE_SENSOR_LASER) return;
	
	// collect lidar data
	bool valid = false;
	float d = _sub_distance.get().current_distance;
	if (d < _sub_distance.get().max_distance &&
		d > _sub_distance.get().min_distance) {
		valid = true;
	}
	if (!_lidarInitialized && valid) {
		// increament sums for mean
		_lidarAltHome += _sub_distance.get().current_distance;
		if (_lidarInitCount++ > REQ_INIT_COUNT) {
			_lidarAltHome /= _lidarInitCount;
			mavlink_log_info(_mavlink_fd, "[lpe] lidar init: "
					"alt %d cm",
					int(100*_lidarAltHome));
			warnx("[lpe] lidar init: alt %d cm",
					int(100*_lidarAltHome));
			_lidarInitialized = true;
		}
	}
}

void BlockLocalPositionEstimator::initSonar() {

	if(_sub_distance.get().type != distance_sensor_s::MAV_DISTANCE_SENSOR_ULTRASOUND) return;

	// collect sonar data
	bool valid = false;
	float d = _sub_distance.get().current_distance;
	if (d < _sub_distance.get().max_distance &&
		d > _sub_distance.get().min_distance) {
		valid = true;
	}
	if (!_sonarInitialized && valid) {
		// increament sums for mean
		_sonarAltHome += _sub_distance.get().current_distance;
		if (_sonarInitCount++ > REQ_INIT_COUNT) {
			_sonarAltHome /= _sonarInitCount;
			mavlink_log_info(_mavlink_fd, "[lpe] sonar init: "
					"alt %d cm",
					int(100*_sonarAltHome));
			warnx("[lpe] sonar init: alt %d cm",
					int(100*_sonarAltHome));
			_sonarInitialized = true;
		}
	}
}

void BlockLocalPositionEstimator::initFlow() {
	
	// collect pixel flow data 
	if (!_flowInitialized) {
		// increament sums for mean
		_flowMeanQual += _sub_flow.get().quality;

		if (_flowInitCount++ > REQ_INIT_COUNT) {
			_flowMeanQual /= _flowInitCount;
			if(_flowMeanQual < MIN_FLOW_QUALITY)	
			{
				// retry initialisation till we have better flow data
				warnx("[lpe] flow quality bad, retrying init:%d",
					int(_flowMeanQual));
				_flowMeanQual = 0;
				_flowInitCount = 0;
				return;
			}
			mavlink_log_info(_mavlink_fd, "[lpe] flow init: "
					"quality %d",
					int(_flowMeanQual));
			warnx("[lpe] flow init: quality %d",
					int(_flowMeanQual));
			_flowInitialized = true;
		}
	}
}

void BlockLocalPositionEstimator::initVisionPos() {
	// collect vision position data
	if (!_visionPosInitialized) {
		// increament sums for mean
		math::Vector<3> pos;
		pos(0) = _sub_vision_pos.get().x;
		pos(1) = _sub_vision_pos.get().y;
		pos(2) = _sub_vision_pos.get().z;
		_visionHome += pos;
		if (_visionPosInitCount++ > REQ_INIT_COUNT) {
			_visionHome /= _visionPosInitCount;
			mavlink_log_info(_mavlink_fd, "[lpe] vision position init: "
					"%f, %f, %f m", double(pos(0)), double(pos(1)), double(pos(2)));
			warnx("[lpe] vision position init: "
					"%f, %f, %f m", double(pos(0)), double(pos(1)), double(pos(2)));
			_visionPosInitialized = true;
		}
	}
}

void BlockLocalPositionEstimator::initVisionVel() {
	// collect vision velocity data
	if (!_visionVelInitialized) {
		// increament sums for mean
		math::Vector<3> vel;
		vel(0) = _sub_vision_vel.get().x;
		vel(1) = _sub_vision_vel.get().y;
		vel(2) = _sub_vision_vel.get().z;
		_visionBaseVel += vel;
		if (_visionVelInitCount++ > REQ_INIT_COUNT) {
			_visionBaseVel /= _visionVelInitCount;
			mavlink_log_info(_mavlink_fd, "[lpe] vision velocity init: "
					"%f, %f, %f m/s", double(vel(0)), double(vel(1)), double(vel(2)));
			warnx("[lpe] vision velocity init: "
					"%f, %f, %f m/s", double(vel(0)), double(vel(1)), double(vel(2)));
			_visionVelInitialized = true;
		}
	}
}

void BlockLocalPositionEstimator::initVicon() {
	// collect vicon data
	if (!_viconInitialized) {
		// increament sums for mean
		math::Vector<3> pos;
		pos(0) = _sub_vicon.get().x;
		pos(1) = _sub_vicon.get().y;
		pos(2) = _sub_vicon.get().z;
		_viconHome += pos;
		if (_viconInitCount++ > REQ_INIT_COUNT) {
			_viconHome /= _viconInitCount;
			mavlink_log_info(_mavlink_fd, "[lpe] vicon init: "
					"%f, %f, %f m", double(pos(0)), double(pos(1)), double(pos(2)));
			warnx("[lpe] vicon init: "
					"%f, %f, %f m", double(pos(0)), double(pos(1)), double(pos(2)));
			_viconInitialized = true;
		}
	}
}

void BlockLocalPositionEstimator::publishLocalPos(bool z_valid, bool xy_valid) {
	// publish local position
	if (isfinite(_x(X_x)) && isfinite(_x(X_y)) && isfinite(_x(X_z)) &&
		isfinite(_x(X_vx)) && isfinite(_x(X_vy))
		&& isfinite(_x(X_vz))) {
		_pub_lpos.get().timestamp = _timeStamp;
		_pub_lpos.get().xy_valid = xy_valid;
		_pub_lpos.get().z_valid = z_valid;
		_pub_lpos.get().v_xy_valid = xy_valid;
		_pub_lpos.get().v_z_valid = z_valid;
		_pub_lpos.get().x = _x(X_x); 	// north
		_pub_lpos.get().y = _x(X_y);  	// east
		_pub_lpos.get().z = _x(X_z); 	// down
		_pub_lpos.get().vx = _x(X_vx);  // north
		_pub_lpos.get().vy = _x(X_vy);  // east
		_pub_lpos.get().vz = _x(X_vz); 	// down
		_pub_lpos.get().yaw = _sub_att.get().yaw;
		_pub_lpos.get().xy_global = _sub_home.get().timestamp != 0; // need home for reference
		_pub_lpos.get().z_global = _baroInitialized;
		_pub_lpos.get().ref_timestamp = _sub_home.get().timestamp;
		_pub_lpos.get().ref_lat = _map_ref.lat_rad*180/M_PI;
		_pub_lpos.get().ref_lon = _map_ref.lon_rad*180/M_PI;
		_pub_lpos.get().ref_alt = _sub_home.get().alt;
		// TODO, terrain alt
		_pub_lpos.get().dist_bottom = -_x(X_z);
		_pub_lpos.get().dist_bottom_rate = -_x(X_vz);
		_pub_lpos.get().surface_bottom_timestamp = 0;
		_pub_lpos.get().dist_bottom_valid = true;
		_pub_lpos.get().eph = sqrtf(_P(X_x, X_x) + _P(X_y, X_y));
		_pub_lpos.get().epv = sqrtf(_P(X_z, X_z));
		_pub_lpos.update();
	}
}

void BlockLocalPositionEstimator::publishGlobalPos() {
	// publish global position
	double lat = 0;
	double lon = 0;
	map_projection_reproject(&_map_ref, _x(X_x), _x(X_y), &lat, &lon);
	float alt = -_x(X_z) + _sub_home.get().alt;
	if(isfinite(lat) && isfinite(lon) && isfinite(alt) &&
			isfinite(_x(X_vx)) && isfinite(_x(X_vy)) &&
			isfinite(_x(X_vz))) {
		_pub_gpos.get().timestamp = _timeStamp;
		_pub_gpos.get().time_utc_usec = _sub_gps.get().time_utc_usec;
		_pub_gpos.get().lat = lat;
		_pub_gpos.get().lon = lon;
		_pub_gpos.get().alt = alt;
		_pub_gpos.get().vel_n = _x(X_vx);
		_pub_gpos.get().vel_e = _x(X_vy);
		_pub_gpos.get().vel_d = _x(X_vz);
		_pub_gpos.get().yaw = _sub_att.get().yaw;
		_pub_gpos.get().eph = sqrtf(_P(X_x, X_x) + _P(X_y, X_y));
		_pub_gpos.get().epv = sqrtf(_P(X_z, X_z));
		_pub_gpos.get().terrain_alt = 0;
		_pub_gpos.get().terrain_alt_valid = false;
		if (_timeStamp - _time_last_gps < 1) {
			_pub_gpos.get().dead_reckoning = false;
		} else {
			_pub_gpos.get().dead_reckoning = true;
		}
		_pub_gpos.update();
	}
}

void BlockLocalPositionEstimator::publishFilteredFlow() {
	// publish filtered flow
	if(isfinite(_pub_filtered_flow.get().sumx) &&
		isfinite(_pub_filtered_flow.get().sumy) &&
		isfinite(_pub_filtered_flow.get().vx) &&
		isfinite(_pub_filtered_flow.get().vy)) {
		_pub_filtered_flow.update();
	}
}

void BlockLocalPositionEstimator::predict() {
	if (_integrate.get() && _sub_att.get().R_valid) {
		math::Matrix<3,3> R_att(_sub_att.get().R);
		math::Vector<3> a(_sub_sensor.get().accelerometer_m_s2);
		_u = R_att*a;
		_u(2) += 9.81f; // add g
	} else {
		_u = math::Vector<3>({0,0,0});
	}

	// dynamics matrix
	math::Matrix<n_x, n_x>  A; // state dynamics matrix
	// derivative of position is velocity
	A(X_x, X_vx) = 1;
	A(X_y, X_vy) = 1;
	A(X_z, X_vz) = 1;
	// derivative of velocity is accelerometer
	// 	bias + acceleration
	//_A(X_vx, X_bx) = 1;
	//_A(X_vy, X_by) = 1;
	//_A(X_vz, X_bz) = 1;

	// input matrix
	math::Matrix<n_x, n_u>  B; // input matrix
	B(X_vx, U_ax) = 1;
	B(X_vy, U_ay) = 1;
	B(X_vz, U_az) = 1;

	// input noise
	math::Matrix<n_u, n_u> R;
	R(U_ax, U_ax) =
		_accel_xy_stddev.get()*_accel_xy_stddev.get();
	R(U_ay, U_ay) =
		_accel_xy_stddev.get()*_accel_xy_stddev.get();
	R(U_az, U_az) =
		_accel_z_stddev.get()*_accel_z_stddev.get();

	// process noise matrix
	math::Matrix<n_x, n_x>  Q; // process noise
	float pn_p_sq = _pn_p_stddev.get()*_pn_p_stddev.get();
	float pn_v_sq = _pn_v_stddev.get()*_pn_v_stddev.get();
	Q(X_x, X_x) = pn_p_sq;
	Q(X_y, X_y) = pn_p_sq;
	Q(X_z, X_z) = pn_p_sq;
	Q(X_vx, X_vx) = pn_v_sq;
	Q(X_vy, X_vy) = pn_v_sq;
	Q(X_vz, X_vz) = pn_v_sq;

	// continuous time kalman filter prediction
	_x += (A*_x + B*_u)*getDt();
	_P += (A*_P + _P*A.transposed() +
		B*R*B.transposed() + Q)*getDt();
}

void BlockLocalPositionEstimator::correctFlow() {	// TODO : use another other metric for glitch detection

	// flow measurement matrix and noise matrix
	math::Matrix<n_y_flow, n_x> C;
	C(Y_flow_x, X_x) = 1;
	C(Y_flow_y, X_y) = 1;

	math::Matrix<n_y_flow, n_y_flow> R;
	R(Y_flow_x, Y_flow_x) =
		_flow_xy_stddev.get()*_flow_xy_stddev.get();
	R(Y_flow_y, Y_flow_y) =
		_flow_xy_stddev.get()*_flow_xy_stddev.get();

	float flow_speed[3] = {0.0f, 0.0f, 0.0f};
	float global_speed[3] = {0.0f, 0.0f, 0.0f};

	/* calc dt between flow timestamps */
	/* ignore first flow msg */
	if (_time_last_flow == 0) {
		_time_last_flow = _sub_flow.get().timestamp;
		return;
	}
	float dt = (_sub_flow.get().timestamp - _time_last_flow) * 1.0e-6f ;
	_time_last_flow = _sub_flow.get().timestamp;
	
	// calculate velocity over ground
	if (_sub_flow.get().integration_timespan > 0) {
		flow_speed[0] = _sub_flow.get().pixel_flow_x_integral /
			(_sub_flow.get().integration_timespan / 1e6f) *
			_x(X_z);
		flow_speed[1] = _sub_flow.get().pixel_flow_y_integral /
			(_sub_flow.get().integration_timespan / 1e6f) *
			_x(X_z);
	} else {
		flow_speed[0] = 0;
		flow_speed[1] = 0;
	}
	flow_speed[2] = 0.0f;

	/* update filtered flow */
	_pub_filtered_flow.get().sumx += flow_speed[0] * dt;
	_pub_filtered_flow.get().sumy += flow_speed[1] * dt;
	_pub_filtered_flow.get().vx = flow_speed[0];
	_pub_filtered_flow.get().vy = flow_speed[1];

	// TODO add yaw rotation correction (with distance to vehicle zero)

	// convert to globalframe velocity
	for(uint8_t i = 0; i < 3; i++) {
		float sum = 0.0f;
		for(uint8_t j = 0; j < 3; j++) {
			sum += flow_speed[j] * PX4_R(_sub_att.get().R, i, j);
		}
		global_speed[i] = sum;
	}

	// flow integral
	_flowX += global_speed[0]*dt;
	_flowY += global_speed[1]*dt;

	// measurement
	math::Vector<2> y;
	y(0) = _flowX;
	y(1) = _flowY;

	// residual
	math::Vector<2> r = y - C*_x;

	// residual covariance, (inversed)
	math::Matrix<n_y_flow, n_y_flow> S_I =
		(C*_P*C.transposed() + R).inversed();

	// fault detection
	float beta = sqrtf(r*(S_I*r));

	// 3 std devations away
	if (beta > 3) {
		if (!_flowFault) {
			mavlink_log_info(_mavlink_fd, "[lpe] flow fault,  beta %5.2f", double(beta));
			warnx("[lpe] flow fault,  beta %5.2f", double(beta));
		}
		_flowFault = 1;
	// turn off if fault ok
	} else if (_flowFault) {
		_flowFault = 0;
		mavlink_log_info(_mavlink_fd, "[lpe] flow OK");
		warnx("[lpe] flow OK");
	}

	// kalman filter correction if no fault
	if (_flowFault < 1) {
		math::Matrix<n_x, n_y_flow> K =
			_P*C.transposed()*S_I;
		_x += K*r;
		_P -= K*C*_P;
	// reset flow integral to current estimate of position
	// if a fault occurred
	} else {
		_flowX = _x(X_x);
		_flowY = _x(X_y);	
	}

}

void BlockLocalPositionEstimator::correctSonar() {
	
	if(_sub_distance.get().type != distance_sensor_s::MAV_DISTANCE_SENSOR_ULTRASOUND) return;
	
	float d = _sub_distance.get().current_distance;
	if (d < _sub_distance.get().min_distance ||
			d > _sub_distance.get().max_distance) {
		mavlink_log_info(_mavlink_fd, "[lpe] sonar out of range");
		warnx("[lpe] sonar out of range");
		return;
	}

	// sonar measurement matrix and noise matrix
	math::Matrix<n_y_sonar, n_x> C;
	C(Y_sonar_z, X_z) = -1;

	// use parameter covariance unless sensor provides reasonable value
	math::Matrix<n_y_sonar, n_y_sonar> R;
	float cov = _sub_distance.get().covariance;
	if (cov < 1.0e-3f) {
		R(0,0) = _sonar_z_stddev.get()*_sonar_z_stddev.get();
	} else {
		R(0,0) = cov;
	}

	// measurement
	math::Vector<1> y;
	y(0) = (d - _sonarAltHome)*
		cosf(_sub_att.get().roll)*
		cosf(_sub_att.get().pitch);

	// residual
	math::Vector<1> r = y - C*_x;

	// residual covariance, (inversed)
	math::Matrix<n_y_sonar, n_y_sonar> S_I =
		(C*_P*C.transposed() + R).inversed();

	// fault detection
	float beta = sqrtf(r*(S_I*r));

	// zero is an error code for the sonar
	if (d < 0.001f) {
		if (!_sonarFault) {
			mavlink_log_info(_mavlink_fd, "[lpe] sonar error");
			warnx("[lpe] sonar error");
		}
		_sonarFault = 2;
	// 3 std devations away
	} else if (beta > 3) {
		if (!_sonarFault) {
			mavlink_log_info(_mavlink_fd, "[lpe] sonar fault,  beta %5.2f", double(beta));
			warnx("[lpe] sonar fault,  beta %5.2f", double(beta));
		}
		_sonarFault = 1;
	// turn off if fault ok
	} else if (_sonarFault) {
		_sonarFault = 0;
		mavlink_log_info(_mavlink_fd, "[lpe] sonar OK");
		warnx("[lpe] sonar OK");
	}

	// kalman filter correction if no fault
	if (_sonarFault < 1) {
		math::Matrix<n_x, n_y_sonar> K =
			_P*C.transposed()*S_I;
		_x += K*r;
		_P -= K*C*_P;
	}
	_time_last_sonar = _sub_distance.get().timestamp;	
	
}

void BlockLocalPositionEstimator::correctBaro() {

	math::Vector<1> y;
	y(0) = _sub_sensor.get().baro_alt_meter - _baroAltHome;

	// baro measurement matrix
	math::Matrix<n_y_baro, n_x> C;
	C(Y_baro_z, X_z) = -1; // measured altitude, negative down dir.

	math::Matrix<n_y_baro, n_y_baro> R;
	R(0,0) = _baro_stddev.get()*_baro_stddev.get();

	// residual
	math::Matrix<1,1> S_I =
		((C*_P*C.transposed()) + R).inversed();
	math::Vector<1> r = y- (C*_x);

	// fault detection
	float beta = sqrtf(r*(S_I*r));
	if (beta > 3) { // 3 standard deviations away
		if (!_baroFault) {
			mavlink_log_info(_mavlink_fd, "[lpe] baro fault, beta %5.2f", double(beta));
			warnx("[lpe] baro fault, beta %5.2f", double(beta));
		}
		_baroFault = 1;
		// lower baro trust
		S_I = ((C*_P*C.transposed()) + R*10).inversed();
	} else if (_baroFault) {
		_baroFault = 0;
		mavlink_log_info(_mavlink_fd, "[lpe] baro OK");
		warnx("[lpe] baro OK");
	}

	// kalman filter correction if no hard fault
	// always trust baro a little
	if (_baroFault < 2) {
		math::Matrix<n_x, n_y_baro> K = _P*C.transposed()*S_I;
		_x = _x + K*r;
		_P -= K*C*_P;
	}
	_time_last_baro = _sub_sensor.get().baro_timestamp;
}

void BlockLocalPositionEstimator::correctLidar() {

	if(_sub_distance.get().type != distance_sensor_s::MAV_DISTANCE_SENSOR_LASER) return;	

	float d = _sub_distance.get().current_distance;
	if (d < _sub_distance.get().min_distance ||
			d > _sub_distance.get().max_distance) {
		mavlink_log_info(_mavlink_fd, "[lpe] lidar out of range");
		warnx("[lpe] lidar out of range");
		return;
	}
	math::Matrix<n_y_lidar, n_x> C;
	C(Y_lidar_z, X_z) = -1; // measured altitude,
		 // negative down dir.

	// use parameter covariance unless sensor provides reasonable value
	math::Matrix<n_y_lidar, n_y_lidar> R;
	float cov = _sub_distance.get().covariance;
	if (cov < 1.0e-3f) {
		R(0,0) = _lidar_z_stddev.get()*_lidar_z_stddev.get();
	} else {
		R(0,0) = cov;
	}

	math::Vector<1> y;
	y(0) = (d - _lidarAltHome)*
		cosf(_sub_att.get().roll)*
		cosf(_sub_att.get().pitch);

	// residual
	math::Matrix<1,1> S_I = ((C*_P*C.transposed()) + R).inversed();
	math::Vector<1> r = y - C*_x;

	// fault detection
	float beta = sqrtf(r*(S_I*r));

	// zero is an error code for the lidar
	if (d < 0.001f) {
		if (!(_lidarFault == 2)) {
			mavlink_log_info(_mavlink_fd, "[lpe] lidar error");
			warnx("[lpe] lidar error");
		}
		_lidarFault = 2;
	} else if (beta > 3) { // 3 standard deviations away
		if (!_lidarFault) {
			mavlink_log_info(_mavlink_fd, "[lpe] lidar fault, beta %5.2f", double(beta));
			warnx("[lpe] lidar fault, beta %5.2f", double(beta));
		}
		_lidarFault = 1;
	// disable fault if ok
	} else if (_lidarFault) {
		_lidarFault = 0;
		mavlink_log_info(_mavlink_fd, "[lpe] lidar OK");
		warnx("[lpe] lidar OK");
	}

	// kalman filter correction if no fault
	// want to ignore corrections > 3 std. dev since lidar gives
	// bogus readings at times
	if (_lidarFault == 0) {
		math::Matrix<n_x, n_y_lidar> K = _P*C.transposed()*S_I;
		_x += K*r;
		_P -= K*C*_P;
	}
	_time_last_lidar = _sub_distance.get().timestamp;
}

void BlockLocalPositionEstimator::correctGps() {	// TODO : use another other metric for glitch detection

	// gps measurement in local frame
	double  lat = _sub_gps.get().lat*1.0e-7;
	double  lon = _sub_gps.get().lon*1.0e-7;
	float  alt = _sub_gps.get().alt*1.0e-3f;

	float px = 0;
	float py = 0;
	float pz = alt - _gpsAltHome;
	map_projection_project(&_map_ref, lat, lon, &px, &py);

	//printf("gps: lat %10g, lon, %10g alt %10g\n", lat, lon, double(alt));
	//printf("home: lat %10g, lon, %10g alt %10g\n", _sub_home.lat, _sub_home.lon, double(_sub_home.alt));
	//printf("local: x %10g y %10g z %10g\n", double(px), double(py), double(pz));

	math::Vector<6> y;
	y(0) = px;
	y(1) = py;
	y(2) = pz;
	y(3) = _sub_gps.get().vel_n_m_s;
	y(4) = _sub_gps.get().vel_e_m_s;
	y(5) = _sub_gps.get().vel_d_m_s;

	// gps measurement matrix, measures position and velocity
	math::Matrix<n_y_gps, n_x> C;
	C(Y_gps_x, X_x) = 1;
	C(Y_gps_y, X_y) = 1;
	C(Y_gps_z, X_z) = 1;
	C(Y_gps_vx, X_vx) = 1;
	C(Y_gps_vy, X_vy) = 1;
	C(Y_gps_vz, X_vz) = 1;

	// gps covariance matrix
	math::Matrix<n_y_gps, n_y_gps> R;
	R(0,0) = _gps_xy_stddev.get()*_gps_xy_stddev.get();
	R(1,1) = _gps_xy_stddev.get()*_gps_xy_stddev.get();
	R(2,2) = _gps_z_stddev.get()*_gps_z_stddev.get();
	R(3,3) = _gps_vxy_stddev.get()*_gps_vxy_stddev.get();
	R(4,4) = _gps_vxy_stddev.get()*_gps_vxy_stddev.get();
	R(5,5) = _gps_vz_stddev.get()*_gps_vz_stddev.get();

	// residual
	math::Matrix<6,6> S_I = ((C*_P*C.transposed()) + R).inversed();
	math::Vector<6> r = y - C*_x;

	// fault detection
	float beta = sqrtf(r*(S_I*r));
	if (beta > 3) { // 3 standard deviations away
		if (!_gpsFault) {
			mavlink_log_info(_mavlink_fd, "[lpe] gps fault, beta: %5.2f", double(beta));
			warnx("[lpe] gps fault, beta: %5.2f", double(beta));
		}
		_gpsFault = 1;
		// trust GPS less
		S_I = ((C*_P*C.transposed()) + R*10).inversed();
	} else if (_gpsFault) {
		_gpsFault = 0;
		mavlink_log_info(_mavlink_fd, "[lpe] GPS OK");
		warnx("[lpe] GPS OK");
	}

	// kalman filter correction if no hard fault
	if (_gpsFault < 2) {
		math::Matrix<n_x, n_y_gps> K = _P*C.transposed()*S_I;
		_x += K*r;
		_P -= K*C*_P;
	}
	_time_last_gps = _timeStamp;
}

void BlockLocalPositionEstimator::correctVision() {

	math::Vector<6> y;

	static float last_vision_x = 0.0f;
	static float last_vision_y = 0.0f;
	static float last_vision_z = 0.0f;

	y(0) = _sub_vision_pos.get().x - _visionHome(0);
	y(1) = _sub_vision_pos.get().y - _visionHome(1);
	y(2) = _sub_vision_pos.get().z - _visionHome(2);

	if(_visionVelInitialized) {	// If subscribed to valid vision_speed_estimate topic, use its data
		y(3) = _sub_vision_vel.get().x - _visionBaseVel(0);
		y(4) = _sub_vision_vel.get().y - _visionBaseVel(1);
		y(5) = _sub_vision_vel.get().z - _visionBaseVel(2);
	}
	else {	// Else, derivate velocity from position
		static hrt_abstime last_vision_time = 0;
		static float vx = 0.0f;
		static float vy = 0.0f;
		static float vz = 0.0f;

		float vision_dt = (_sub_vision_pos.get().timestamp_boot - last_vision_time) / 1e6f;
		last_vision_time = _sub_vision_pos.get().timestamp_boot;

		if (vision_dt > 0.000001f && vision_dt < 0.2f) {
			vx = (y(0) - last_vision_x) / vision_dt;
			vy = (y(1) - last_vision_y) / vision_dt;
			vz = (y(2) - last_vision_z) / vision_dt;

			last_vision_x = y(0);
			last_vision_y = y(1);
			last_vision_z = y(2);

			y(3) = vx - y(0);
			y(4) = vy - y(1);
			y(5) = vz - y(2);
		}	
	}

	// vision measurement matrix, measures position and velocity
	math::Matrix<n_y_vision, n_x> C;
	C(Y_vision_x, X_x) = 1;
	C(Y_vision_y, X_y) = 1;
	C(Y_vision_z, X_z) = 1;
	C(Y_vision_vx, X_vx) = 1;
	C(Y_vision_vy, X_vy) = 1;
	C(Y_vision_vz, X_vz) = 1;

	// measurement covariance
	math::Matrix<n_y_vision, n_y_vision> R;
	float vision_p_var = _vision_p_stddev.get()* \
		_vision_p_stddev.get();
	R(Y_vision_x, Y_vision_x) = vision_p_var;
	R(Y_vision_y, Y_vision_y) = vision_p_var;
	R(Y_vision_z, Y_vision_z) = vision_p_var;

	// residual
	math::Matrix<6,6> S_I =
		((C*_P*C.transposed()) + R).inversed();
	math::Vector<6> r = y - C*_x;

	// fault detection
	float beta = sqrtf(r*(S_I*r));
	if (beta > 3) { // 3 standard deviations away
		if (!_visionFault) {
			mavlink_log_info(_mavlink_fd, "[lpe] vision fault, beta %5.2f", double(beta));
			warnx("[lpe] vision fault, beta %5.2f", double(beta));
		}
		_visionFault = 1;
		// trust less
		S_I = ((C*_P*C.transposed()) + R*10).inversed();
	} else if (_visionFault) {
		_visionFault = 0;
		mavlink_log_info(_mavlink_fd, "[lpe] vision OK");
		warnx("[lpe] vision OK");
	}

	// kalman filter correction if no hard fault
	if (_visionFault < 2) {
		math::Matrix<n_x, n_y_vision> K = _P*C.transposed()*S_I;
		_x += K*r;
		_P -= K*C*_P;
	}

	_time_last_vision_p = _sub_vision_pos.get().timestamp_boot;
	_time_last_vision_v = _sub_vision_vel.get().timestamp_boot;
}

void BlockLocalPositionEstimator::correctVicon() {

	math::Vector<3> y;
	y(0) = _sub_vicon.get().x - _viconHome(0);
	y(1) = _sub_vicon.get().y - _viconHome(1);
	y(2) = _sub_vicon.get().z - _viconHome(2);

	// vicon measurement matrix, measures position
	math::Matrix<n_y_vicon, n_x> C;
	C(Y_vicon_x, X_x) = 1;
	C(Y_vicon_y, X_y) = 1;
	C(Y_vicon_z, X_z) = 1;

	// noise matrix
	math::Matrix<n_y_vicon, n_y_vicon> R;
	float vicon_p_var = _vicon_p_stddev.get()* \
		_vicon_p_stddev.get();
	R(Y_vicon_x, Y_vicon_x) = vicon_p_var;
	R(Y_vicon_y, Y_vicon_y) = vicon_p_var;
	R(Y_vicon_z, Y_vicon_z) = vicon_p_var;

	// residual
	math::Matrix<3,3> S_I = ((C*_P*C.transposed()) + R).inversed();
	math::Vector<3> r = y - C*_x;

	// fault detection
	float beta = sqrtf(r*(S_I*r));
	if (beta > 3) { // 3 standard deviations away
		if (!_viconFault) {
			mavlink_log_info(_mavlink_fd, "[lpe] vicon fault, beta %5.2f", double(beta));
			warnx("[lpe] vicon fault, beta %5.2f", double(beta));
		}
		_viconFault = 1;
		// trust less
		S_I = ((C*_P*C.transposed()) + R*10).inversed();
	} else if (_viconFault) {
		_viconFault = 0;
		mavlink_log_info(_mavlink_fd, "[lpe] vicon OK");
		warnx("[lpe] vicon OK");
	}

	// kalman filter correction if no fault
	if (_viconFault < 2) {
		math::Matrix<n_x, n_y_vicon> K = _P*C.transposed()*S_I;
		_x += K*r;
		_P -= K*C*_P;
	}

	_time_last_vicon = _sub_vicon.get().timestamp;
}
