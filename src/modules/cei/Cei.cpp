#include "Cei.hpp"
#include "casadi_ekf.h"
#include <cstdlib>
#include <drivers/drv_hrt.h>

Cei::Cei() :
	SuperBlock(nullptr, "CEI"),
	ModuleParams(nullptr),
	// subscriptions
	_sub_param_update(ORB_ID(parameter_update), 1000 / 2, 0, &getSubscriptions()),
	_sub_sensor(ORB_ID(sensor_combined), 1000 / 200, 0, &getSubscriptions()),
	_sub_mag(ORB_ID(vehicle_magnetometer), 1000 / 50, 0, &getSubscriptions()),
	_sub_gps(ORB_ID(vehicle_gps_position), 1000 / 10, 0, &getSubscriptions()),

	// publications
	_pub_lpos(ORB_ID(vehicle_local_position), -1, &getPublications()),
	// misc
	_polls()
{
	_polls[POLL_PARAM].fd = _sub_param_update.getHandle();
	_polls[POLL_PARAM].events = POLLIN;

	_polls[POLL_SENSORS].fd = _sub_sensor.getHandle();
	_polls[POLL_SENSORS].events = POLLIN;
}

void Cei::update() {
	// wait for a sensor update, check for exit condition every 100 ms
	int ret = px4_poll(_polls, 3, 100);

	if (ret < 0) {
		return;
	}

	uint64_t newTimeStamp = hrt_absolute_time();
	float dt = (newTimeStamp - _timeStamp) / 1.0e6f;
	_timeStamp = newTimeStamp;

	// set dt for all child blocks
	setDt(dt);

	// predict
	{
		casadi_int iw[7];
		casadi_real w[297];
		const casadi_real* arg[7] = {w+0, w+6, w+9, w+45};
		casadi_real* res[3] = {w+51, w+57};
		ekf_predict(arg, res, iw, w+87, 0);
	}

	// correct gps
	if (_sub_gps.updated()) {
		_sub_gps.update();
		casadi_int iw[7];
		casadi_real w[312];
		const casadi_real* arg[7] = {w+0, w+6, w+9, w+45};
		casadi_real* res[3] = {w+48, w+54};
		//for (j=0; j<48; ++j) scanf("%lf", a++);
		ekf_correct(arg, res, iw, w+90, 0);
	}

	// correct mag
	if (_sub_mag.updated()) {
		_sub_mag.update();
		casadi_int iw[7];
		casadi_real w[312];
		const casadi_real* arg[7] = {w+0, w+6, w+9, w+45};
		casadi_real* res[3] = {w+48, w+54};
		//for (j=0; j<48; ++j) scanf("%lf", a++);
		ekf_correct(arg, res, iw, w+90, 0);
	}
}
