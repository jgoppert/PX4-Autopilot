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

/**
 * @file Lidar.cpp
 * @author James Goppert <james.goppert@gmail.com>
 */

#include "Lidar.hpp"

Lidar::Lidar(SuperBlock * parent, const char * name, float timeOut) :
	Measurement<float, 1>(parent, name, timeOut),
	_sub(NULL),
	_lidar_z_stddev(this, "Z")
{
}

void Lidar::init()
{
	// collect lidar data
	//bool valid = false;
	//float d = _sub_lidar->get().current_distance;

	//if (d < _sub_lidar->get().max_distance &&
		//d > _sub_lidar->get().min_distance &&
		//_sub_lidar->get().timestamp != 0) {
		//valid = true;
	//}

	//if (!_lidarInitialized && valid) {
		//// increament sums for mean
		//_lidarAltHome += _sub_lidar->get().current_distance;

		//if (_lidarInitCount++ > REQ_INIT_COUNT) {
			//_lidarAltHome /= _lidarInitCount;
			//mavlink_log_info(_mavlink_fd, "[lpe] lidar init: "
					 //"alt %d cm",
					 //int(100 * _lidarAltHome));
			//warnx("[lpe] lidar init: alt %d cm",
				  //int(100 * _lidarAltHome));
			//_lidarInitialized = true;
		//}

		//if (!_altHomeInitialized) {
			//_altHomeInitialized = true;
			//_altHome = _lidarAltHome;
		//}
	//}
}

void Lidar::correct() {
	//if (_sub_lidar->get().timestamp == 0) { return; }

	//float d = _sub_lidar->get().current_distance;

	//Matrix<float, n_y_lidar, n_x> C;
	//C.setZero();
	//C(Y_lidar_z, X_z) = -1; // measured altitude,
	//// negative down dir.

	//// use parameter covariance unless sensor provides reasonable value
	//Matrix<float, n_y_lidar, n_y_lidar> R;
	//R.setZero();
	//float cov = _sub_lidar->get().covariance;

	//if (cov < 1.0e-3f) {
		//R(0, 0) = _lidar_z_stddev.get() * _lidar_z_stddev.get();

	//} else {
		//R(0, 0) = cov;
	//}

	//Vector<float, n_y_lidar> y;
	//y.setZero();
	//y(0) = (d - _lidarAltHome) *
		   //cosf(_sub_att.get().roll) *
		   //cosf(_sub_att.get().pitch);

	//// residual
	//Matrix<float, n_y_lidar, n_y_lidar> S_I = inv<float, n_y_lidar>((C * _P * C.transpose()) + R);
	//Vector<float, n_y_lidar> r = y - C * _x;

	//// fault detection
	//float beta = sqrtf((r.transpose() * (S_I * r))(0, 0));

	//// zero is an error code for the lidar
	//if (d < _sub_lidar->get().min_distance ||
		//d > _sub_lidar->get().max_distance) {
		//if (!_lidarFault) {
			//mavlink_log_info(_mavlink_fd, "[lpe] lidar out of range");
			//warnx("[lpe] lidar out of range");
			//_lidarFault = FAULT_SEVERE;
		//}

	//} else if (beta > _beta_max.get()) {
		//if (!_lidarFault) {
			//mavlink_log_info(_mavlink_fd, "[lpe] lidar fault, beta %5.2f", double(beta));
			//warnx("[lpe] lidar fault, beta %5.2f", double(beta));
			//_lidarFault = FAULT_MINOR;
		//}

	//} else if (_lidarFault) { // disable fault if ok
		//_lidarFault = FAULT_NONE;
		//mavlink_log_info(_mavlink_fd, "[lpe] lidar OK");
		//warnx("[lpe] lidar OK");
	//}

	//// kalman filter correction if no fault
	//if (_lidarFault == FAULT_NONE) {
		//Matrix<float, n_x, n_y_lidar> K = _P * C.transpose() * S_I;
		//_x += K * r;
		//_P -= K * C * _P;
	//}

	//_time_last_lidar = _sub_lidar->get().timestamp;
}
