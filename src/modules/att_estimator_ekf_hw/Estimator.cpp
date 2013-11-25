/****************************************************************************
 *
 *   Copyright (c) 2012, 2013 PX4 Development Team. All rights reserved.
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
 * @file Estimator.cpp
 *
 * Hardware Accelerated Attitude Only Extended Kalman filter
 */

#include <poll.h>

#include "Estimator.hpp"
#include <systemlib/err.h>
#include <geo/geo.h>

// constants
static const int8_t ret_ok = 0; 	// no error in function
static const int8_t ret_error = -1; 	// error occurred

static const uint8_t n_states = 6;
static const uint8_t n_input = 3;
static const uint8_t n_meas_mag = 1;
static const uint8_t n_meas_accel = 3;

Estimator::Estimator(SuperBlock *parent, const char *name) :
	SuperBlock(parent, name),
	// ekf matrices
	F(n_states, n_states),
	G(n_states, n_input),
	P(n_states, n_states),
	P0(n_states, n_states),
	Q(n_states, n_states),
	V(n_input, n_input),
	// attitude measurement ekf matrices
	HMag(n_meas_mag, n_states),
	HAccel(n_meas_accel, n_states),
	RMag(n_meas_mag, n_meas_mag),
	RAccel(n_meas_accel, n_meas_accel),
	// attitude representations
	C_nb(),
	q(),
	// subscriptions
	_sensors(&getSubscriptions(), ORB_ID(sensor_combined), 3), // limit to 300 Hz
	_param_update(&getSubscriptions(), ORB_ID(parameter_update), 1000), // limit to 1 Hz
	// publications
	_att(&getPublications(), ORB_ID(vehicle_attitude)),
	// timestamps
	_pubTimeStamp(hrt_absolute_time()),
	_predictTimeStamp(hrt_absolute_time()),
	_sensorsTimeStamp(hrt_absolute_time()),
	_outTimeStamp(hrt_absolute_time()),
	_magCountLast(0),
	_accelCountLast(0),
	// frame count
	_navFrames(0),
	// miss counts
	_miss(0),
	// state
	phi(0), theta(0), psi(0),
	_gyroBias(),
	// parameters for ground station
	_vGyro(this, "V_GYRO"),
	_qGyroBias(this, "V_GYRO_BIAS"),
	_rMag(this, "R_MAG"),
	_rAccel(this, "R_ACCEL"),
	_magDec(this, "ENV_MAG_DEC"),
	_g(this, "ENV_G"),
	_faultMag(this, "FAULT_MAG"),
	_faultAccel(this, "FAULT_ACCEL"),
	_attitudeInitialized(false),
	_attitudeInitCounter(0)
{
	using namespace math;

	// initial state covariance matrix
	P0 = Matrix::identity(n_states) * 0.01f;
	P = P0;

	// i_nitialize the attitude with current measurements
	_sensors.update();
	setAttitudeUsingMeasurements();
	_magCountLast = _sensors.magnetometer_counter;
	_accelCountLast = _sensors.accelerometer_counter;

	// intiailize constant matrices
	// rest of entries zero from ctor
	HMag(0, 2) = 1; // measures psi

	// initialize all parameters
	updateParams();
}

void Estimator::setAttitudeUsingMeasurements()
{
	using namespace math;

	Vector3 accelBody(_sensors.accelerometer_m_s2);
	Vector3 magBody(_sensors.magnetometer_ga);

	float phi0 = atan2(-accelBody.getY(), -accelBody.getZ());
	float theta0 = atan2(accelBody.getX(), -accelBody.getZ());

	// body to frame t (phi 1, theta 2 rotation)
	Dcm C_tb(EulerAngles(phi0, theta0, 0));
	// mag vector expressed in frame t
	Vector3 magT = C_tb*magBody;
	// compute heading using projection to x-y plane in frame t
	float psi0 = atan2f(-magT(1), magT(0));

	// euler angles
	phi = phi0;
	theta = theta0;
	psi = psi0;

	// intialize quaternions
	q = Quaternion(EulerAngles(phi,theta,psi));

	// initialize dcm
	C_nb = Dcm(EulerAngles(phi,theta,psi));
}

void Estimator::update()
{
	using namespace math;

	struct pollfd fds[1];
	fds[0].fd = _sensors.getHandle();
	fds[0].events = POLLIN;

	// poll for new data
	int ret = poll(fds, 1, 1000);

	if (ret < 0) {
		// XXX this is seriously bad - should be an emergency
		return;

	} else if (ret == 0) { // timeout
		return;
	}

	// get new timestamp
	uint64_t newTimeStamp = hrt_absolute_time();

	// check updated subscriptions
	if (_param_update.updated()) updateParams();

	// have to check if updated here, cleared when
	// we call update subscriptions
	bool sensorsUpdate = _sensors.updated();

	// get new information from subscriptions
	// this clears update flag
	updateSubscriptions();

	uint32_t magCount = _sensors.magnetometer_counter;
	uint32_t accelCount = _sensors.accelerometer_counter;

	// initialize attitude when sensors online
	if (!_attitudeInitialized && sensorsUpdate) {


		if (_magCountLast == 0 && magCount > 0 ||
			_accelCountLast == 0 && accelCount > 0) {
			setAttitudeUsingMeasurements();
		}

		if (_magCountLast > 0 && _accelCountLast > 0 &&
			correctAccel() == ret_ok &&
			correctMag() == ret_ok) {
			_attitudeInitCounter++;
		}

		if (_attitudeInitCounter > 100) {
			warnx("initialized EKF attitude\n");
			warnx("phi: %8.4f, theta: %8.4f, psi: %8.4f\n",
			       double(phi), double(theta), double(psi));
			_attitudeInitialized = true;
		}
	}

	// prediction step
	// using sensors timestamp so we can account for packet lag
	float dt = (_sensors.timestamp - _predictTimeStamp) / 1.0e6f;
	//printf("dt: %15.10f\n", double(dt));
	_predictTimeStamp = _sensors.timestamp;

	// don't predict if time greater than a second
	if (dt < 1.0f) {
		predictState(dt);
		predictStateCovariance(dt);
		// count fast frames
		_navFrames += 1;
	}

	// count times 100 Hz rate isn't met
	if (dt > 0.01f) _miss++;

	// correction step
	if (_attitudeInitialized) { // initialized
		if (sensorsUpdate && // new data
		(_sensors.timestamp - _sensorsTimeStamp > 1e6 / 50)) { // 50 Hz
			// we have to limit this because the accel/mag measurements
			// are in the sensors packet which is also being used for
			// gyroscope measurements
			_sensorsTimeStamp = _sensors.timestamp;
			if (_accelCountLast != accelCount) {
				correctAccel();
			}
			if (_magCountLast != magCount) {
				correctMag();
			}
		}
	}

	// publication
	if (newTimeStamp - _pubTimeStamp > 1e6 / 50) { // 50 Hz
		_pubTimeStamp = newTimeStamp;
		updatePublications();
	}

	// output
	if (newTimeStamp - _outTimeStamp > 10e6) { // 0.1 Hz
		_outTimeStamp = newTimeStamp;
		//printf("nav: %4d Hz, miss #: %4d\n",
		//       _navFrames / 10, _miss / 10);
		_navFrames = 0;
		_miss = 0;
		//warnx("bias: %10.4f %10.4f %10.4f", _gyroBias(0), _gyroBias(1), _gyroBias(2));
	}

	// update count history
	_magCountLast = magCount;
	_accelCountLast = accelCount;
}

void Estimator::updatePublications()
{
	using namespace math;

	// attitude publication
	_att.timestamp = _pubTimeStamp;
	_att.roll = phi;
	_att.pitch = theta;
	_att.yaw = psi;
	_att.rollspeed = _sensors.gyro_rad_s[0];
	_att.pitchspeed = _sensors.gyro_rad_s[1];
	_att.yawspeed = _sensors.gyro_rad_s[2];
	// TODO, add gyro offsets to filter
	_att.rate_offsets[0] = 0.0f;
	_att.rate_offsets[1] = 0.0f;
	_att.rate_offsets[2] = 0.0f;

	for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++)
		_att.R[i][j] = C_nb(i, j);

	for (int i = 0; i < 4; i++) _att.q[i] = q(i);

	_att.R_valid = true;
	_att.q_valid = true;

	if (_attitudeInitialized) {
		_att.update();
	}
}

int Estimator::predictState(float dt)
{
	using namespace math;

	// attitude prediction
	if (_attitudeInitialized) {
		Vector3 w(_sensors.gyro_rad_s);

		// attitude
		q = q + q.derivative(w + _gyroBias) * dt;

		// renormalize quaternion if needed
		if (fabsf(q.norm() - 1.0f) > 1e-4f) {
			q = q.unit();
		}

		// C_nb update
		C_nb = Dcm(q);

		// euler update
		EulerAngles euler(C_nb);
		phi = euler.getPhi();
		theta = euler.getTheta();
		psi = euler.getPsi();
	}
	return ret_ok;
}

int Estimator::predictStateCovariance(float dt)
{
	using namespace math;

	// avoid covariance update during gimbal lock
	if (fabs(theta - M_PI_2) < 0.01) return;

	float sinPhi = sin(phi);
	float cosPhi = cos(phi);
	float cosTheta = cos(theta);
	float tanTheta = tan(theta);

	// F Matrix
	F(0, 3) = 1;
	F(0, 4) = sinPhi*tanTheta;
	F(0, 5) = cosPhi*tanTheta;
	F(1, 3) = 0;
	F(1, 4) = cosPhi;
	F(1, 5) = -sinPhi;
	F(2, 3) = 0;
	F(2, 4) = sinPhi/cosTheta;
	F(2, 5) = cosPhi/cosTheta;

	// G Matrix
	G(0, 0) = F(0,3);
	G(0, 1) = F(0,4);
	G(0, 2) = F(0,5);
	G(1, 0) = F(1,3);
	G(1, 1) = F(1,4);
	G(1, 2) = F(1,5);
	G(2, 0) = F(2,3);
	G(2, 1) = F(2,4);
	G(2, 2) = F(2,5);

	// continuous predictioon equations
	// for discrte time EKF
	// http://en.wikipedia.org/wiki/Extended_Kalman_filter
	P = P + (F * P + P * F.transpose() + G * V * G.transpose() + Q) * dt;

	return ret_ok;
}

int Estimator::correctAccel()
{
	using namespace math;

	// trig
	float cosPhi = cosf(phi);
	float cosTheta = cosf(theta);
	// float cosPsi = cosf(psi);
	float sinPhi = sinf(phi);
	float sinTheta = sinf(theta);
	// float sinPsi = sinf(psi);

	// accel measurement
	Vector3 zAccel(_sensors.accelerometer_m_s2);

	// correct accel if accel close to g
	if (fabsf(zAccel.norm() - _g.get()) < 1.0f) {

		// correction vector
		Vector xCorrect(6);

		// update H
		HAccel(0, 1) = cosTheta;
		HAccel(1, 0) = -cosPhi * cosTheta;
		HAccel(1, 1) = sinPhi * sinTheta;
		HAccel(2, 0) = sinPhi * cosTheta;
		HAccel(2, 1) = cosPhi * sinTheta;

		// accel predicted measurement
		Vector3 zAccelUnitHat = (C_nb.transpose() * Vector3(0, 0, -_g.get())).unit();
		Vector3 y = zAccel.unit() - zAccelUnitHat;

		// compute correction
		// http://en.wikipedia.org/wiki/Extended_Kalman_filter
		Matrix S = HAccel * P * HAccel.transpose() + RAccel; // residual covariance
		Matrix K = P * HAccel.transpose() * S.inverse(); // kalman gain
		xCorrect = K * y;

		// check correciton is sane
		for (size_t i = 0; i < xCorrect.getRows(); i++) {
			float val = xCorrect(i);
			if (isnan(val) || isinf(val)) {
				// abort correction and return
				warnx("numerical failure in accel correction\n");
				// reset P matrix to P0
				P = P0;
				return ret_error;
			}
		}

		// update state covariance
		// http://en.wikipedia.org/wiki/Extended_Kalman_filter
		P = P - K * HAccel * P;

		// fault detection
		float beta = y.dot(S.inverse() * y);

		if (beta > _faultAccel.get()) {
			warnx("fault in accel: beta = %8.4f", (double)beta);
			warnx("y:"); y.print();
		}

		// update euler states
		phi += xCorrect(PHI);
		theta += xCorrect(THETA);
		_gyroBias(0) += xCorrect(GYRO_BIAS_X);
		_gyroBias(1) += xCorrect(GYRO_BIAS_Y);
		_gyroBias(2) += xCorrect(GYRO_BIAS_Z);

		// psi should never change
		// psi += xCorrect(PSI);

		// update quaternions from euler
		// angle correction
		q = Quaternion(EulerAngles(phi, theta, psi));

	} else {
		//warnx("ignoring accel correction");
	}

	return ret_ok;
}

int Estimator::correctMag() {
	using namespace math;

	Vector3 magBody(_sensors.magnetometer_ga);

        // mag predicted measurement
        // choosing some typical magnetic field properties,
        // TODO dip/dec depend on lat/ lon/ time
        float dec = _magDec.get() / M_RAD_TO_DEG_F; // declination, clockwise rotation from north

	// body to frame t (phi 1, theta 2 rotation)
	Dcm C_tb(EulerAngles(phi, theta, 0));
	// mag vector expressed in frame t
	Vector3 magTUnit = (C_tb*magBody).unit();
	// compute heading using projection to x-y plane in frame t
	float psiMag = atan2f(-magTUnit(1), magTUnit(0)) - dec;

	// correct  if mag vector isn't perpendicular
	if ((magTUnit(0)*magTUnit(0) + magTUnit(1)*magTUnit(1)) > 1e-2) {

		Vector xCorrect(6);

		// H is constant
		
		// calculate error between estimate and measurement
		// apply declination correction for true heading as well.
		float ys = psiMag - psi;
		if (ys > M_PI_F) ys -= 2*M_PI_F;
		if (ys < -M_PI_F) ys += 2*M_PI_F;
		Vector y(1);
		y(0) = ys;

		// compute correction
		// http://en.wikipedia.org/wiki/Extended_Kalman_filter
		Matrix S = HMag * P * HMag.transpose() + RMag; // residual covariance
		Matrix K = P * HMag.transpose() * S.inverse(); // kalman gain
		xCorrect = K * y;

		// check correciton is sane
		for (size_t i = 0; i < xCorrect.getRows(); i++) {
			float val = xCorrect(i);
			if (isnan(val) || isinf(val)) {
				// abort correction and return
				warnx("numerical failure in mag correction\n");
				// reset P matrix to P0
				P = P0;
				return ret_error;
			}
		}

		// update state covariance
		// http://en.wikipedia.org/wiki/Extended_Kalman_filter
		P = P - K * HMag * P;

		// fault detection
		float beta = y.dot(S.inverse() * y);

		if (beta > _faultMag.get()) {
			warnx("fault in mag: beta = %8.4f", (double)beta);
			warnx("y:"); y.print();
		}

		// update euler states
		// phi, theta should never change
		// phi += xCorrect(PHI);
		// theta += xCorrect(THETA);
		psi += xCorrect(PSI);
		_gyroBias(0) += xCorrect(GYRO_BIAS_X);
		_gyroBias(1) += xCorrect(GYRO_BIAS_Y);
		_gyroBias(2) += xCorrect(GYRO_BIAS_Z);

		// update quaternions from euler
		// angle correction
		q = Quaternion(EulerAngles(phi, theta, psi));

	} else {
		//warnx("ignoring mag correction");
	}

	return ret_ok;
}

void Estimator::updateParams()
{
	using namespace math;
	using namespace control;
	SuperBlock::updateParams();

	// process noise
	Q(3, 3) = _qGyroBias.get();   // gyro bias x
	Q(4, 4) = _qGyroBias.get();   // gyro bias y
	Q(5, 5) = _qGyroBias.get();   // gyro bias z

	// gyro noise
	V(0, 0) = _vGyro.get();   // gyro x, rad/s
	V(1, 1) = _vGyro.get();   // gyro y
	V(2, 2) = _vGyro.get();   // gyro z

	// magnetometer noise
	float noiseMin = 1e-6f;
	float noiseMagSq = _rMag.get() * _rMag.get();
	if (noiseMagSq < noiseMin) noiseMagSq = noiseMin;
	RMag(0, 0) = noiseMagSq; // normalized direction

	// accelerometer noise
	float noiseAccelSq = _rAccel.get() * _rAccel.get();
	if (noiseAccelSq < noiseMin) noiseAccelSq = noiseMin;
	RAccel(0, 0) = noiseAccelSq;
	RAccel(1, 1) = noiseAccelSq;
	RAccel(2, 2) = noiseAccelSq;
}
