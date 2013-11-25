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
 * @file Estimator.hpp
 *
 * kalman filter navigation code
 */

#pragma once

// helpful for debug
#define MATRIX_ASSERT
#define VECTOR_ASSERT

#include <nuttx/config.h>

#include <mathlib/mathlib.h>
#include <controllib/blocks.hpp>
#include <controllib/block/BlockParam.hpp>
#include <controllib/uorb/UOrbSubscription.hpp>
#include <controllib/uorb/UOrbPublication.hpp>

#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/parameter_update.h>

#include <drivers/drv_accel.h>
#include <drivers/drv_gyro.h>
#include <drivers/drv_mag.h>

#include <drivers/drv_hrt.h>
#include <poll.h>
#include <unistd.h>

/**
 * Kalman filter navigation class
 * http://en.wikipedia.org/wiki/Extended_Kalman_filter
 * Discrete-time extended Kalman filter
 */
class Estimator : public control::SuperBlock
{
public:
	/**
	 * Constructor
	 */
	Estimator(SuperBlock *parent, const char *name);

	/**
	 * Deconstuctor
	 */

	virtual ~Estimator() {};

	void setAttitudeUsingMeasurements();

	/**
	 * The main callback function for the class
	 */
	void update();


	/**
	 * Publication update
	 */
	virtual void updatePublications();

	/**
	 * State prediction
	 * Continuous, non-linear
	 */
	int predictState(float dt);

	/**
	 * State covariance prediction
	 * Continuous, linear
	 */
	int predictStateCovariance(float dt);

	/**
	 * Accelerometer correction
	 */
	int correctAccel();

	/**
	 * Magnetometer correction
	 */
	int correctMag();

	/**
	 * Position correction
	 */
	int correctPos();

	/**
	 * Overloaded update parameters
	 */
	virtual void updateParams();
protected:
	// kalman filter
	math::Matrix F;             /**< Jacobian(f,x), where dx/dt = f(x,u) */
	math::Matrix G;             /**< noise shaping matrix for gyro/accel */
	math::Matrix P;             /**< state covariance matrix */
	math::Matrix P0;            /**< initial state covariance matrix */
	math::Matrix Q;             /**< process noise matrix */
	math::Matrix V;             /**< gyro noise matrix */
	math::Matrix HMag;          /**< magnetometer measurement jacobian matrix */
	math::Matrix HAccel;        /**< accelerometer measurement jacobian matrix */
	math::Matrix RMag;          /**< position measurement noise matrix */
	math::Matrix RAccel;        /**< accelerometer measurement noise matrix */
	// attitude
	math::Dcm C_nb;             /**< direction cosine matrix from body to nav frame */
	math::Quaternion q;         /**< quaternion from body to nav frame */
	// subscriptions
	control::UOrbSubscription<sensor_combined_s> _sensors;          /**< sensors sub. */
	control::UOrbSubscription<parameter_update_s> _param_update;    /**< parameter update sub. */
	// publications
	control::UOrbPublication<vehicle_attitude_s> _att;              /**< attitude pub. */

	// time stamps
	uint64_t _pubTimeStamp;     /**< output data publication time stamp */
	uint64_t _predictTimeStamp; /**< prediction time stamp */
	uint64_t _sensorsTimeStamp; /**< sensors correction time stamp */
	uint64_t _outTimeStamp;     /**< output time stamp */
	// counters
	uint32_t _magCountLast;     /**< last mag reading count*/
	uint32_t _accelCountLast;   /**< last mag reading count*/
	// frame count
	uint16_t _navFrames;        /**< navigation frames completed in output cycle */
	// miss counts
	uint16_t _miss;         	/**< number of times fast prediction loop missed */
	// states
	enum {PHI = 0, THETA, PSI, GYRO_BIAS_X, GYRO_BIAS_Y, GYRO_BIAS_Z};  /**< state enumeration */
	float phi, theta, psi;                  /**< 3-2-1 euler angles */
	math::Vector3 _gyroBias;
	// parameters
	control::BlockParamFloat _vGyro;      /**< gyro process noise */
	control::BlockParamFloat _qGyroBias;  /**< gyro bias process noise */
	control::BlockParamFloat _rMag;       /**< magnetometer measurement noise  */
	control::BlockParamFloat _rAccel;     /**< accelerometer measurement noise */
	control::BlockParamFloat _magDec;     /**< magnetic declination, clockwise rotation */
	control::BlockParamFloat _g;          /**< gravitational constant */
	control::BlockParamFloat _faultMag;   /**< fault threshold for mag*/
	control::BlockParamFloat _faultAccel; /**< fault threshold  for accelerometer */
	// status
	bool _attitudeInitialized;
	uint16_t _attitudeInitCounter;
};
