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

#include <systemlib/param/param.h>

/*PARAM_DEFINE_FLOAT(NAME,0.0f);*/
PARAM_DEFINE_FLOAT(ATTH_V_GYRO, 0.008f); // gyroscope noise std dev. rad/s
PARAM_DEFINE_FLOAT(ATTH_Q_GYRO_BIAS, 0.001f); // gyro bias noise std dev. rad/s
PARAM_DEFINE_FLOAT(ATTH_R_MAG, 1.0f); // magnetometer noise std dev. normalized
PARAM_DEFINE_FLOAT(ATTH_R_ACCEL, 1.0f); // accelerometer noise std dev. norm.
PARAM_DEFINE_FLOAT(ATTH_FAULT_MAG, 1.0f); // fault threshold magnetometer
PARAM_DEFINE_FLOAT(ATTH_FAULT_ACCEL, 1.0f); // " accelerometer
PARAM_DEFINE_FLOAT(ATTH_ENV_G, 9.8f); // local gravity
PARAM_DEFINE_FLOAT(ATTH_ENV_MAG_DEC, 0.0f); // magnetic declination
