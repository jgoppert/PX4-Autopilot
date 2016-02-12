#include <systemlib/param/param.h>
#include <math.h>

// 16 is max name length

/**
 * Yaw error to yaw rate command
 *
 * @group Segway
 * @min 0
 * @max 1.5
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(SEG_YAW2R, 1.0f); // yaw error to yaw rate

/**
 * Yaw rate error to voltage output
 *
 * @group Segway
 * @min 0 
 * @max 0.5
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(SEG_R2V, 0.2f); // yaw rate error to voltage

/**
 * Pitch error to voltage output
 *
 * @group Segway
 * @min 0
 * @max 20
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(SEG_TH2V, 10.0f); // pitch error to voltage

/**
 * Pitch rate error to voltage output
 *
 * @group Segway
 * @min 0
 * @max 1
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(SEG_Q2V, 0.5f); // pitch rate error to voltage

/**
 * Position error to velocity command
 *
 * @group Segway
 * @min 0
 * @max 1
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(SEG_X2VEL_P, 0.1f); // proportional gain

/**
 * Integral of position error to velocity command
 *
 * @group Segway
 * @min 0
 * @max 1
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(SEG_X2VEL_I, 0.0f); // integrator gain

/**
 * Maximum Integral of position error to velocity command
 *
 * @group Segway
 * @min 0
 * @max 1
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(SEG_X2VEL_I_MAX, 0.0f); // max integrator windup

// for dynamics
/*PARAM_DEFINE_FLOAT(SEG_MGL, 7.0f); // feedforward term, gravity torque*/
/*PARAM_DEFINE_FLOAT(SEG_J, 7.0f); // inertia*/
/*PARAM_DEFINE_FLOAT(SEG_K_EMF, 1.0f); // motor electromotive force constant*/
/*PARAM_DEFINE_FLOAT(SEG_K_DAMP, 1.0f); // motor damping*/
/*PARAM_DEFINE_FLOAT(SEG_WN_THETA, 1.0f); // desired natural freq*/
/*PARAM_DEFINE_FLOAT(SEG_ZETA_THETA, 0.7f); // desired damping ratio*/
/*PARAM_DEFINE_FLOAT(SEG_BEMF, 0.0f); // desired damping ratio*/

/**
 * Velocity error to pitch command
 *
 * @group Segway
 * @min 0
 * @max 1
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(SEG_VEL2TH_P, 0.2f); // proportional gain

/**
 * Velocity error integral to pitch command
 *
 * @group Segway
 * @min 0
 * @max 1
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(SEG_VEL2TH_I, 0.2f); // integrator gain

/**
 * Velocity error max integral to pitch command
 *
 * @group Segway
 * @min 0
 * @max 1
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(SEG_VEL2TH_I_MAX, 0.5f); // max integrator windup

/**
 * Maximum commanded pitch
 *
 * @group Segway
 * @min 0
 * @max 1
 * @unit radians
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(SEG_TH_LIM_MAX, 0.1f); // pitch limit

/**
 * Maximum commanded velocity
 *
 * @group Segway
 * @min 0
 * @max 1
 * @unit meters/sec
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(SEG_VEL_LIM_MAX, 0.2f); // velocity limit

/**
 * Pitch angles to shut motors off at
 *
 * @group Segway
 * @min 0
 * @max 1
 * @unit radians
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(SEG_TH_STOP, 0.2f); // turn off motors when over

// system id
/*PARAM_DEFINE_FLOAT(SEG_SYSID_ENABLE, 0.0f); // wave amplitude, deg pitch*/
/*PARAM_DEFINE_FLOAT(SEG_SYSID_AMP, 0.5f); // wave amplitude, deg pitch*/
/*PARAM_DEFINE_FLOAT(SEG_SYSID_FREQ, 0.1f); // wave frquency, Hz*/

/**
 * Radius of wheel
 *
 * @group Segway
 * @min 0
 * @max 1
 * @unit meters
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(ENCP_RWHEEL, 0.1f); // radius of wheel

/**
 * Pulses per revolution of encoder
 *
 * @group Segway
 * @min 0
 * @max 10000
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(ENCP_PPR, 3200.0f); // encoder pulses per revolution of wheel
