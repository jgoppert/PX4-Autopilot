#include <parameters/param.h>

/**
 * Gyro Noise Std. Dev.
 *
 * @group CEI
 * @unit mrad
 * @min 0.01
 * @max 100.0
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(CEI_STD_GYRO, 1.0f);

/**
 * Gyro Random Walk Sqrt Noise Power
 *
 * @group CEI
 * @unit (mrad/s) * sqrt(s)
 * @min 0
 * @max 100.0
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(CEI_SN_GYRO_RW, 0.01f);


/**
 * Magnetometer Rotational Noise Std. Dev. (std. dev./ typical norm)
 *
 * @group CEI
 * @unit rad
 * @min 0.01
 * @max 100.0
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(CEI_STD_MAG, 25.0f);

/**
 * Mag Beta Scaling
 *
 * @group CEI
 * @unit None
 * @min 0.01
 * @max 100.0
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(CEI_BETA_MAG_C, 6.6f);

/**
 * Magnetic Declination
 *
 * @group CEI
 * @unit rad
 * @min -3.14159
 * @max 3.14159
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(CEI_MAG_DECL, 0.0f);

/**
 * Accel Rotational Noise Std. Dev. (std. dev./ typical norm)
 *
 * @group CEI
 * @unit rad
 * @min 0.01
 * @max 100.0
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(CEI_STD_ACC, 35.0f);

/**
 * Accel Angular Velocity Noise Std. Dev.
 *
 * Accounts for degradation of orientation measurement due to centrip. accel.
 *
 * @group CEI
 * @unit rad/ (rad/s)^2
 * @min 0.00
 * @max 1000.0
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(CEI_STD_ACC_W, 100.0f);

/**
 * Accel Beta Scaling
 *
 * @group CEI
 * @unit None
 * @min 0.01
 * @max 100.0
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(CEI_BETA_ACC_C, 9.2f);
