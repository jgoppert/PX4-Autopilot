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
 * @min 0.01
 * @max 100.0
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(CEI_SN_GYRO_RW, 0.1f);


/**
 * Magnetometer Rotational Noise Std. Dev. (std. dev./ typical norm)
 *
 * @group CEI
 * @unit rad
 * @min 0.01
 * @max 100.0
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(CEI_STD_MAG, 2.5f);

/**
 * Accel Rotational Noise Std. Dev. (std. dev./ typical norm)
 *
 * @group CEI
 * @unit rad
 * @min 0.01
 * @max 100.0
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(CEI_STD_ACC, 3.5.0f);

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
