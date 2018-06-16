#include <parameters/param.h>

/**
 * Attitude Process Noise Std. Dev.
 *
 * @group CEI
 * @unit
 * @min 0
 * @max 1.0
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(CEI_ATT_W, 0.0f);

/**
 * Magnetometer Noise Std. Dev.
 *
 * @group CEI
 * @unit radians
 * @min 0.01
 * @max 1.0
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(CEI_MAG_W, 0.002f);

/**
 * Accel Noise Std. Dev.
 *
 * @group CEI
 * @unit radians
 * @min 0.01
 * @max 1.0
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(CEI_ACC_W, 0.005f);

/**
 * Magnetic Declination
 *
 * @group CEI
 * @unit radians
 * @min -3.14159
 * @max 3.14159
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(CEI_MAG_DECL, 0.0f);
