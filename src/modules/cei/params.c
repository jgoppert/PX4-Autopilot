#include <parameters/param.h>

/**
 * Magnetometer Noise
 *
 * @group CEI
 * @unit m
 * @min 0.001
 * @max 1.0
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(CEI_MAG_W, 0.1f);
