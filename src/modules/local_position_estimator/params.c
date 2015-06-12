#include <systemlib/param/param.h>

// 16 is max name length


/**
 * Enable local position estimator.
 *
 * @group Local Position Estimator
 */
PARAM_DEFINE_INT32(LPE_ENABLED, 1);

/**
 * Enable accelerometer integration for prediction.
 *
 * @group Local Position Estimator
 */
PARAM_DEFINE_INT32(LPE_INTEGRATE, 1);

/**
 * Optical flow xy standard deviation.
 *
 * @group Local Position Estimator
 * @unit m
 * @min 0.01
 * @max 1
 */
PARAM_DEFINE_FLOAT(LPE_FLW_XY, 0.01f);

/**
 * Sonar z standard deviation.
 *
 * @group Local Position Estimator
 * @unit m
 * @min 0.01
 * @max 1
 */
PARAM_DEFINE_FLOAT(LPE_SNR_Z, 0.2f);

/**
 * Lidar z standard deviation.
 *
 * @group Local Position Estimator
 * @unit m
 * @min 0.01
 * @max 1
 */
PARAM_DEFINE_FLOAT(LPE_LDR_Z, 0.03f);

/**
 * Accelerometer xy noise power (variance*sampling rate).
 *
 * @group Local Position Estimator
 * @unit (m/s^2)^2-s
 * @min 0.01
 * @max 2
 */
PARAM_DEFINE_FLOAT(LPE_ACC_XY, 0.135f);

/**
 * Accelerometer z noise power (variance*sampling rate).
 *
 * @group Local Position Estimator
 * @unit (m/s^2)^2-s
 * @min 0.01
 * @max 2
 */
PARAM_DEFINE_FLOAT(LPE_ACC_Z, 0.175f);

/**
 * Barometric presssure altitude z standard deviation.
 *
 * @group Local Position Estimator
 * @unit m
 * @min 0.01
 * @max 3
 */
PARAM_DEFINE_FLOAT(LPE_BAR_Z, 0.255f);

/**
 * GPS xy standard deviation.
 *
 * @group Local Position Estimator
 * @unit m
 * @min 0.01
 * @max 5
 */
PARAM_DEFINE_FLOAT(LPE_GPS_XY, 0.523f);

/**
 * GPS z standard deviation.
 *
 * @group Local Position Estimator
 * @unit m
 * @min 0.01
 * @max 5
 */
PARAM_DEFINE_FLOAT(LPE_GPS_Z, 3.55f);

/**
 * GPS xy velocity standard deviation.
 *
 * @group Local Position Estimator
 * @unit m/s
 * @min 0.01
 * @max 2
 */
PARAM_DEFINE_FLOAT(LPE_GPS_VXY, 0.275f);

/**
 * GPS z velocity standard deviation.
 *
 * @group Local Position Estimator
 * @unit m/s
 * @min 0.01
 * @max 2
 */
PARAM_DEFINE_FLOAT(LPE_GPS_VZ, 0.237f);

/**
 * Vision xy standard deviation.
 *
 * @group Local Position Estimator
 * @unit m
 * @min 0.01
 * @max 1
 */
PARAM_DEFINE_FLOAT(LPE_VIS_XY, 0.5f);

/**
 * Vision z standard deviation.
 *
 * @group Local Position Estimator
 * @unit m
 * @min 0.01
 * @max 2
 */
PARAM_DEFINE_FLOAT(LPE_VIS_Z, 0.5f);

/**
 * Vision xy velocity standard deviation.
 *
 * @group Local Position Estimator
 * @unit m/s
 * @min 0.01
 * @max 2
 */
PARAM_DEFINE_FLOAT(LPE_VIS_VXY, 1.0f);

/**
 * Vision z velocity standard deviation.
 *
 * @group Local Position Estimator
 * @unit m/s
 * @min 0.01
 * @max 2
 */
PARAM_DEFINE_FLOAT(LPE_VIS_VZ, 1.0f);

/**
 * Circuit breaker to disable vision input.
 *
 * Set to the appropriate key (328754) to disable vision input.
 *
 * @group Local Position Estimator
 * @min 0
 * @max 1
 */
PARAM_DEFINE_INT32(LPE_NO_VISION, 0);

/**
 * Vicon position standard deviation.
 *
 * @group Local Position Estimator
 * @unit m
 * @min 0.01
 * @max 1
 */
PARAM_DEFINE_FLOAT(LPE_VIC_P, 0.05f);

/**
 * Position propagation process noise power (variance*sampling rate).
 *
 * @group Local Position Estimator
 * @unit (m/s)^2-s
 * @min 0
 * @max 1
 */
PARAM_DEFINE_FLOAT(LPE_PN_P, 0.0f);

/**
 * Velocity propagation process noise power (variance*sampling rate).
 *
 * @group Local Position Estimator
 * @unit m/s
 * @min 0
 * @max 5
 */
PARAM_DEFINE_FLOAT(LPE_PN_V, 0.0f);

/**
 * Fault detection threshold in standard deviations
 *
 * @group Local Position Estimator
 * @unit stddev
 * @min 3
 * @max 10
 */
PARAM_DEFINE_FLOAT(LPE_BETA_MAX, 5.0f);
