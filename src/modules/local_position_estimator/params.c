#include <systemlib/param/param.h>

// 16 is max name length
PARAM_DEFINE_FLOAT(LPE_SD_FLW_V, 1.0e-2f); // std dev for flow velocity
PARAM_DEFINE_FLOAT(LPE_SD_FLW_Z, 2.0e-2f); // flow z
PARAM_DEFINE_FLOAT(LPE_SD_LDR_Z, 5.0e-2f); // lidar z
PARAM_DEFINE_FLOAT(LPE_SD_ACC_XY, 1.0e-3f); // accel xy
PARAM_DEFINE_FLOAT(LPE_SD_ACC_Z, 1.0e-2f); // accel z
PARAM_DEFINE_FLOAT(LPE_SD_BAR_Z, 5.0f); // baro z
