#include <systemlib/param/param.h>
#include <math.h>

// 16 is max name length
PARAM_DEFINE_FLOAT(TRT_AZM_P, 1.0f); // azimuth error to duty
PARAM_DEFINE_FLOAT(TRT_ELV_P, 1.0f); // elevation error to duty
PARAM_DEFINE_FLOAT(TRT_AZM_D, 0.1f); // azim rate to duty
PARAM_DEFINE_FLOAT(TRT_ELV_D, 0.1f); // elev rate error to duty
PARAM_DEFINE_FLOAT(TRT_LIM_AZM_MAX, 1.0f); // max azimuth angle
PARAM_DEFINE_FLOAT(TRT_LIM_ELV_MAX, 1.0f); // max elevation angle

PARAM_DEFINE_FLOAT(TRT_TRG_RMIN, 1.0f); // min trigger range, m
PARAM_DEFINE_FLOAT(TRT_TRG_RMAX, 30.0f); // max trigger range, m
PARAM_DEFINE_FLOAT(TRT_TRG_AMAX, 10.0f); // max trigger angle error, deg
