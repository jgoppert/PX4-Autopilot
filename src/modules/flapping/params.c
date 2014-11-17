#include <systemlib/param/param.h>

// 16 is max name length
PARAM_DEFINE_FLOAT(FL_TH2V_P, 10.0f); // pitch to voltage
PARAM_DEFINE_FLOAT(FL_TH2V_I, 0.0f); // pitch integral to voltage
PARAM_DEFINE_FLOAT(FL_TH2V_I_MAX, 0.0f); // integral limiter
PARAM_DEFINE_FLOAT(FL_Q2V, 1.0f); // pitch rate to voltage

PARAM_DEFINE_FLOAT(FL_SRV_TRV, 45.0f); // servo travel, deg
PARAM_DEFINE_FLOAT(FL_WNG_UP, -30.0f); // wing up pos, deg
PARAM_DEFINE_FLOAT(FL_WNG_DWN, 25.0f); // wing down pos, deg
PARAM_DEFINE_FLOAT(FL_WNG_GLD, -8.0f); // wing glide pos, deg
PARAM_DEFINE_FLOAT(FL_T_DWN2UP, 0.06f); // down to up time, s
PARAM_DEFINE_FLOAT(FL_T_UP2GLD, 0.16f); // down to up time, s
PARAM_DEFINE_FLOAT(FL_THR_GLD, 0.2f); // throttle to glide below, 0-1
PARAM_DEFINE_FLOAT(FL_THR2FREQ, 3.3f); // norm. throttle to freq gain
PARAM_DEFINE_FLOAT(FL_MIN_FREQ, 1.7f); // min flapping freq
PARAM_DEFINE_FLOAT(FL_FITNESS,-1e9f); // fitness for learning
