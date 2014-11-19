#include <systemlib/param/param.h>

// 16 is max name length
PARAM_DEFINE_FLOAT(FL_SRV_TRV, 100.0f); // servo travel, deg
PARAM_DEFINE_FLOAT(FL_WNG_UP, -45.0f); // wing up pos, deg
PARAM_DEFINE_FLOAT(FL_WNG_DWN, 45.0f); // wing down pos, deg
PARAM_DEFINE_FLOAT(FL_WNG_GLD, 0.0f); // wing glide pos, deg

// not currently used (old square wave version)
PARAM_DEFINE_FLOAT(FL_T_DWN2UP, 0.06f); // down to up time, s
PARAM_DEFINE_FLOAT(FL_T_UP2GLD, 0.16f); // down to up time, s
PARAM_DEFINE_FLOAT(FL_THR_GLD, 0.2f); // throttle to glide below, 0-1

// flapping function
PARAM_DEFINE_FLOAT(FL_THR2FREQ, 3.3f); // norm. throttle to freq gain
PARAM_DEFINE_FLOAT(FL_MIN_FREQ, 1.7f); // min flapping freq

// learning
PARAM_DEFINE_FLOAT(FL_LRN_TIME, 10e6f); // test duration for each genome
PARAM_DEFINE_FLOAT(FL_MUT_PROB, 0.05f); // mutation probability
PARAM_DEFINE_FLOAT(FL_REP_RATIO, 0.34f); // mutation probability
PARAM_DEFINE_FLOAT(FL_AIL_MIN, -16.0f); // ail min value
PARAM_DEFINE_FLOAT(FL_AIL_RANGE, 32.0f); // ail range
PARAM_DEFINE_FLOAT(FL_ELEV_MIN, -16.0f); // elv min value
PARAM_DEFINE_FLOAT(FL_ELEV_RANGE, 32.0f); // elv range

// low pass wing outputs
PARAM_DEFINE_FLOAT(FL_WING_LP, 2.0f); // make transitions smooth, for testing

// delta shift flap parameters
PARAM_DEFINE_FLOAT(FL_DELTA, 0.0f);
