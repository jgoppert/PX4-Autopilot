#include <px4_module_params.h>
#include <controllib/blocks.hpp>
#include <px4_posix.h>
#include <perf/perf_counter.h>

// uORB subscriptions
#include <uORB/Subscription.hpp>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_magnetometer.h>


// uORB publications
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/ekf2_innovations.h>

// controllib blocks
#include <controllib/BlockStats.hpp>

#include "casadi/mem.h"


using namespace matrix;


class CasadiFunc
{
private:
	casadi_mem *_mem = nullptr;
public:
	CasadiFunc(casadi_functions *f) :
		_mem(casadi_alloc(f))
	{
	}
	~CasadiFunc()
	{
		casadi_free(_mem);
	}
	void res(int i, casadi_real *v)
	{
		_mem->res[i] = v;
	}
	void arg(int i, const casadi_real *v)
	{
		_mem->arg[i] = v;
	}
	void eval()
	{
		casadi_eval(_mem);
	}
	casadi_io in(int i)
	{
		return _mem->in[i];
	}
	casadi_io out(int i)
	{
		return _mem->out[i];
	}
};

class Cei : public control::SuperBlock, public ModuleParams
{
public:
	// constants
	enum {POLL_SENSORS, POLL_PARAM, n_poll};
	enum {X_rx, X_ry, X_rz, X_bgx, X_bgy, X_bgz, n_x};

	// methods
	Cei();
	~Cei();

	void update();
	void status();

private:

	perf_counter_t _perf_elapsed;
	bool _initialized;
	bool _shadow;

	// blocks
	control::BlockStats<float, 3> _mag_stats;
	control::BlockStats<float, 3> _accel_stats;

	// casadi function interfaces
	CasadiFunc _mrp_shadow;
	CasadiFunc _mrp_to_quat;
	CasadiFunc _quat_to_euler;
	CasadiFunc _predict_W;
	CasadiFunc _x_predict;
	CasadiFunc _correct_accel;
	CasadiFunc _correct_mag;
	CasadiFunc _init;

	// subscriptions
	uORB::Subscription<parameter_update_s> _sub_param_update;
	uORB::Subscription<sensor_combined_s> _sub_sensor;
	uORB::Subscription<vehicle_magnetometer_s> _sub_mag;
	uORB::Subscription<vehicle_gps_position_s> _sub_gps;

	// publications
	uORB::Publication<vehicle_local_position_s> _pub_lpos;
	uORB::Publication<vehicle_global_position_s> _pub_gpos;
	uORB::Publication<vehicle_attitude_s> _pub_att;
	uORB::Publication<estimator_status_s> _pub_est;
	uORB::Publication<ekf2_innovations_s> _pub_innov;

	// misc
	px4_pollfd_struct_t _polls[n_poll];
	uint64_t _timeStamp;

	// private
	const static int n_W = 21;
	matrix::Vector<float, n_W> _W;
	matrix::Vector<float, n_x>_x;

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SYS_AUTOSTART>) _sys_autostart,   // example parameter
		(ParamFloat<px4::params::CEI_MAG_W>) _w_mag   // mag noise std.
	)

	bool array_finite(float *a, int n);
	void handle_correction(float *x, float *W, const char *msg);
	Quatf compute_quaternion();
	void handle_shadow();
};
