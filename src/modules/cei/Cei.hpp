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


class Cei : public control::SuperBlock, public ModuleParams
{
public:
	// constants
	enum {POLL_SENSORS, POLL_PARAM, n_poll};
	enum {X_x, X_y, X_z, X_vx, X_vy, X_vz, n_x};

	// methods
	Cei();
	~Cei();

	void update();
	void status();

	const float &x(int i)
	{
		return _x[i];
	}

	// get variance from upper triangular P
	const float &var(int16_t i)
	{
		return _PU[_lu_index(i)];
	}

	// get variance from upper triangular P
	double var_d(int16_t i)
	{
		return double(var(i));
	}

	// get variance from upper triangular P
	double x_d(int16_t i)
	{
		return double(x(i));
	}


private:

	perf_counter_t _perf_elapsed;
	perf_counter_t _perf_interval;
	perf_counter_t _perf_count;


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
	float _PU[21];
	float _x[6];

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SYS_AUTOSTART>) _sys_autostart   // example parameter
	)

	int _lu_index(int i)
	{
		return (i + 1) * (i + 2) / 2 - 1;
	}

	bool array_finite(float *a, int n)
	{
		for (int i = 0; i < n; i++) {
			if (!PX4_ISFINITE(a[i])) {
				return false;
			}
		}

		return true;
	}
};
