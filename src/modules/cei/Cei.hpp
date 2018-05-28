#include <px4_module_params.h>
#include <controllib/blocks.hpp>
#include <px4_posix.h>


// uORB Subscriptions
#include <uORB/Subscription.hpp>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_magnetometer.h>


// uORB publications
#include <uORB/topics/vehicle_local_position.h>


class Cei : public control::SuperBlock, public ModuleParams {
public:
	// constants
	enum {POLL_SENSORS, POLL_PARAM, n_poll};

	// methods
	Cei();
	void update();

private:
	// subscriptions
	uORB::Subscription<parameter_update_s> _sub_param_update;
	uORB::Subscription<sensor_combined_s> _sub_sensor;
	uORB::Subscription<vehicle_magnetometer_s> _sub_mag;
	uORB::Subscription<vehicle_gps_position_s> _sub_gps;

	// publications
	uORB::Publication<vehicle_local_position_s> _pub_lpos;
	
	// misc
	px4_pollfd_struct_t _polls[3];
	uint64_t _timeStamp;
};
