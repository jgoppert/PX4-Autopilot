/**
 * @file cei_main.cpp
 * @author James Goppert <james.goppert@gmail.com>
 *
 * Interface to casadi estimators.
 */

#include <px4_log.h>
#include <px4_module.h>
#include <px4_tasks.h>

#include "Cei.hpp"

extern "C" __EXPORT int cei_main(int argc, char *argv[]);

class CeiModule : public ModuleBase<CeiModule>
{
public:
	virtual ~CeiModule() = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static CeiModule *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

private:
	Cei _estimator;
};

int CeiModule::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Attitude and position estimator using an Extended Kalman Filter.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("cei", "estimator");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int CeiModule::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}


int CeiModule::task_spawn(int argc, char *argv[])
{
		_task_id = px4_task_spawn_cmd("cei",
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_ESTIMATOR,
						 7900,
						 (px4_main_t)&run_trampoline,
						 (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

CeiModule *CeiModule::instantiate(int argc, char *argv[])
{
	CeiModule *instance = new CeiModule();

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

void CeiModule::run()
{
	while (!should_exit()) {
		_estimator.update();
	}
}


int cei_main(int argc, char *argv[])
{
	return CeiModule::main(argc, argv);
}
