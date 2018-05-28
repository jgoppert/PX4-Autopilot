include(cmake/configs/posix_sitl_default.cmake)

list(REMOVE_ITEM config_module_list
	modules/ekf2
	modules/local_position_estimator
	modules/attitude_estimator_q
	)

set(config_sitl_rcS_dir
	posix-configs/SITL/init/cei
	)
