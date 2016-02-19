include(cmake/configs/nuttx_px4fmu-v2_default.cmake)

list(REMOVE_ITEM config_module_list
	modules/position_estimator_inav
	)

list(APPEND config_module_list
	modules/local_position_estimator
	)
