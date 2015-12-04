include(cmake/configs/nuttx_px4fmu-v2_default.cmake)

list(APPEND config_module_list
    drivers/roboclaw
	modules/segway
	)
