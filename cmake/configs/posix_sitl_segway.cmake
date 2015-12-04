include(cmake/configs/posix_sitl_default.cmake)

list(APPEND config_module_list
	modules/segway
	)

set(config_sitl_rcS
	posix-configs/SITL/init/rcS_segway
	)
