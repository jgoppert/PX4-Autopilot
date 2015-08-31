set(module qurt_px4_layer)
set(stack)
set(main)
set(SRCS
	px4_qurt_impl.cpp
	px4_qurt_tasks.cpp
	lib_crc32.c
	drv_hrt.c
	qurt_stubs.c
	)
add_library(${module} STATIC EXCLUDE_FROM_ALL ${SRCS})
add_dependencies(${module} qurt_px4_layer)
set_target_properties(${module} PROPERTIES COMPILE_FLAGS "-Os")

# vim: set noet fenc=utf-8 ft=cmake ff=unix sts=4 sw=4 ts=4 :
