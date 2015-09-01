set(module platform_nuttx)
set(stack)
set(main)
set(SRCS
	px4_nuttx_impl.cpp
	)
add_library(${module} STATIC EXCLUDE_FROM_ALL ${SRCS})
set_target_properties(${module} PROPERTIES
	COMPILE_FLAGS "-Os")
add_dependencies(${module} NuttX_${BOARD})

# vim: set noet fenc=utf-8 ft=cmake ff=unix sts=4 sw=4 ts=4 :
