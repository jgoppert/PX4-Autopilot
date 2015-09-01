set(module platform_common)
set(stack)
set(main)
set(SRCS
	px4_getopt.c
	)
add_library(${module} STATIC EXCLUDE_FROM_ALL ${SRCS})
add_dependencies(${module} msg_files msg_files_multi)

if (${OS} STREQUAL "nuttx")
	add_dependencies(${module} NuttX_${BOARD})
elseif (${OS} STREQUAL "posix")
	add_dependencies(${module} posix_apps)
endif()

# vim: set noet fenc=utf-8 ft=cmake ff=unix sts=4 sw=4 ts=4 :
