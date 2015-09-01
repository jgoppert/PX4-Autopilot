set(module device)
set(stack)
set(main)

set(SRCS
	ringbuffer.cpp
	)

if(${TARGET_OS} STREQUAL "nuttx")
	list(APPEND SRCS
		device_nuttx.cpp
		cdev.cpp
		i2c_nuttx.cpp
		pio.cpp
		spi.cpp
		)
else()
	list(APPEND SRCS
		device_posix.cpp
		vdev.cpp
		vfile.cpp
		vdev_posix.cpp
		i2c_posix.cpp
		sim.cpp
	)
endif()

add_library(${module} STATIC EXCLUDE_FROM_ALL ${SRCS})

# vim: set noet fenc=utf-8 ft=cmake ff=unix sts=4 sw=4 ts=4 :
