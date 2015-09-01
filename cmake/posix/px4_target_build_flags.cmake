include_directories(
	src/platforms/posix/include
	)
add_definitions(
	-D__PX4_POSIX
	-D__PX4_LINUX
	"-Dnoreturn_function=__attribute__\(\(noreturn\)\)"
	-DCLOCK_MONOTONIC=1
	)
list(APPEND EXE_LINK_LIBS
	pthread
	)
