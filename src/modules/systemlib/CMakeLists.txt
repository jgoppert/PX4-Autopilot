set(module systemlib)
set(stack)
set(main)
set(SRCS
	perf_counter.c
	param/param.c
	conversions.c
	cpuload.c
	pid/pid.c
	airspeed.c
	system_params.c
	mavlink_log.c
	rc_check.c
	otp.c
	board_serial.c
	pwm_limit/pwm_limit.c
	mcu_version.c
	bson/tinybson.c
	circuit_breaker.cpp
	circuit_breaker_params.c
	)

if (${OS} STREQUAL "nuttx")
	list(APPEND SRCS
		err.c
		printload.c
		up_cxxinitialize.c
		)

elseif (${OS} STREQUAL "qurt")
	list(APPEND SRCS
		hx_stream.c
		)

endif()

add_library(${module} STATIC EXCLUDE_FROM_ALL ${SRCS})
set_target_properties(${module} PROPERTIES
	COMPILE_FLAGS "-Os -Wno-sign-compare")

# vim: set noet fenc=utf-8 ft=cmake ff=unix sts=4 sw=4 ts=4 :
