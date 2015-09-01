set(module stm32_drivers)
set(SRCS
	drv_hrt.c
	drv_pwm_servo.c
	)
include_directories(
	${CMAKE_SOURCE_DIR}/NuttX/arch/arm/src/stm32
	${CMAKE_SOURCE_DIR}/NuttX/arch/arm/src/common
	)
add_library(${module} STATIC EXCLUDE_FROM_ALL ${SRCS})
set_target_properties(${module} PROPERTIES
	COMPILE_FLAGS "-Os")
add_dependencies(${module} NuttX_${BOARD} msg_files msg_files_multi)

# vim: set noet fenc=utf-8 ft=cmake ff=unix sts=4 sw=4 ts=4 :
