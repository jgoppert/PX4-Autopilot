
#=============================================================================
#              Config validation
#
function(px4_verify_targets)
	if(${CMAKE_SYSTEM_NAME} STREQUAL "Arm")
	else()
	       if (${TARGET_BOARD} STREQUAL "px4fmu-v2")
		       message(FATAL_ERROR "Can only build ${TARGET_NAME} on arm")
	       endif()
	endif()

	if (${TARGET_NAME} STREQUAL "nuttx-px4fmu-v2-simple")
	elseif (${TARGET_NAME} STREQUAL "posix-sitl-simple")
	else()
	       message(FATAL_ERROR "not implemented yet: ${TARGET_NAME}")
	endif()
endfunction()
