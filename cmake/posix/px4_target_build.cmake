#=============================================================================
#		Computed target parameters
#
set(POSIX_APPS_HEADER ${CMAKE_BINARY_DIR}/apps.h)

#=============================================================================
#		apps
#
add_custom_command(OUTPUT ${POSIX_APPS_HEADER}
	COMMAND PYTHONPATH=${PYTHONPATH} ${PYTHON_EXECUTABLE} 
		${CMAKE_SOURCE_DIR}/Tools/posix_apps.py > ${POSIX_APPS_HEADER}
	COMMENT "Generating posix apps"
	VERBATIM
	)

add_custom_target(posix_apps DEPENDS ${POSIX_APPS_HEADER})

# The function px4_add_targets must be defined
function(px4_add_targets)
	add_executable(main ./src/platforms/posix/main.cpp)
	foreach(module ${module_list})
		message("px4_add_targets " ${module})
	endforeach()
	target_link_libraries(main ${module_list} ${EXE_LINK_LIBS})
endfunction()

