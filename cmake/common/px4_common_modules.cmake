# Inputs:
#	CMAKE_BINARY_DIR
#	module_directories
#
# Outputs:
#	module_list
#	platform_common (deps)
#
set(module_list)
file(REMOVE ${CMAKE_BINARY_DIR}/builtin_commands)
file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/builtin_commands)
foreach(module_directory ${module_directories})
	message(STATUS ${module_directory})
	add_subdirectory(${module_directory})
	# get the updated module list
	get_directory_property(module
		DIRECTORY ${module_directory} DEFINITION module)
	get_directory_property(module_stack
		DIRECTORY ${module_directory} DEFINITION stack)
	get_directory_property(module_main
		DIRECTORY ${module_directory} DEFINITION main)
	get_directory_property(module_priority
		DIRECTORY ${module_directory} DEFINITION priority)
	set(required_module_vars module)
	foreach(var ${required_module_vars})
		if (${var} STREQUAL "")
			message(FATAL_ERROR "${module_directory}/CMakeLists.txt does not define ${var}")
		endif()
	endforeach()
	if ("${module_priority}" STREQUAL "")
		set(module_priority SCHED_PRIORITY_DEFAULT)
	endif()
	if ("${module_stack}" STREQUAL "")
		set(module_stack 1024)
	endif()
	set(module_list ${module_list} 
			${module}
			)
	#message(STATUS "module: ${module}\n\tstack: ${module_stack}\n\tmain: ${module_main}")
	if(NOT "${module_main}" STREQUAL "")
		file(WRITE "${CMAKE_BINARY_DIR}/builtin_commands/COMMAND.${module}.${module_priority}.${module_stack}.${module_main}")
	endif()
	add_dependencies(${module} platform_common)
endforeach()
