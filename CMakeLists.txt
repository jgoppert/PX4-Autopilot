##############################################################################
# PX4 Autopilot Project
#
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(px4 CXX C ASM)

message(STATUS "Found ar: -${CMAKE_AR}-")

if (NOT ${CMAKE_VERSION} VERSION_LESS 3.0.0)
	cmake_policy(SET CMP0046 NEW) # no non-existent targets as dependencies
endif()

##############################################################################
#	CMake Packages/ Modules
#
list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)
enable_testing()
include(CMakeParseArguments)
include(px4_utils)

if (NOT NESTED_CMAKE_CALL)
  px4_add_target( qurt
                  ${CMAKE_SOURCE_DIR}/cmake/qurt-clang-hexagon.cmake 
                  "-DTARGET_OS=qurt;-DTARGET_BOARD=hil;-DTARGET_LABEL=simple"
                )

  add_custom_target( everything DEPENDS all qurt)
  add_custom_target( clean-all DEPENDS clean qurt-clean )
  return()
endif(NOT NESTED_CMAKE_CALL)


##############################################################################
#	Parameters
#
#=============================================================================
#		Build options
#
set(TARGET_LABEL "simple" CACHE STRING "module set label")
set_property(CACHE TARGET_LABEL PROPERTY STRINGS simple default)

set(PYTHONPATH "${CMAKE_SOURCE_DIR}/Tools/genmsg/src:${CMAKE_SOURCE_DIR}/Tools/gencpp/src:$ENV{PYTHONPATH}" CACHE INTERNAL "PYTHONPATH" )

if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE Debug "Choose type type of build" FORCE)
	set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS Release Debug RelWithDebInfo MinSizeRel)
endif()

#=============================================================================
#		Constants
set(VERSION_MAJOR 0)
set(VERSION_MINOR 1)
set(VERSION_PATCH 2)
set(PACKAGE_CONTACT "px4users@googlegroups.com")

#=============================================================================
#		Computed parameters
#
set(VERSION "${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH}")
set(POSIX_APPS_HEADER ${CMAKE_BINARY_DIR}/apps.h)
set(MSG_OUT_PATH ${CMAKE_BINARY_DIR}/src/modules/uORB/topics)
set(TARGET_NAME "${TARGET_OS}-${TARGET_BOARD}-${TARGET_LABEL}")
string(TOUPPER ${TARGET_BOARD} BOARD_UPPER)
string(REPLACE "-" "_" BOARD_CONFIG ${BOARD_UPPER})

if(NOT DEFAULT_VISIBILITY)
	set(DEFAULT_VISIBILITY "hidden")
else()
	set(DEFAULT_VISIBILITY "default")
endif()

#=============================================================================
#		Config validation
#
if (${TARGET_NAME} STREQUAL "qurt-hil-simple")
else()
	message(FATAL_ERROR "not implemented yet: ${TARGET_NAME}")
endif()

##############################################################################
#	GIT
#

# convenience target to nuke all submodules
add_custom_target(submodule_clean
	WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
	COMMAND git submodule deinit -f .
	COMMAND rm -rf .git/modules/*
	)

# add all git submodules and paths
add_git_submodule(mavlink mavlink/include/mavlink/v1.0)
add_git_submodule(genmsg Tools/genmsg)
add_git_submodule(gencpp Tools/gencpp)
add_git_submodule(gtest unittests/gtest)
add_git_submodule(eigen src/lib/eigen) 
if(${TARGET_OS} STREQUAL "qurt")
#add_git_submodule(dspal src/lib/dspal) 
endif()

##############################################################################
#	Programs
#
find_package(PythonInterp REQUIRED)

##############################################################################
#	Code Generation
#
# Source code generation is conducted here.
#
#=============================================================================
#		messages
#
file(GLOB_RECURSE MSG_FILES RELATIVE ${CMAKE_SOURCE_DIR}/msg/ *.msg)
list_prepend(
	OUT MSG_FILES_IN
	LIST ${MSG_FILES}
	PATH ${CMAKE_SOURCE_DIR}/msg/)
#message(STATUS "msg files in ${MSG_FILES_IN}")
list_prepend(
	OUT MSG_FILES_OUT
	LIST ${MSG_FILES}
	PATH ${MSG_OUT_PATH})
#message(STATUS "msg files out ${MSG_FILES_OUT}")
set(PYTHONPATH "${CMAKE_SOURCE_DIR}/Tools/genmsg/src:${CMAKE_SOURCE_DIR}/Tools/gencpp/src:$ENV{PYTHONPATH}")
add_custom_command(OUTPUT ${MSG_FILES_OUT}
	COMMAND PYTHONPATH=${PYTHONPATH} ${PYTHON_EXECUTABLE} 
		Tools/px_generate_uorb_topic_headers.py
		-d msg
		-o ${MSG_OUT_PATH} 
		-e msg/templates/uorb
		-t ${CMAKE_BINARY_DIR}/topics_temporary
	DEPENDS git_genmsg git_gencpp ${MSG_FILES_IN}
	WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
	COMMENT "Generating uORB topic headers"
	VERBATIM
	)
add_custom_target(msg_files DEPENDS ${MSG_FILES_OUT})

#=============================================================================
#		multi messages for target OS
#
set(MSG_MULTI_OUT_PATH
	${CMAKE_BINARY_DIR}/src/platforms/${TARGET_OS}/px4_messages)
list_prepend(
	OUT MSG_MULTI_FILES_OUT
	LIST ${MSG_FILES}
	PATH ${MSG_MULTI_OUT_PATH})
#message(STATUS "files out: ${MSG_MULTI_FILES_OUT}")
add_custom_command(OUTPUT ${MSG_MULTI_FILES_OUT}
	COMMAND PYTHONPATH=${PYTHONPATH} ${PYTHON_EXECUTABLE} 
		Tools/px_generate_uorb_topic_headers.py
		-d msg
		-o ${MSG_MULTI_OUT_PATH} 
		-e msg/templates/px4/uorb
		-t ${CMAKE_BINARY_DIR}/multi_topics_temporary/${TARGET_OS}
		-p "px4_"
	DEPENDS git_genmsg git_gencpp ${MSG_FILES_IN}
	WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
	COMMENT "Generating uORB topic multi headers for ${TARGET_OS}"
	VERBATIM
	)
add_custom_target(msg_files_multi DEPENDS ${MSG_MULTI_FILES_OUT})

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

##############################################################################
#	Build Flags
# 
# These are the project build flags.
#
#=============================================================================
#		generic flags
#
include_directories(
	src
	${CMAKE_BINARY_DIR}/src
	src/modules
	src/include
	src/lib
	src/platforms
	# TODO Build/versioning was in Makefile, do we need this, how does it work with cmake
	src/drivers/boards/${TARGET_BOARD}
	src/lib/eigen
	${CMAKE_BINARY_DIR}
	${CMAKE_BINARY_DIR}/src/modules/px4_messages
	${CMAKE_BINARY_DIR}/src/modules
	)

add_definitions(
	-DCONFIG_ARCH_BOARD_${BOARD_CONFIG}
	)

#=============================================================================
#		os/board specific flags
#
set(CMAKE_SYSTEM_VERSION 1)
if(${TARGET_OS} STREQUAL "posix")
	include_directories(
		src/platforms/posix/include
		)
	add_definitions(
		-D__PX4_POSIX
		-D__PX4_LINUX
		"-Dnoreturn_function=__attribute__\(\(noreturn\)\)"
		-DCLOCK_MONOTONIC=1
		)
	# Used to call CMAKE_SYSTEM_NAME-compiler-CMAKE_SYSTEM_PROCESSOR.cmake
	set(CMAKE_SYSTEM_NAME posix)
	set(CMAKE_SYSTEM_PROCESSOR native)
elseif(${TARGET_OS} STREQUAL "qurt")
	set(DSPAL_ROOT src/lib/dspal)
	include_directories(
		${DSPAL_ROOT}/include 
		${DSPAL_ROOT}/sys 
		${DSPAL_ROOT}/sys/sys 
		${DSPAL_ROOT}/mpu_spi/inc
		${DSPAL_ROOT}/uart_esc/inc
    		src/platforms/qurt/include
    		src/platforms/posix/include
		)
	add_definitions(
		-D__PX4_QURT -D__PX4_POSIX
    		-fvisibility=${DEFAULT_VISIBILITY} -include ${PX4_INCLUDE_DIR}visibility.h
		)
	# Used to call CMAKE_SYSTEM_NAME-compiler-CMAKE_SYSTEM_PROCESSOR.cmake
	set(CMAKE_SYSTEM_NAME qurt)
	set(CMAKE_SYSTEM_PROCESSOR hexagon)

	set(CMAKE_C_FLAGS ${QURT_CMAKE_C_FLAGS})
	set(CMAKE_CXX_FLAGS ${QURT_CMAKE_CXX_FLAGS})
	set(CMAKE_SHARED_LINKER_FLAGS "")

	message(STATUS "CMAKE_C_FLAGS: -${CMAKE_C_FLAGS}-")
	message(STATUS "CMAKE_CXX_FLAGS: -${CMAKE_CXX_FLAGS}-")

	# Clear -rdynamic flag which fails for hexagon
	set(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS "")
	set(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS "")
endif()

##############################################################################
#	Modules
#
# Modules are archives that are linked later by main.
# Each module is a process that runs on PX4.
# Interprocess communication is conducted using uORB.
#
# Note that not all modules are used on every board 
# configuration.
#

#=============================================================================
#		module directories, order determines linking order
#
set(module_directories)

if ("${TARGET_BOARD}" STREQUAL "px4fmu-v2")
	list(APPEND module_directories
		./src/drivers/boards/px4fmu-v2
		)
endif()

list(APPEND module_directories
	./src/drivers/led
	./src/drivers/device
	./src/modules/systemlib
	./src/modules/systemlib/mixer
	./src/platforms/common
	)

if (${TARGET_OS} STREQUAL "posix")
	list(APPEND module_directories
		#./src/platforms/posix/tests/hrt_test
		#./src/platforms/posix/tests/wqueue
		#./src/platforms/posix/tests/hello
		#./src/platforms/posix/tests/muorb
		#./src/platforms/posix/tests/vcdev_test
		./src/platforms/posix/px4_layer
		./src/platforms/posix/work_queue
		#./src/platforms/posix/drivers/adcsim
		#./src/platforms/posix/drivers/gpssim
		#./src/platforms/posix/drivers/tonealrmsim
		#./src/platforms/posix/drivers/accelsim
		#./src/platforms/posix/drivers/airspeedsim
		#./src/platforms/posix/drivers/barosim
		#./src/platforms/posix/drivers/gyrosim
		)
elseif (${TARGET_OS} STREQUAL "qurt")
	list(APPEND module_directories
		./src/platforms/qurt/px4_layer
		./src/platforms/posix/work_queue
		)
endif()

list(APPEND module_directories
	./src/examples/px4_simple_app
	./src/modules/uORB
	#./src/systemcmds/perf
	)

#=============================================================================
#		add required modules to build
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
	list(APPEND module_list ${module})
	#message(STATUS "module: ${module}\n\tstack: ${module_stack}\n\tmain: ${module_main}")
	if(NOT "${module_main}" STREQUAL "")
		file(WRITE "${CMAKE_BINARY_DIR}/builtin_commands/COMMAND.${module}.${module_priority}.${module_stack}.${module_main}")
	endif()
endforeach()

##############################################################################
#	Firmware configurations
#
# Defines arch, OS, and modules for each build.
# The main program is the entry point of the os at boot
# and acts as the user command line interface.
#

#=============================================================================
#	Build

set(installed_targets)

if ("${TARGET_OS}" STREQUAL "posix")
	add_executable(main ./src/platforms/posix/main.cpp)
	target_link_libraries(main ${module_list} pthread)


elseif ("${TARGET_OS}" STREQUAL "qurt")
	add_executable(main ./src/platforms/qurt/px4_layer/main.cpp)
	target_link_libraries(main ${module_list} pthread)
endif()

list(APPEND installed_targets main)

#=============================================================================
#	Install
install(TARGETS ${installed_targets}
	RUNTIME DESTINATION bin
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib/static
	)

##############################################################################
#	Testing
#

add_test(test_px4_simple_app ${PYTHON_EXECUTABLE} ${CMAKE_SOURCE_DIR}/cmake/test_compare.py
	--command ${CMAKE_BINARY_DIR}/main
	--stdout ${CMAKE_BINARY_DIR}/test/px4_simple_app_output.txt
	--stdin ${CMAKE_SOURCE_DIR}/cmake/test/px4_simple_app_input.txt
	--check ${CMAKE_SOURCE_DIR}/cmake/test/px4_simple_app_correct.txt
	--start 4 --stop -1
	)

##############################################################################
#	Packaging
#
# Important to having packaging at end of cmake file.
#
set(CPACK_PACKAGE_VERSION ${VERSION})
set(CPACK_PACKAGE_CONTACT ${PACKAGE_CONTACT})
include(CPack)

# vim: set noet fenc=utf-8 ff=unix sts=0 sw=4 ts=4 :
