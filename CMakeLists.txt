##############################################################################
# PX4 Autopilot Project
#
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(px4 CXX C ASM)

if (NOT ${CMAKE_VERSION} VERSION_LESS 3.0.0)
	cmake_policy(SET CMP0046 NEW) # no non-existent targets as dependencies
endif()

##############################################################################
#	Parameters
#
#=============================================================================
#		Build options
#
set(NUTTX_BUILD_THREADS "4" CACHE STRING
	"number of threads to use when building NuttX")
set(TARGET_OS "nuttx" CACHE STRING "desired operating system")
set_property(CACHE TARGET_OS PROPERTY STRINGS nuttx posix qurt)

set(TARGET_BOARD "px4fmu-v2" CACHE STRING "target board")
set_property(CACHE TARGET_BOARD PROPERTY STRINGS px4fmu-v2 sitl)

set(TARGET_LABEL "simple" CACHE STRING "module set label")
set_property(CACHE TARGET_LABEL PROPERTY STRINGS simple default)

set(PYTHONPATH "${CMAKE_SOURCE_DIR}/Tools/genmsg/src:${CMAKE_SOURCE_DIR}/Tools/gencpp/src:$ENV{PYTHONPATH}" CACHE INTERNAL "PYTHONPATH" )

if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE Debug "Choose type type of build" FORCE)
	set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS Release Debug RelWithDebInfo MinSizeRel)
endif()

# Include the target specific build rules
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake
		${CMAKE_SOURCE_DIR}/cmake/common
		${CMAKE_SOURCE_DIR}/cmake/${TARGET_OS})

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
set(MSG_OUT_PATH ${CMAKE_BINARY_DIR}/src/modules/uORB/topics)
set(TARGET_NAME "${TARGET_OS}-${TARGET_BOARD}-${TARGET_LABEL}")
string(TOUPPER ${TARGET_BOARD} BOARD_UPPER)
string(REPLACE "-" "_" BOARD_CONFIG ${BOARD_UPPER})

##############################################################################
#	CMake Packages/ Modules
#
enable_testing()
include(CMakeParseArguments)
include(px4_utils)

#=============================================================================
#		Config validation
#
include(px4_target_config)
px4_verify_targets()

##############################################################################
#	GIT
#

include(px4_common_submodules)

##############################################################################
#	Programs
#
find_package(PythonInterp REQUIRED)

##############################################################################
#	Target specific build instructions
#
include(px4_target_build)

##############################################################################
#	Code Generation
#
# Source code generation is conducted here.
#
include(px4_common_messages)

##############################################################################
#	Build Flags
# 
# These are the project build flags.
#
#=============================================================================
#		generic flags
#
include(px4_common_flags)

#=============================================================================
#		os/board specific flags
#
include(px4_target_build_flags)

#=============================================================================
#		os/board specific modules
#
include(px4_target_modules)

#=============================================================================
#		set cmake flags
#
join(OUT CMAKE_C_FLAGS
	LIST
		${CMAKE_C_FLAGS}
		${C_FLAGS}
		${WARNINGS}
		${C_WARNINGS}
		${MAX_OPTIMIZATION}
		${OPTIMIZATION_FLAGS}
		${VISIBILITY_FLAGS}
	GLUE " ")

join(OUT CMAKE_CXX_FLAGS
	LIST
		${CMAKE_CXX_FLAGS}
		${CXX_FLAGS}
		${WARNINGS}
		${CXX_WARNINGS}
		${MAX_OPTIMIZATION}
		${OPTIMIZATION_FLAGS}
		${VISIBILITY_FLAGS}
	GLUE " ")

join(OUT CMAKE_EXE_LINKER_FLAGS
	LIST ${CMAKE_EXE_LINKER_FLAGS} ${LD_FLAGS} ${EXE_LINK_FLAGS}
	GLUE " ")

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
#		add required modules to build
#
# The following include file will set ${module_list}
include(px4_common_modules)


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

px4_add_targets(${module_list})

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


if (NOT ${TARGET_OS} STREQUAL "nuttx")

	add_test(test_px4_simple_app ${PYTHON_EXECUTABLE} ${CMAKE_SOURCE_DIR}/cmake/test_compare.py
		--command ${CMAKE_BINARY_DIR}/main
		--stdout ${CMAKE_BINARY_DIR}/test/px4_simple_app_output.txt
		--stdin ${CMAKE_SOURCE_DIR}/cmake/test/px4_simple_app_input.txt
		--check ${CMAKE_SOURCE_DIR}/cmake/test/px4_simple_app_correct.txt
		--start 4 --stop -1
		)

endif()

##############################################################################
#	Packaging
#
# Important to having packaging at end of cmake file.
#
set(CPACK_PACKAGE_VERSION ${VERSION})
set(CPACK_PACKAGE_CONTACT ${PACKAGE_CONTACT})
include(CPack)

# vim: set noet fenc=utf-8 ff=unix sts=4 sw=4 ts=4 :
