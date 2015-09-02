############################################################################
#
# Copyright (c) 2015 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

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
set(OS "posix" CACHE STRING "desired operating system")
set_property(CACHE OS PROPERTY STRINGS nuttx posix qurt)

set(BOARD "sitl" CACHE STRING "target board")
set_property(CACHE BOARD PROPERTY STRINGS px4fmu-v2 sitl)

set(LABEL "simple" CACHE STRING "module set label")
set_property(CACHE LABEL PROPERTY STRINGS simple default)

set(PYTHONPATH "${CMAKE_SOURCE_DIR}/Tools/genmsg/src:${CMAKE_SOURCE_DIR}/Tools/gencpp/src:$ENV{PYTHONPATH}" CACHE INTERNAL "PYTHONPATH" )

if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE Debug "Choose type type of build" FORCE)
	set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS Release Debug RelWithDebInfo MinSizeRel)
endif()

# Include the target specific build rules
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake
		${CMAKE_SOURCE_DIR}/cmake/common
		${CMAKE_SOURCE_DIR}/cmake/${OS})

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
set(TARGET_NAME "${OS}-${BOARD}-${LABEL}")
string(TOUPPER ${BOARD} BOARD_UPPER)
string(REPLACE "-" "_" BOARD_CONFIG ${BOARD_UPPER})

##############################################################################
#	CMake Packages/ Modules
#
list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)
enable_testing()
include(CMakeParseArguments)
include(px4_utils)

# Include common PX4 rules
include(px4_common)

# Include the PX4 rules for the specific target
include(px4_target_impl)

#=============================================================================
#		Config validation
#
px4_target_validate_config()

##############################################################################
#	GIT
#
px4_common_git_submodules()

##############################################################################
#	Programs
#
find_package(PythonInterp REQUIRED)

##############################################################################
#	Target specific rules
#
px4_target_rules()

##############################################################################
#	Code Generation
#
# Source code generation is conducted here.
#
#=============================================================================
#		messages
#
px4_common_generate_messages()

##############################################################################
#	Build Flags
# 
# These are the project build flags.
#
#=============================================================================
#		generic flags
#
px4_common_set_flags()
px4_common_set_modules()
px4_target_set_modules()

#=============================================================================
#		os/board specific flags
#
px4_target_set_flags()

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
px4_common_modules()

##############################################################################
#	Firmware configurations
#
# Defines arch, OS, and modules for each build.
# The main program is the entry point of the os at boot
# and acts as the user command line interface.
#

#=============================================================================
#	Build

px4_target_firmware()

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
px4_target_testing()

##############################################################################
#	Packaging
#
# Important to having packaging at end of cmake file.
#
set(CPACK_PACKAGE_VERSION ${VERSION})
set(CPACK_PACKAGE_CONTACT ${PACKAGE_CONTACT})
include(CPack)

# vim: set noet fenc=utf-8 ff=unix sts=4 sw=4 ts=4 :
