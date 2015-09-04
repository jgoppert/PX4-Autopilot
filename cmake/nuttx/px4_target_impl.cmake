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

#=============================================================================
# FILE: nuttx/px4_target_impl.cmake
#
# Each PX4 target OS must implement the cmake/${OS}/px4_target_impl.cmake
# rules for their target that implement the following macros:
#
#	px4_target_set_flags
#	px4_target_validate_config
#	px4_target_firmware
#	px4_target_rules
#	px4_target_testing
#
# The macros are called from the top level CMakeLists.txt
#
include(nuttx_utils)

set(nuttx_configs px4fmu-v2)

set(NUTTX_EXPORT_DIR ${CMAKE_BINARY_DIR}/${BOARD}/NuttX/nuttx-export)

add_git_submodule(eigen src/lib/eigen)

macro(px4_target_set_flags)
	include_directories(
		${NUTTX_EXPORT_DIR}/include
		${NUTTX_EXPORT_DIR}/include/cxx
		${NUTTX_EXPORT_DIR}/arch/chip
		${NUTTX_EXPORT_DIR}/arch/common
		)
	add_definitions(
		-D__PX4_NUTTX
		)
	list(APPEND C_FLAGS
		-nodefaultlibs
		-nostdlib
		)
	list(APPEND CXX_FLAGS
		-nodefaultlibs
		-nostdlib
		)
	list(APPEND EXE_LINK_LIBS
		apps
		nuttx
		nosys
		m
		gcc
		)

	if (${CMAKE_SYSTEM_NAME} STREQUAL "Arm")
		if ("${BOARD}" STREQUAL "px4fmu-v2")
			set(ARM_BUILD_FLAGS
				-mcpu=cortex-m4
				-mthumb
				-march=armv7e-m
				-mfpu=fpv4-sp-d16
				-mfloat-abi=hard
				)
			list(APPEND C_FLAGS ${ARM_BUILD_FLAGS})
			list(APPEND CXX_FLAGS ${ARM_BUILD_FLAGS})
		endif()
	endif()
endmacro()

macro(px4_target_modules)
	# Include the target config file
	include(${TARGET_NAME})
endmacro()

macro(px4_target_validate_config)
	if(NOT ${CMAKE_SYSTEM_NAME} STREQUAL "Arm")
		if (${BOARD} STREQUAL "px4fmu-v2")
			message(FATAL_ERROR "Can only build ${TARGET_NAME} on arm")
		endif()
	endif()

	if(NOT EXISTS ${CMAKE_SOURCE_DIR}/cmake/${OS}/${TARGET_NAME}.cmake)
		message(FATAL_ERROR "not implemented yet: ${TARGET_NAME}")
	endif()
endmacro()

macro(px4_target_firmware)
	set(installed_targets)
	message(STATUS "modules: ${module_list}")
	link_directories(${NUTTX_EXPORT_DIR}/libs)

	add_executable(main ./cmake/builtin_commands.c)
	add_dependencies(main NuttX_${BOARD})

	set(main_link_list
		${module_list}
		${EXE_LINK_LIBS}
		)

	target_link_libraries(main
		${main_link_list}
		${main_link_list}
		${main_link_list}
		)

	# startup library
	file(GLOB STARTUP_OBJS ${NUTTX_EXPORT_DIR}/startup/*.o)
	if(STARTUP_OBJS)
		message(STATUS startup objects: ${STARTUP_OBJS})
		add_library(startup EXCLUDE_FROM_ALL STATIC
			${STARTUP_OBJS})
		add_dependencies(startup NuttX_${BOARD})
		set_source_files_properties(${STARTUP_OBJS}
			PROPERTIES
			# Identifies this as an object file
			EXTERNAL_OBJECT TRUE
			# Avoids need for file to exist at configure-time
			GENERATED TRUE
			)
		set_target_properties(startup PROPERTIES
			LINKER_LANGUAGE C
			)
		target_link_libraries(main startup)
	endif()

	# generate the firmware
	if ("${BOARD}" STREQUAL "sitl")
		target_link_libraries(main pthread z)
	else()
		set(MAIN_LINK_FLAGS
			"-T${NUTTX_EXPORT_DIR}/build/ld.script"
			"-Wl,-Map=${CMAKE_BINARY_DIR}/main.map"
			)
		join(OUT MAIN_LINK_FLAGS LIST ${MAIN_LINK_FLAGS} GLUE " ")
		set_target_properties(main PROPERTIES LINK_FLAGS ${MAIN_LINK_FLAGS})
		generate_firmware(${TARGET_NAME})
	endif()
	list(APPEND installed_targets main)
endmacro()

macro(px4_target_rules)
	##############################################################################
	#	submodules
	#
	add_git_submodule(nuttx NuttX)

	##############################################################################
	#	Programs
	#
	find_program(OPENOCD openocd HINT ../../sat/bin/)

	#=============================================================================
	#		Patching
	#
	add_custom_target(nuttx_patch)
	file(GLOB NUTTX_PATCHES RELATIVE ${CMAKE_SOURCE_DIR}
	    ${CMAKE_SOURCE_DIR}/nuttx-patches/*.patch)
	foreach(PATCH IN LISTS NUTTX_PATCHES)
	    string(REPLACE "/" "_" PATCH_NAME ${PATCH})
	    message(STATUS nuttx-patch: ${PATCH})
	    add_custom_command(OUTPUT nuttx_patch_${PATCH_NAME}.stamp
		COMMAND patch -p0 -N  < ${CMAKE_SOURCE_DIR}/${PATCH}
		COMMAND touch nuttx_patch_${PATCH_NAME}.stamp
		)
	    add_custom_target(nuttx_patch_${PATCH_NAME}
		DEPENDS nuttx_patch_${PATCH_NAME}.stamp git_nuttx)
	    add_dependencies(nuttx_patch nuttx_patch_${PATCH_NAME})
	endforeach()
	add_dependencies(nuttx_patch git_nuttx)

	#=============================================================================
	#		Manage exports
	#
	add_custom_target(link_exports)
	foreach(config ${nuttx_configs})
	    add_nuttx_export(${config})
	    add_dependencies(link_exports link_export_${config})
	endforeach()

endmacro()

macro(px4_target_testing)
endmacro()
