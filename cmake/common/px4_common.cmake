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
# FILE: px4_common.cmake
#
# Common PX4 cmake macros
#
# Defined macros:
#
# 	px4_common_set_flags
#	px4_common_set_modules
#	px4_common_generate_messages
#	px4_common_modules
#	px4_common_git_submodules
#

function(px4_common_set_flags WARNINGS OPTIMIZATION_FLAGS C_WARNINGS C_FLAGS CXX_WARNINGS CXX_FLAGS LD_FLAGS VISIBILITY_FLAGS)

	message(STATUS "Running px4_common_set_flags")
	include_directories(
		src
		${CMAKE_BINARY_DIR}/src
		src/modules
		src/include
		src/lib
		src/platforms
		# TODO Build/versioning was in Makefile, do we need this, how does it work with cmake
		src/drivers/boards/${BOARD}
		src/lib/eigen
		${CMAKE_BINARY_DIR}
		${CMAKE_BINARY_DIR}/src/modules/px4_messages
		${CMAKE_BINARY_DIR}/src/modules
		mavlink/include/mavlink
		)

	set(${WARNINGS}
		-Wall
		-Wno-sign-compare
		-Wextra
		#-Wshadow # very verbose due to eigen
		-Wfloat-equal
		-Wframe-larger-than=1024
		-Wpointer-arith
		-Wmissing-declarations
		-Wpacked
		-Wno-unused-parameter
		-Werror=format-security
		-Werror=array-bounds
		-Wfatal-errors
		-Werror=unused-variable
		-Werror=reorder
		-Werror=uninitialized
		-Werror=init-self 
		#-Wcast-qual  - generates spurious noreturn attribute warnings,
		#               try again later
		#-Wconversion - would be nice, but too many "risky-but-safe"
		#               conversions in the code
		#-Wcast-align - would help catch bad casts in some cases,
		#               but generates too many false positives
		PARENT_SCOPE
		)

	if (NOT "${CMAKE_C_COMPILER_ID}" STREQUAL "Clang")
		list(APPEND ${WARNINGS}
			-Werror=unused-but-set-variable
			-Wformat=1
			#-Wlogical-op # very verbose due to eigen
			-Wdouble-promotion
			-Werror=double-promotion
			PARENT_SCOPE
		)
	endif()
	

	set(${OPTIMIZATION_FLAGS}
		-fno-strict-aliasing
		-fomit-frame-pointer
		-funsafe-math-optimizations
		-ffunction-sections
		-fdata-sections
		PARENT_SCOPE
		)
	if (NOT "${CMAKE_C_COMPILER_ID}" STREQUAL "Clang")
		list(APPEND ${OPTIMIZATION_FLAGS}
			-fno-strength-reduce
			-fno-builtin-printf
			PARENT_SCOPE
		)
	endif()

	#=============================================================================
	#		c flags
	#
	set(${C_WARNINGS}
		-Wbad-function-cast
		-Wstrict-prototypes
		-Wmissing-prototypes
		-Wnested-externs
		PARENT_SCOPE
		)

	if (NOT "${CMAKE_C_COMPILER_ID}" STREQUAL "Clang")
		list(APPEND ${C_WARNINGS}
			-Wold-style-declaration
			-Wmissing-parameter-type
			PARENT_SCOPE
		)
	endif()

	set(${C_FLAGS}
		-std=gnu99
		-fno-common
		PARENT_SCOPE
		)

	#=============================================================================
	#		cxx flags
	#
	set(${CXX_WARNINGS}
		-Wno-missing-field-initializers
		PARENT_SCOPE
		)
	set(${CXX_FLAGS}
		-fno-exceptions
		-fno-rtti
		-std=gnu++0x
		-fno-threadsafe-statics
		-DCONFIG_WCHAR_BUILTIN
		-D__CUSTOM_FILE_IO__
		PARENT_SCOPE
		)

	add_definitions(
		-DCONFIG_ARCH_BOARD_${BOARD_CONFIG}
		)

	#=============================================================================
	#		ld flags
	#
	set(${LD_FLAGS}
		-Wl,--warn-common
		-Wl,--gc-sections
		PARENT_SCOPE
		)

	#=============================================================================
	#		misc flags
	#
	set(${VISIBILITY_FLAGS}
		-fvisibility=hidden
		"-include ${CMAKE_SOURCE_DIR}/src/include/visibility.h"
		PARENT_SCOPE
		)
endfunction()

function(px4_common_set_modules module_directories)
	message(STATUS "Running px4_common_set_modules")
	#=============================================================================
	#		Common Modules
	#
	set(module_directories
		./src/drivers/led
		./src/drivers/device
		./src/platforms/common
		./src/modules/systemlib
		./src/modules/systemlib/mixer
		./src/modules/uORB
		./src/examples/px4_simple_app
		#./src/systemcmds/perf
		./src/lib/mathlib/math/filter
		./src/lib/conversion
		PARENT_SCOPE
		)
endfunction()

function(px4_common_generate_messages OUT_MSG_FILES OUT_MSG_MULTI_FILES)
	message(STATUS "Running px4_common_generate_messages")
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
	set(${OUT_MSG_FILES} ${MSG_FILES_OUT} PARENT_SCOPE)

	#=============================================================================
	#		multi messages for target OS
	#
	set(MSG_MULTI_OUT_PATH
		${CMAKE_BINARY_DIR}/src/platforms/${OS}/px4_messages)
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
			-t ${CMAKE_BINARY_DIR}/multi_topics_temporary/${OS}
			-p "px4_"
		DEPENDS git_genmsg git_gencpp ${MSG_FILES_IN}
		WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
		COMMENT "Generating uORB topic multi headers for ${OS}"
		VERBATIM
		)
endfunction()

#=============================================================================
# Function px4_common_modules
#
# Inputs:
#	CMAKE_BINARY_DIR
#	module_directories
#
# Outputs:
#	out_module_list
#
function(px4_common_modules out_module_list)
	set(${out_module_list} PARENT_SCOPE)
	message(STATUS "Running px4_common_modules")
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
		list(APPEND ${out_module_list} ${module} PARENT_SCOPE)
		#message(STATUS "module: ${module}\n\tstack: ${module_stack}\n\tmain: ${module_main}")
		if(NOT "${module_main}" STREQUAL "")
			file(WRITE "${CMAKE_BINARY_DIR}/builtin_commands/COMMAND.${module}.${module_priority}.${module_stack}.${module_main}")
		endif()
	endforeach()
endfunction()

