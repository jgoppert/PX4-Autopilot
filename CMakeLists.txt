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

#=============================================================================
#		Constants
set(VERSION_MAJOR 0)
set(VERSION_MINOR 1)
set(VERSION_PATCH 2)
set(PACKAGE_CONTACT "px4users@googlegroups.com")
set(nuttx_configs px4fmu-v2)

#=============================================================================
#		Computed parameters
#
set(VERSION "${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH}")
set(POSIX_APPS_HEADER ${CMAKE_BINARY_DIR}/apps.h)
set(MSG_OUT_PATH ${CMAKE_BINARY_DIR}/src/modules/uORB/topics)
set(TARGET_NAME "${TARGET_OS}-${TARGET_BOARD}-${TARGET_LABEL}")
set(NUTTX_EXPORT_DIR ${CMAKE_BINARY_DIR}/${TARGET_BOARD}/NuttX/nuttx-export)
string(TOUPPER ${TARGET_BOARD} BOARD_UPPER)
string(REPLACE "-" "_" BOARD_CONFIG ${BOARD_UPPER})

#=============================================================================
#		Config validation
#
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

##############################################################################
#	CMake Packages/ Modules
#
list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)
enable_testing()
include(CMakeParseArguments)
include(px4_utils)

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
add_git_submodule(nuttx NuttX)
add_git_submodule(genmsg Tools/genmsg)
add_git_submodule(gencpp Tools/gencpp)
add_git_submodule(gtest unittests/gtest)
add_git_submodule(eigen src/lib/eigen) 

##############################################################################
#	Programs
#
find_program(OPENOCD openocd HINT ../../sat/bin/)
find_package(PythonInterp REQUIRED)

##############################################################################
#	NuttX
#

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

set(WARNINGS
    -Wall
    -Wextra
    -Wdouble-promotion
    -Wshadow
    -Wfloat-equal
    -Wframe-larger-than=1024
    -Wpointer-arith
    -Wlogical-op
    -Wmissing-declarations
    -Wpacked
    -Wno-unused-parameter
    -Werror=format-security
    -Werror=array-bounds
    -Wfatal-errors
    -Wformat=1
    -Werror=unused-but-set-variable
    -Werror=unused-variable
    -Werror=double-promotion
    -Werror=reorder
	-Werror=uninitialized
	-Werror=init-self 
    #-Wcast-qual  - generates spurious noreturn attribute warnings,
    #               try again later
    #-Wconversion - would be nice, but too many "risky-but-safe"
    #               conversions in the code
    #-Wcast-align - would help catch bad casts in some cases,
    #               but generates too many false positives
    )

set(OPT_FLAGS
	-Os -g3
	)

#=============================================================================
#		c flags
#
set(C_WARNINGS
	-Wbad-function-cast
	-Wstrict-prototypes
	-Wold-style-declaration
	-Wmissing-parameter-type
	-Wmissing-prototypes
	-Wnested-externs
	)
set(C_FLAGS
	-std=gnu99
	-fno-common
	)

#=============================================================================
#		cxx flags
#
set(CXX_WARNINGS
	-Wno-missing-field-initializers
	)
set(CXX_FLAGS
	-fno-exceptions
	-fno-rtti
	-std=gnu++0x
	-fno-threadsafe-statics
	-DCONFIG_WCHAR_BUILTIN
	-D__CUSTOM_FILE_IO__
	)

add_definitions(
	-DCONFIG_ARCH_BOARD_${BOARD_CONFIG}
	)

#=============================================================================
#		ld flags
#
set(LD_FLAGS
	-Wl,--warn-common
	-Wl,--gc-sections
	)

#=============================================================================
#		misc flags
#
set(VISIBILITY_FLAGS
	-fvisibility=hidden
	"-include ${CMAKE_SOURCE_DIR}/src/include/visibility.h"
	)
set(EXE_LINK_FLAGS)

#=============================================================================
#		os/board specific flags
#
if (${TARGET_OS} STREQUAL "nuttx")
	include_directories(
		${NUTTX_EXPORT_DIR}/include
		${NUTTX_EXPORT_DIR}/include/cxx
		${NUTTX_EXPORT_DIR}/arch/chip
		${NUTTX_EXPORT_DIR}/arch/common
		)
	add_definitions(
		-D__PX4_NUTTX
		)
elseif(${TARGET_OS} STREQUAL "posix")
	include_directories(
		src/platforms/posix/include
		)
	add_definitions(
		-D__PX4_POSIX
		-D__PX4_LINUX
		"-Dnoreturn_function=__attribute__\(\(noreturn\)\)"
		-DCLOCK_MONOTONIC=1
		)
endif()

if (${CMAKE_SYSTEM_NAME} STREQUAL "Arm")
	if ("${TARGET_BOARD}" STREQUAL "px4fmu-v2")
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

#=============================================================================
#		set cmake flags
#
join(OUT CMAKE_C_FLAGS
	LIST
		${CMAKE_C_FLAGS}
		${C_FLAGS}
		${WARNINGS}
		${C_WARNINGS}
		${OPT_FLAGS}
		${VISIBILITY_FLAGS}
	GLUE " ")

join(OUT CMAKE_CXX_FLAGS
	LIST
		${CMAKE_CXX_FLAGS}
		${CXX_FLAGS}
		${WARNINGS}
		${CXX_WARNINGS}
		${OPT_FLAGS}
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

if ("${CMAKE_SYSTEM_NAME}" STREQUAL "Arm")
	list(APPEND module_directories
		./src/drivers/stm32
	)
endif()

if (${TARGET_OS} STREQUAL "nuttx")
	list(APPEND module_directories
		./src/platforms/nuttx
		./src/platforms/nuttx/px4_layer
	)

elseif (${TARGET_OS} STREQUAL "posix")
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

link_directories(${NUTTX_EXPORT_DIR}/libs)
set(installed_targets)

if ("${TARGET_OS}" STREQUAL "posix")
	add_executable(main ./src/platforms/posix/main.cpp)
	target_link_libraries(main ${module_list} pthread)


elseif ("${TARGET_OS}" STREQUAL "nuttx")
	message(STATUS "modules: ${module_list}")
	add_executable(main ./cmake/builtin_commands.c)

	add_dependencies(main NuttX_${TARGET_BOARD})
	target_link_libraries(main
		apps
		nuttx
		${module_list}
		c
		m
		nuttx
		)

	# startup library
	file(GLOB STARTUP_OBJS ${NUTTX_EXPORT_DIR}/startup/*.o)
	if(STARTUP_OBJS)
		message(STATUS startup objects: ${STARTUP_OBJS})
		add_library(startup EXCLUDE_FROM_ALL STATIC
			${STARTUP_OBJS})
		add_dependencies(startup NuttX_${TARGET_BOARD})
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
	if ("${TARGET_BOARD}" STREQUAL "sitl")
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
