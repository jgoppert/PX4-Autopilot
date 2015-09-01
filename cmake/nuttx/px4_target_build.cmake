include(nuttx_utils)

add_git_submodule(nuttx NuttX)

set(nuttx_configs px4fmu-v2)

#=============================================================================
#		Computed target parameters
#
set(NUTTX_EXPORT_DIR ${CMAKE_BINARY_DIR}/${TARGET_BOARD}/NuttX/nuttx-export)

##############################################################################
#	Required Programs
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
#		OS Specific flags
#
include_directories(${CMAKE_BINARY_DIR}/${TARGET_BOARD}/NuttX/apps/nshlib)


#=============================================================================
#		Manage exports
#
add_custom_target(link_exports)
foreach(config ${nuttx_configs})
    add_nuttx_export(${config})
    add_dependencies(link_exports link_export_${config})
endforeach()

#=============================================================================
#		os/board specific flags
#

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
		list(APPEND module_directories
			./src/drivers/boards/px4fmu-v2
			./src/drivers/stm32
			)
	endif()
endif()

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

message(STATUS "modules: ${module_list}")
add_executable(main ./cmake/builtin_commands.c)
add_dependencies(main NuttX_${TARGET_BOARD})

set(main_link_list
	${EXE_LINK_LIBS}
	${module_list}
	${EXE_LINK_LIBS}
	)

##############################################################################
#	px4_add_build_targets
#
# This is a required function for each target OS
#
function(px4_add_targets module_list)
	target_link_libraries(main
		${main_link_list}
		${main_link_list} # link twice to handle library dependencies
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
endfunction()

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

