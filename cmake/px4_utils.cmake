##############################################################################
#	Utility Functions
# 
# Functions have their own scope. Added targets are in the parent
# scope, as well as variables explicity set with PARENT_SCOPE.
#

function(add_git_submodule NAME PATH)
	string(REPLACE "/" "_" ${NAME} ${PATH})
	add_custom_command(OUTPUT ${CMAKE_BINARY_DIR}/git_${NAME}.stamp
		WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
		COMMAND git submodule init ${PATH}
		COMMAND git submodule update ${PATH}
		COMMAND touch ${CMAKE_BINARY_DIR}/git_${NAME}.stamp
		)
	add_custom_target(git_${NAME}
		DEPENDS git_${NAME}.stamp
)
endfunction()

function(function_parsing NAME FARGN)
	# This function simplifies interaction cmake parse args
	# to allow complex function parsing in cmake

	string(TOUPPER "${NAME}" NAME_UPPER)
	cmake_parse_arguments(${NAME_UPPER} "${options}"
		"${oneValueArgs}" "${multiValueArgs}" "${FARGN}")
	# check for required arguments
	foreach(arg ${requiredArgs})
		if (NOT DEFINED ${NAME_UPPER}_${arg})
			message(FATAL_ERROR "${NAME} requires argument ${arg}")
		endif()
	endforeach()
	# check for unparsed arguments
	if (NOT ${${NAME_UPPER}_UNPARSED_ARGUMENTS} STREQUAL "")
		message(FATAL_ERROR "${NAME} unparsed arguments"
		"${${NAME_UPPER}_UNPARSED_ARGUMENTS}")
	endif()
	# set variables without prefix
	foreach(val ${options} ${oneValueArgs} ${multiValueArgs})
		set(${val} ${${NAME_UPPER}_${val}} PARENT_SCOPE)
	endforeach()
endfunction(function_parsing)

function(function_args_default)
	set(options PARENT_SCOPE)
	set(oneValueArgs PARENT_SCOPE)
	set(multiValueArgs PARENT_SCOPE)
	set(requiredArgs PARENT_SCOPE)
endfunction(function_args_default)

function(list_prepend)
	#message(STATUS "${ARGN}")
	function_args_default()
	set(multiValueArgs LIST)
	set(oneValueArgs OUT PATH)
	set(requiredArgs LIST OUT PATH)
	function_parsing(list_prepend "${ARGN}")
	foreach(FILE ${LIST})
		list(APPEND ${OUT} ${PATH}/${FILE})
	endforeach()
	set(${OUT} ${${OUT}} PARENT_SCOPE)
endfunction(list_prepend)

function(join)
	function_args_default()
	set(oneValueArgs OUT GLUE)
	set(multiValueArgs LIST)
	set(requiredArgs LIST GLUE OUT)
	function_parsing(join "${ARGN}")
	string (REPLACE ";" "${GLUE}" _TMP_STR "${LIST}")
	set (${OUT} "${_TMP_STR}" PARENT_SCOPE)
endfunction(join)

function(generate_firmware NAME)
	add_custom_target(firmware_${NAME} DEPENDS ${NAME}.px4)
	add_custom_command(OUTPUT ${NAME}.px4
		COMMAND objcopy --output-format=binary ${NAME} ${NAME}.bin
		COMMAND python -u ${CMAKE_SOURCE_DIR}/Tools/px_mkfw.py
			--board_id 6 > ${NAME}_prototype.px4
		COMMAND python -u ${CMAKE_SOURCE_DIR}/Tools/px_mkfw.py
			--prototype ${NAME}_prototype.px4 --image ${NAME}.bin > ${NAME}.px4
			DEPENDS ${NAME})
endfunction()

function(add_nuttx_export BOARD)
	set(BOARD_NUTTX_SRC ${CMAKE_BINARY_DIR}/${BOARD}/NuttX)

	# copy
	add_custom_command(OUTPUT nuttx_copy_${BOARD}.stamp
		COMMAND mkdir -p ${CMAKE_BINARY_DIR}/${BOARD}
		COMMAND cp -r ${CMAKE_SOURCE_DIR}/NuttX ${BOARD_NUTTX_SRC}
		COMMAND rm -rf ${BOARD_NUTTX_SRC}/.git
		COMMAND touch nuttx_copy_${BOARD}.stamp)
	add_custom_target(nuttx_copy_${BOARD}
		DEPENDS nuttx_copy_${BOARD}.stamp nuttx_patch)

	# export
	add_custom_command(OUTPUT ${BOARD}.export
		COMMAND echo Configuring NuttX for ${BOARD}
		COMMAND make -C${BOARD_NUTTX_SRC}/nuttx -j${NUTTX_BUILD_THREADS}
			-r --quiet distclean
		COMMAND cp -r ${CMAKE_SOURCE_DIR}/nuttx-configs/${BOARD}
			${BOARD_NUTTX_SRC}/nuttx/configs
		COMMAND cd ${BOARD_NUTTX_SRC}/nuttx/tools &&
			./configure.sh ${BOARD}/nsh
		COMMAND echo Exporting NuttX for ${BOARD}
		COMMAND make -C ${BOARD_NUTTX_SRC}/nuttx -j${NUTTX_BUILD_THREADS}
			-r CONFIG_ARCH_BOARD=${BOARD} export
		COMMAND cp -r ${BOARD_NUTTX_SRC}/nuttx/nuttx-export.zip
			${BOARD}.export
		DEPENDS nuttx_copy_${BOARD})

	# extract
	add_custom_command(OUTPUT nuttx_export_${BOARD}.stamp
		COMMAND rm -rf ${BOARD_NUTTX_SRC}/nuttx-export
		COMMAND unzip ${BOARD}.export -d ${BOARD_NUTTX_SRC}
		COMMAND touch nuttx_export_${BOARD}.stamp
		DEPENDS ${BOARD}.export)
	add_custom_target(NuttX_${BOARD}
		DEPENDS nuttx_export_${BOARD}.stamp)

	# this symbolic linking is to allow cmake to build all of the
	# archives for the current makefile, it won't be needed in a
	# complete cmake build
	add_custom_command(OUTPUT ${CMAKE_SOURCE_DIR}/Archives/${BOARD}.export
		COMMAND mkdir -p ${CMAKE_SOURCE_DIR}/Archives
		COMMAND rm -f ${CMAKE_SOURCE_DIR}/Archives/${BOARD}.export
		COMMAND ln -sf ${CMAKE_BINARY_DIR}/${BOARD}.export
		${CMAKE_SOURCE_DIR}/Archives/${BOARD}.export
		DEPENDS ${BOARD}.export)
	add_custom_target(link_export_${BOARD}
		DEPENDS ${CMAKE_SOURCE_DIR}/Archives/${BOARD}.export)
endfunction()

# vim: set noet fenc=utf-8 ff=unix sts=4 sw=4 ts=4 :
