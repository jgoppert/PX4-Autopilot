
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
