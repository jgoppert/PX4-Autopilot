
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
