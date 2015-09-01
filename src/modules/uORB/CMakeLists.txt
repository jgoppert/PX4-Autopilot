set(module uorb)

set(SRCS
	objects_common.cpp
	uORBUtils.cpp
	uORB.cpp
	uORBMain.cpp
	)

if (${OS} STREQUAL "nuttx")
	list(APPEND SRCS
		uORBDevices_nuttx.cpp
		uORBTest_UnitTest.cpp
		uORBManager_nuttx.cpp
		)

elseif (${OS} STREQUAL "posix")
	list(APPEND SRCS
		uORBTest_UnitTest.cpp
		uORBDevices_posix.cpp
		uORBManager_posix.cpp
		)

elseif (${OS} STREQUAL "posix-arm")
	list(APPEND SRCS
		uORBTest_UnitTest.cpp
		uORBDevices_posix.cpp
		uORBManager_posix.cpp
		)
elseif (${OS} STREQUAL "qurt")
	list(APPEND SRCS
		uORBDevices_posix.cpp
		uORBManager_posix.cpp
		)
endif()

include_directories(${CMAKE_CURRENT_BINARY_DIR})
add_library(${module} STATIC EXCLUDE_FROM_ALL ${SRCS})
set_target_properties(${module} PROPERTIES COMPILE_FLAGS "-Os")
set(stack 2048)
set(main uorb_main)

# vim: set noet fenc=utf-8 ft=cmake ff=unix sts=4 sw=4 ts=4 :
