cmake_minimum_required(VERSION 2.8)


project(FsmMainState C CXX)

enable_testing()

include_directories(../../)


if (CBMC)
	set(CBMC_FLAGS ${CBMC_FLAGS}
		#--unwind 3
		)
else()
	add_library(cbmc_stub
		src/cbmc_stub.c
		)
	link_libraries(cbmc_stub)
endif()

add_library(fsm
	src/fsm.c
	)

link_libraries(fsm)

add_executable(test_fsm
	src/test_fsm.c
	)
add_test(test_fsm
	${CBMC} ${CBMC_FLAGS} test_fsm)

