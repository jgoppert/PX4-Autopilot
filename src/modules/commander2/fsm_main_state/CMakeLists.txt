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

add_executable(test_c
	src/test_c.c
	)
add_test(test_c
	${CBMC} ${CBMC_FLAGS} test_c)

