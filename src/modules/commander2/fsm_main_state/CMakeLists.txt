cmake_minimum_required(VERSION 2.8)


project(FsmMainState CXX)

enable_testing()

include_directories(../../)

add_library(fsm
	FsmMainState.cpp
	MainState.cpp
	MainStateManual.cpp
	)

link_libraries(fsm)

add_executable(test_fsm_main_state
	test_fsm_main_state.cpp
	)
add_test(test_fsm_main_state
	${CBMC} ${CBMC_FLAGS} test_fsm_main_state)
