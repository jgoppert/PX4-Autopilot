set(CMAKE_SYSTEM_NAME Model-checking)

find_program(CBMC cbmc)
find_program(GOTO_CC goto-cc)
find_program(GOTO_INST goto-instrument)

set(CBMC_FLAGS
	--bounds-check
	--div-by-zero-check
	--pointer-check
	#--memory-leak-check
	--signed-overflow-check
	--unsigned-overflow-check
	--float-overflow-check
	--nan-check
	)

set(CMAKE_C_COMPILER ${GOTO_CC})
set(CMAKE_CXX_COMPILER ${GOTO_CC})
