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
add_git_submodule(genmsg Tools/genmsg)
add_git_submodule(gencpp Tools/gencpp)
add_git_submodule(gtest unittests/gtest)
add_git_submodule(eigen src/lib/eigen) 

