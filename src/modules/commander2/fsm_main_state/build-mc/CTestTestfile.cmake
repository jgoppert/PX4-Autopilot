# CMake generated Testfile for 
# Source directory: /home/jgoppert/git/px4/Firmware/src/modules/commander2/fsm_main_state
# Build directory: /home/jgoppert/git/px4/Firmware/src/modules/commander2/fsm_main_state/build-mc
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
ADD_TEST(test_fsm_main_state "/usr/local/bin/cbmc" "--bounds-check" "--div-by-zero-check" "--pointer-check" "--signed-overflow-check" "--unsigned-overflow-check" "--float-overflow-check" "--nan-check" "test_fsm_main_state")
