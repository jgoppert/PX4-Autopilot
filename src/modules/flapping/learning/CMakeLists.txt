cmake_minimum_required(VERSION 2.8)

project(px4_ga)

set(CMAKE_BUILD_TYPE "DEBUG")

# packaging
set(CPACK_PACKAGE_CONTACT  "james.goppert@gmail.com")
set(CPACK_SOURCE_ZIP ON)
set(CPACK_SOURCE_IGNORE_FILES "build/")
include(CPack)
include(CTest)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -Wall -Wno-long-long -pedantic")

add_executable(learn
	GeneticAlgorithm.cpp
	learn.cpp)
add_test(learn learn)
