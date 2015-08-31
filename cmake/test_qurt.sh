#!/bin/bash
d=$PWD && \
mkdir -p $d/build_qurt && cd $d/build_qurt && cmake .. -DCMAKE_TOOLCHAIN_FILE=../cmake/qurt-clang-hexagon.cmake -DTARGET_OS=qurt -DTARGET_BOARD=hil -DTARGET_LABEL=simple && make VERBOSE=1 && ctest -V && cpack -G ZIP
