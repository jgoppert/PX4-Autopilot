d=$(PWD)

px4fmu-v2_default:
	mkdir -p $d/build_arm && cd $d/build_arm && cmake -DCMAKE_TOOLCHAIN_FILE=../cmake/Toolchain-arm-none-eabi.cmake .. && make && ctest -V && cpack -G ZIP 

posix-sitl:
	mkdir -p $d/build_host && cd $d/build_host && cmake .. -DTARGET_OS=posix -DTARGET_BOARD=sitl -DTARGET_LABEL=simple && make && ctest -V && cpack -G ZIP

clean:
	rm -rf build_*/

.PHONY: clean
