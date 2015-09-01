d=$(PWD)

px4fmu-v2_simple:
	mkdir -p $d/build_$@ && cd $d/build_$@ && \
		cmake .. -DCMAKE_TOOLCHAIN_FILE=../cmake/Toolchain-arm-none-eabi.cmake \
		-DOS=nuttx -DBOARD=px4fmu-v2 -DLABEL=simple && \
		make && ctest -V && cpack -G ZIP 

posix-sitl_simple:
	mkdir -p $d/build_$@ && cd $d/build_$@ && \
		cmake .. \
		-DOS=posix -DBOARD=sitl -DLABEL=simple && \
		make && ctest -V && cpack -G ZIP

clean:
	rm -rf build_*/

.PHONY: clean
