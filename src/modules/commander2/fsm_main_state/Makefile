all: test test-mc

build:
	mkdir -p build && cd build && cmake .. && make

test: build
	cd build && ctest -V

build-mc:
	mkdir -p build-mc && \
		cd build-mc && \
		cmake .. -DCMAKE_TOOLCHAIN_FILE=../cmake/Toolchain-goto-cc.cmake && \
		make

test-mc: build-mc
	cd build-mc && ctest -V

clean:
	rm -rf build build-mc


.PHONY: test clean build build-mc test-mc
