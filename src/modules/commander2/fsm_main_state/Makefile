all: test test-mc

CTEST_ARGS?="-V"

build:
	mkdir -p build && cd build && cmake .. && make

test: build
	cd build && ctest $(CTEST_ARGS)

build-mc:
	mkdir -p build-mc && \
		cd build-mc && \
		cmake .. -DCMAKE_TOOLCHAIN_FILE=../cmake/Toolchain-goto-cc.cmake && \
		make

test-mc: build-mc
	cd build-mc && ctest $(CTEST_ARGS)

clean:
	rm -rf build build-mc


.PHONY: test clean build build-mc test-mc
