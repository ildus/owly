mkdir build && cd build
cmake -G "Unix Makefiles" -D \
	"CMAKE_TOOLCHAIN_FILE=../cmake/GNU-ARM-Toolchain.cmake" ../
