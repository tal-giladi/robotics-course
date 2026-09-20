# FC.04 - a complete, real CMake project in 40 lines.
#
#   cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo
#   cmake --build build -j
#   ctest --test-dir build --output-on-failure
#   ./build/drive_demo
#
# Read it as: describe TARGETS and how they depend on each other. Never list flags globally
# when you can attach them to the target that needs them.

cmake_minimum_required(VERSION 3.16)          # Ubuntu 24.04 ships CMake 3.28; 3.16 is a safe floor
project(wheelodom VERSION 0.1.0 LANGUAGES CXX)

# Default to a build type, otherwise a bare `cmake -S . -B build` produces an unoptimised,
# unsymbolised build and people then benchmark it.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "" FORCE)
endif()

# ---- the library -------------------------------------------------------------------------------
add_library(wheelodom src/odometry.cpp)
add_library(wheelodom::wheelodom ALIAS wheelodom)     # so users write the same name in- and out-of-tree

# PUBLIC = "anyone who links me also needs this include path", because it appears in my header.
# PRIVATE would mean "only my own .cpp files need it".
target_include_directories(wheelodom PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(wheelodom PUBLIC cxx_std_17)  # not `set(CMAKE_CXX_FLAGS -std=c++17)`

target_compile_options(wheelodom PRIVATE
  $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall -Wextra -Wpedantic -Wconversion>)

# libm: on Linux std::sin lives in a separate library. Not needed on MSVC, hence the generator
# expression rather than a bare `target_link_libraries(wheelodom m)`.
target_link_libraries(wheelodom PUBLIC $<$<PLATFORM_ID:Linux>:m>)

# ---- the executable ----------------------------------------------------------------------------
add_executable(drive_demo src/drive_demo.cpp)
target_link_libraries(drive_demo PRIVATE wheelodom)   # include path and C++17 arrive with it

# ---- tests -------------------------------------------------------------------------------------
include(CTest)                                        # defines BUILD_TESTING, default ON
if(BUILD_TESTING)
  add_executable(test_odometry test/test_odometry.cpp)
  target_link_libraries(test_odometry PRIVATE wheelodom)
  add_test(NAME odometry COMMAND test_odometry)
endif()

# ---- install -----------------------------------------------------------------------------------
install(TARGETS wheelodom drive_demo
        ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
