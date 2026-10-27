# Builds the Jolt driver against a fetched Jolt release (path in JOLT_ROOT) in its documented default configuration.
cmake_minimum_required(VERSION 3.16)
project(jolt_driver CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Only the library is needed; every optional target is off. Cross-platform determinism, double precision and the other
# library options keep their defaults (off).
set(TARGET_UNIT_TESTS OFF CACHE BOOL "" FORCE)
set(TARGET_HELLO_WORLD OFF CACHE BOOL "" FORCE)
set(TARGET_PERFORMANCE_TEST OFF CACHE BOOL "" FORCE)
set(TARGET_SAMPLES OFF CACHE BOOL "" FORCE)
set(TARGET_VIEWER OFF CACHE BOOL "" FORCE)
add_subdirectory(${JOLT_ROOT}/Build ${CMAKE_BINARY_DIR}/jolt)

add_executable(jolt-driver driver.cpp)
target_link_libraries(jolt-driver PRIVATE Jolt)
# Jolt is built without RTTI by default; the code using it must match.
if (NOT MSVC)
  target_compile_options(jolt-driver PRIVATE -fno-rtti)
endif()
