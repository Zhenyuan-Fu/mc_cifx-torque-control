cmake_minimum_required(VERSION 3.5)

project(mc_cifx VERSION 1.0.0 LANGUAGES C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 11)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

find_package(mc_rtc REQUIRED)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/sanitizers/cmake" ${CMAKE_MODULE_PATH})
find_package(Sanitizers)

set(mc_cifx_SOURCES
  src/main.cpp
  src/Display.c
  src/IOAPPzm.c
  src/SDOApp.c
  src/CalculateProg.c
  src/InitCommunication.c
  src/utils.cpp
)

add_executable(mc_cifx ${mc_cifx_SOURCES})
add_sanitizers(mc_cifx)
target_include_directories(mc_cifx PUBLIC /usr/local/include/cifx)
target_link_libraries(mc_cifx PUBLIC cifx Threads::Threads mc_rtc::mc_control)
if(UNIX)
  target_compile_options(mc_cifx PUBLIC -pedantic -Wno-long-long -Wall -Wextra -Wcast-align -Wcast-qual -Wformat -Wwrite-strings -Wconversion)
endif()

install(TARGETS mc_cifx RUNTIME DESTINATION bin)
