cmake_minimum_required(VERSION 3.13)
include(pico_sdk_import.cmake)
project(hello-sx1276)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED True)
#set(CMAKE_VERBOSE_MAKEFILE True)
set(HOME $ENV{HOME})
pico_sdk_init()
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -O0")

# ----- main ------------------------------------------------------------------

add_executable(main
  src/main.cpp
  kc1fsz-tools-cpp/src/Common.cpp
  kc1fsz-tools-cpp/src/rp2040/PicoPollTimer.cpp
  kc1fsz-tools-cpp/src/rp2040/SX1276Driver.cpp
)

target_include_directories(main PRIVATE 
  kc1fsz-tools-cpp/include
  #$ENV{HOME}/pico/CMSISDSP/CMSIS-DSP/Include
  #$ENV{HOME}/pico/CMSISDSP/CMSIS_6/CMSIS/Core/Include
)

pico_enable_stdio_usb(main 1)
target_link_libraries(main pico_stdlib hardware_i2c hardware_spi)

