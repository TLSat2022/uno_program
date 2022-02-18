# !!! WARNING !!! AUTO-GENERATED FILE, PLEASE DO NOT MODIFY IT AND USE
# https://docs.platformio.org/page/projectconf/section_env_build.html#build-flags
#
# If you need to override existing CMake configuration or add extra,
# please create `CMakeListsUser.txt` in the root of project.
# The `CMakeListsUser.txt` will not be overwritten by PlatformIO.

cmake_minimum_required(VERSION 3.13)
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_C_COMPILER_WORKS 1)
set(CMAKE_CXX_COMPILER_WORKS 1)

project("dummy_receiver" C CXX)

include(CMakeListsPrivate.txt)
include_directories(".pio/libdeps/uno/Adafruit Unified Sensor")
include_directories(".pio/libdeps/uno/DHT sensor library")
include_directories(".pio/libdeps/uno/LIS3MDL")
include_directories(".pio/libdeps/uno/LPS")
include_directories(".pio/libdeps/uno/LSM6")
include_directories(".pio/libdeps/uno/Adafruit Unified Sensor")
include_directories(".pio/libdeps/uno/Adafruit BMP280 Library")
include_directories(".pio/libdeps/uno/Adafruit BusIO")
include_directories(".pio/libdeps/uno/PID")
include_directories(".pio/libdeps/uno/SD/src")

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/CMakeListsUser.txt)
include(CMakeListsUser.txt)
endif()

add_custom_target(
    Production ALL
    COMMAND platformio -c clion run "$<$<NOT:$<CONFIG:All>>:-e${CMAKE_BUILD_TYPE}>"
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

add_custom_target(
    Debug ALL
    COMMAND platformio -c clion debug "$<$<NOT:$<CONFIG:All>>:-e${CMAKE_BUILD_TYPE}>"
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

add_executable(Z_DUMMY_TARGET ${SRC_LIST})
