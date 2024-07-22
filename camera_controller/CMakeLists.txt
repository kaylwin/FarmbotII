cmake_minimum_required(VERSION 3.10)

# Set the project name
project(ColorDetectionWithVL53L5CX)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_COMPILER g++-9)



# Find the required packages
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBSERIALPORT REQUIRED libserialport)

# Include directories for VL53L5CX API
include_directories(${CMAKE_SOURCE_DIR}/uld-driver/inc
	${CMAKE_SOURCE_DIR}/uld-driver/examples)


file(GLOB ULD_DRIVER_SRCS
    "${CMAKE_SOURCE_DIR}/uld-driver/src/*.c"
    "${CMAKE_SOURCE_DIR}/uld-driver/platform/*.c"

) 

# Source files for the main project
set(SRCS src/color_detection.cpp ${ULD_DRIVER_SRCS})

# Add the executable
add_executable(color_detection ${SRCS})

# Link the libraries
target_link_libraries(color_detection
    ${OpenCV_LIBS}
    ${LIBSERIALPORT_LIBRARIES}
    ${CMAKE_CURRENT_LIST_DIR}/uld-driver/libuld_driver.a
)

# Link directories for libserialport
#link_directories(${LIBSERIALPORT_LIBRARY_DIRS})

