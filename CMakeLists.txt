cmake_minimum_required(VERSION 3.16)

project(main)

link_directories($ENV{DYNAMORIO_HOME}/tools/lib64/release)
link_directories($ENV{DYNAMORIO_HOME}/ext/lib64/release)

link_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/include)

include_directories(/usr/local/include)
link_directories(/usr/local/lib)

# Find the NLopt library (optional, only if find_package doesn't work as expected)
find_package(NLopt REQUIRED)

if(COMMAND cmake_policy)
	cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

# set the C++ standard to C++ 20
set(CMAKE_CXX_STANDARD 20)
# set the C++ compiler to use O3
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

# Add library
add_subdirectory(src)

add_executable(main main.cpp)
add_executable(simulate_and_record simulate_and_record.cpp)

# MPC package
# Set the MPC++ package path
# include_directories(home/yasin/libmpc-0.4.0/include)


find_package(DynamoRIO)
if (NOT DynamoRIO_FOUND)
  message(FATAL_ERROR "DynamoRIO package required to build")
endif(NOT DynamoRIO_FOUND)

find_package(ZLIB)
if (ZLIB_FOUND)
  message(STATUS "Found zlib")
endif ()

find_library(libsnappy snappy)
if (libsnappy)
  message(STATUS "Found snappy: ${libsnappy}")
endif ()

find_library(liblz4 lz4)
if (liblz4)
  message(STATUS "Found liblz4: ${liblz4}")
endif ()

configure_DynamoRIO_static(main)

use_DynamoRIO_static_client(main drmemtrace_static)

if (ZLIB_FOUND)
  target_link_libraries(main ${ZLIB_LIBRARIES})
endif ()
if (libsnappy)
  target_link_libraries(main snappy)
endif ()
if (liblz4)
  target_link_libraries(main lz4)
endif ()

use_DynamoRIO_static_client(main drmgr_static)
use_DynamoRIO_static_client(main drreg_static)
use_DynamoRIO_static_client(main drbbdup_static)
use_DynamoRIO_static_client(main drutil_static)
use_DynamoRIO_static_client(main drx_static)
use_DynamoRIO_static_client(main drwrap_static)
use_DynamoRIO_static_client(main drsyms_static)
use_DynamoRIO_static_client(main drpttracer_static)
use_DynamoRIO_static_client(main drstatecmp_static)
use_DynamoRIO_static_client(main drcovlib_static)

# Add executable to open game environment
find_package(SFML COMPONENTS graphics window system REQUIRED)
# target_link_libraries(main source sfml-graphics sfml-window sfml-system mpc++)
target_link_libraries(main source sfml-graphics sfml-window sfml-system nlopt)
# target_link_libraries(simulate_and_record source sfml-graphics sfml-window sfml-system mpc++)
target_link_libraries(simulate_and_record source sfml-graphics sfml-window sfml-system nlopt)
# target_link_libraries(main source mpc++)
# target_link_libraries(simulate_and_record source mpc++)

# Additional resources like images and fonts
configure_file(res/Roboto-Regular.ttf Roboto-Regular.ttf COPYONLY)

#set_target_properties(main PROPERTIES CXX_STANDARD 20 CXX_EXTENSIONS OFF)
#set_target_properties(simulate_and_record PROPERTIES CXX_STANDARD 20 CXX_EXTENSIONS OFF)
