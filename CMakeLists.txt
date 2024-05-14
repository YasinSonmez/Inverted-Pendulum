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

add_executable(inverted_pendulum_simulation inverted_pendulum_simulation.cpp)
#add_executable(acc_controller acc_controller.cpp)

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

# Find OSQP library and headers
find_package(osqp REQUIRED)
if(osqp_FOUND)
    message(STATUS "Found osqp")
    message(STATUS "osqp include dir: ${OSQP_INCLUDE_DIR}")
    message(STATUS "osqp library dir: ${OSQP_LIBRARIES}")
else()
    message(FATAL_ERROR "Could not locate osqp")
endif()

# Include the external libraries to the project
# This is necessary to include the headers of the external libraries
set(EXTERN_INCLUDE_DIRS ${OSQP_INCLUDE_DIR}) 

include_directories(${EXTERN_INCLUDE_DIRS} "include")

configure_DynamoRIO_global(OFF ON)
# configure_DynamoRIO_static(inverted_pendulum_simulation)
# configure_DynamoRIO_static(acc_controller)

# use_DynamoRIO_static_client(inverted_pendulum_simulation drmemtrace_static)

# if (ZLIB_FOUND)
#   target_link_libraries(inverted_pendulum_simulation ${ZLIB_LIBRARIES})
# endif ()
# if (libsnappy)
#   target_link_libraries(inverted_pendulum_simulation snappy)
# endif ()
# if (liblz4)
#   target_link_libraries(inverted_pendulum_simulation lz4)
# endif ()

# use_DynamoRIO_static_client(inverted_pendulum_simulation drmgr_static)
# use_DynamoRIO_static_client(inverted_pendulum_simulation drreg_static)
# use_DynamoRIO_static_client(inverted_pendulum_simulation drbbdup_static)
# use_DynamoRIO_static_client(inverted_pendulum_simulation drutil_static)
# use_DynamoRIO_static_client(inverted_pendulum_simulation drx_static)
# use_DynamoRIO_static_client(inverted_pendulum_simulation drwrap_static)
# use_DynamoRIO_static_client(inverted_pendulum_simulation drsyms_static)
# use_DynamoRIO_static_client(inverted_pendulum_simulation drpttracer_static)
# use_DynamoRIO_static_client(inverted_pendulum_simulation drstatecmp_static)
# use_DynamoRIO_static_client(inverted_pendulum_simulation drcovlib_static)

# Define a function to link DynamoRIO static clients to a target
function(link_dynamoRIO_clients target)
  configure_DynamoRIO_static(${target})

  use_DynamoRIO_static_client(${target} drmemtrace_static)

  if (ZLIB_FOUND)
    target_link_libraries(${target} ${ZLIB_LIBRARIES})
  endif ()
  if (libsnappy)
    target_link_libraries(${target} snappy)
  endif ()
  if (liblz4)
    target_link_libraries(${target} lz4)
  endif ()
  use_DynamoRIO_static_client(${target} drmgr_static)
  use_DynamoRIO_static_client(${target} drreg_static)
  use_DynamoRIO_static_client(${target} drbbdup_static)
  use_DynamoRIO_static_client(${target} drutil_static)
  use_DynamoRIO_static_client(${target} drx_static)
  use_DynamoRIO_static_client(${target} drwrap_static)
  use_DynamoRIO_static_client(${target} drsyms_static)
  use_DynamoRIO_static_client(${target} drpttracer_static)
  use_DynamoRIO_static_client(${target} drstatecmp_static)
  use_DynamoRIO_static_client(${target} drcovlib_static)
endfunction()

# Link static clients to the original executable
link_dynamoRIO_clients(inverted_pendulum_simulation)
#link_dynamoRIO_clients(acc_controller)

# Add executable to open game environment
find_package(SFML COMPONENTS graphics window system REQUIRED)
target_link_libraries(inverted_pendulum_simulation source sfml-graphics sfml-window sfml-system nlopt)
#target_link_libraries(acc_controller source sfml-graphics sfml-window sfml-system nlopt)

# Link static clients to the acc version executable
add_executable(acc_controller acc_controller.cpp)
link_dynamoRIO_clients(acc_controller)
target_link_libraries(acc_controller source sfml-graphics sfml-window sfml-system nlopt osqp::osqp)