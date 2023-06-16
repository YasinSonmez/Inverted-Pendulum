cmake_minimum_required (VERSION 3.5)

# Set project name
project(inverted_pendulum)

# set the C++ standard to C++ 20
set(CMAKE_CXX_STANDARD 20)
# set the C++ compiler to use O3
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

# Add library
add_subdirectory(src)

# MPC package
find_package(mpc++ CONFIG REQUIRED)
include_directories(${mpc++_INCLUDE_DIRS})

# Add executable to plot states
find_package(sciplot)
if(sciplot_FOUND)
  add_executable(plot src/main.cpp)
  target_link_libraries(plot sciplot::sciplot source)
else()
  message("sciplot need to be installed to generate the plot executable")
endif()

# Add executable to run unit tests
find_package(GTest)
if(GTest_FOUND)
  add_executable(test tests/main.cpp tests/inverted_pendulum_test.cpp)
  target_link_libraries(test source ${GTEST_LIBRARIES} pthread)
else()
  message("gloogletest need to be installed to generate the test executable")
endif()

# Additional resources like images and fonts
configure_file(res/Roboto-Regular.ttf Roboto-Regular.ttf COPYONLY)

# Add executable to open game environment
find_package(SFML COMPONENTS graphics window system REQUIRED)
add_executable(cartpole game/main.cpp)
add_executable(cartpole_visual game/main_visual.cpp)
target_link_libraries(cartpole source sfml-graphics sfml-window sfml-system mpc++)
target_link_libraries(cartpole_visual source sfml-graphics sfml-window sfml-system mpc++)

# Add executable to run examples
add_executable(example examples/lqr_design.cpp)
target_link_libraries(example source)