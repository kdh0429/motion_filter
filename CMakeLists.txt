cmake_minimum_required(VERSION 3.0.2)
project(motion_filter)

# specify the C++ standard
# set(CMAKE_CXX_STANDARD 11)
# set(CMAKE_CXX_STANDARD_REQUIRED True)
add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  VR
  # geometry_msgs
  # message_generation
  roscpp
  roslib
  # sensor_msgs
  std_msgs
  # tf
  #mujoco_ros
  # eigen_conversions
)
find_package(Eigen3 REQUIRED)
include_directories(${Eigen3_INCLUDE_DIRS})
# Find the manif library
find_package(manif REQUIRED)
find_package(toml11 REQUIRED)
include_directories(${toml11_INCLUDE_DIRS})

catkin_package()


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

########
# LIBS #
########
file(GLOB SRC
  "src/data_handler.cpp"
  "src/pre_process.cpp"
)

add_library(motion_filter_lib SHARED
  ${SRC}
)
target_link_libraries(motion_filter_lib
  ${catkin_LIBRARIES}
  ${Eigen3_LIBRARIES}
  ${toml11_LIBRARIES}
  )

target_include_directories(motion_filter_lib SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
target_link_libraries(motion_filter_lib  ${catkin_LIBRARIES})
add_dependencies(motion_filter_lib ${catkin_EXPORTED_TARGETS})

########
# MAIN #
########

add_executable(main src/main.cpp)
target_link_libraries(main 
  motion_filter_lib
  ${catkin_LIBRARIES}
)
add_dependencies(main ${catkin_EXPORTED_TARGETS})


########
# TEST #
########

add_executable(mainf_test test/manif_test.cpp)
target_include_directories(mainf_test SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIRS})
target_include_directories(mainf_test SYSTEM PUBLIC ${manif_INCLUDE_DIRS})
