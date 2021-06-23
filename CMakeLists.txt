cmake_minimum_required(VERSION 3.0.2)
project(motion_filter)

# specify the C++ standard
# set(CMAKE_CXX_STANDARD 11)
# set(CMAKE_CXX_STANDARD_REQUIRED True)
add_compile_options(-std=c++11)

if (WIN32)
message(STATUS "WINDOW")
  set(toml11_INCLUDE_DIRS "C:/Program Files (x86)/toml11/include")
endif (WIN32)
find_package(catkin REQUIRED COMPONENTS
# geometry_msgs
# message_generation
roscpp
roslib
# sensor_msgs
std_msgs
VR
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
message(STATUS "TOML DIR ${toml11_INCLUDE_DIRS}")
message(STATUS "MANIF DIR ${manif_FOUND} ${manif_INCLUDE_DIRS}")

catkin_package(
  CATKIN_DEPENDS VR
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

########
# LIBS #
########
file(GLOB MFSRC
"src/data_handler.cpp"
"src/pre_process.cpp"
)
add_library(logger_lib SHARED
  "src/logger.cpp"
)

add_library(motion_filter_lib SHARED
  ${MFSRC}
)
target_link_libraries(motion_filter_lib
  ${catkin_LIBRARIES}
  ${Eigen3_LIBRARIES}
  ${toml11_LIBRARIES}
  ${manif_LIBRARIES}
  logger_lib
  )

target_include_directories(motion_filter_lib SYSTEM PUBLIC 
${EIGEN3_INCLUDE_DIRS}
${catkin_INCLUDE_DIRS}
${manif_INCLUDE_DIRS}
${toml11_INCLUDE_DIRS}
)
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

add_executable(logger_test test/logger_test.cpp)
target_link_libraries(logger_test logger_lib ${catkin_LIBRARIES})
target_include_directories(logger_test SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIRS})
