cmake_minimum_required(VERSION 3.0.2)
project(motion_filter)

# specify the C++ standard
# set(CMAKE_CXX_STANDARD 11)
# set(CMAKE_CXX_STANDARD_REQUIRED True)
add_compile_options(-std=c++11)

if (WIN32)
message(STATUS "WINDOW")
  set(toml11_INCLUDE_DIRS "C:/Program Files (x86)/toml11/include")
  include_directories("C:/Program Files (x86)/RBDL/include" "C:/Program Files (x86)/qpOASES/include")
  link_directories("C:/Program Files (x86)/RBDL/lib" "C:/Program Files (x86)/qpOASES/lib")
endif (WIN32)
find_package(Boost REQUIRED COMPONENTS filesystem)

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
if((NOT TARGET Eigen3::Eigen) AND (DEFINED EIGEN3_INCLUDE_DIR))
	add_library(AliasEigen3 INTERFACE)
	target_include_directories(AliasEigen3 INTERFACE ${EIGEN3_INCLUDE_DIR})
	add_library(Eigen3::Eigen ALIAS AliasEigen3)
endif()
# Find the manif library
find_package(manif REQUIRED)
find_package(toml11 REQUIRED)
# find_package(rbdl REQUIRED)
# find_package(qpOASES REQUIRED)

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
add_library(bvh11 STATIC src/bvh11.cpp)
target_link_libraries(bvh11 Eigen3::Eigen)
target_include_directories(bvh11 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_library(qp_lib STATIC src/quadraticprogram.cpp)
target_link_libraries(qp_lib Eigen3::Eigen qpOASES)
target_include_directories(qp_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_library(logger_lib STATIC
"src/logger.cpp"
)
target_include_directories(logger_lib SYSTEM PUBLIC 
${Boost_INCLUDE_DIRS}
)
target_link_libraries(logger_lib
${Boost_LIBRARIES}
)

file(GLOB MFSRC
"src/data_handler.cpp"
"src/se3_filter.cpp"
"src/skeleton_kinematics_base.cpp"
"src/skeleton_kinematics_huber.cpp"
"src/skeleton_kinematics_kalman.cpp"
"src/skeleton_kinematics_lsq.cpp"
"src/kalman_filter.cpp"

)
add_library(motion_filter_lib STATIC
  ${MFSRC}
)
target_link_libraries(motion_filter_lib
  qp_lib
  rbdl
  ${catkin_LIBRARIES}
  ${Eigen3_LIBRARIES}
  ${toml11_LIBRARIES}
  ${manif_LIBRARIES}
  logger_lib
  bvh11
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

add_executable(se3_filter_main main/se3_filter_main.cpp)
target_link_libraries(se3_filter_main
  motion_filter_lib
  ${catkin_LIBRARIES}
)
add_dependencies(se3_filter_main ${catkin_EXPORTED_TARGETS})

add_executable(skel_kinect_main main/skel_kinect_main.cpp)
target_link_libraries(skel_kinect_main 
rbdl
Eigen3::Eigen
motion_filter_lib
${catkin_LIBRARIES}
)

########
# TEST #
########

add_executable(mainf_test test/manif_test.cpp)
target_include_directories(mainf_test SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIRS})
target_include_directories(mainf_test SYSTEM PUBLIC ${manif_INCLUDE_DIRS})

add_executable(calibration test/calibration.cpp)
target_link_libraries(calibration
 logger_lib 
 ${catkin_LIBRARIES}
 motion_filter_lib
 )
target_include_directories(calibration SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIRS})

add_executable(logger_test test/logger_test.cpp)
target_link_libraries(logger_test logger_lib ${catkin_LIBRARIES})
target_include_directories(logger_test SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIRS})

add_executable(bvh_read test/bvh_read.cpp)
target_link_libraries(bvh_read bvh11)

add_executable(data_handler_test test/data_handler_test.cpp)
target_link_libraries(data_handler_test 
motion_filter_lib
${catkin_LIBRARIES}
)

add_executable(manifold_kinematics_test test/manifold_kinematics_test.cpp)
target_link_libraries(manifold_kinematics_test 
rbdl
Eigen3::Eigen
qp_lib
)
target_include_directories(manifold_kinematics_test SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIRS})
target_include_directories(manifold_kinematics_test SYSTEM PUBLIC ${manif_INCLUDE_DIRS})

add_executable(pointer_test test/pointer_test.cpp)
target_link_libraries(pointer_test 
rbdl
Eigen3::Eigen
# qp_lib
)
# target_include_directories(manifold_kinematics_test SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIRS})
# target_include_directories(manifold_kinematics_test SYSTEM PUBLIC ${manif_INCLUDE_DIRS})