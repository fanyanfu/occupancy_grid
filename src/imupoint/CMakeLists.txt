cmake_minimum_required(VERSION 2.8.3)
project(imupoint)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sensor_msgs
  roscpp
  rospy
  rosbag
  std_msgs
  image_transport
  move_base_msgs
  cv_bridge
  image_transport
  message_generation
)
add_message_files(
  FILES
  test.msg
  # Message2.msg
)

#find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)

include_directories(
  include
	${catkin_INCLUDE_DIRS} 
	${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS})

  generate_messages(
 DEPENDENCIES
 geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs message_runtime
  DEPENDS EIGEN3 PCL 
  INCLUDE_DIRS include
)
add_executable(imupoint src/rosbagregist.cpp)
target_link_libraries(imupoint ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(imupointsimple src/rosbagsimple.cpp)
target_link_libraries(imupointsimple ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(pcd2topic src/pcd2topic.cpp)
target_link_libraries(pcd2topic ${catkin_LIBRARIES} ${PCL_LIBRARIES})





