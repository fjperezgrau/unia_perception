cmake_minimum_required(VERSION 2.8.3)
project(unia_perception)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  genmsg
  image_transport
  cv_bridge
  pcl_ros
  pcl_conversions
)

find_package(OpenCV)
find_package(PCL)

catkin_package()

include_directories(
	${catkin_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(img_proc_base src/img_proc_base.cpp)
target_link_libraries(img_proc_base ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_executable(img_proc_full src/img_proc_full.cpp)
target_link_libraries(img_proc_full ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(pc_proc_base src/pc_proc_base.cpp)
target_link_libraries(pc_proc_base ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_executable(pc_proc_full src/pc_proc_full.cpp)
target_link_libraries(pc_proc_full ${catkin_LIBRARIES} ${PCL_LIBRARIES})
