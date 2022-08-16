cmake_minimum_required(VERSION 3.0.2)
project(pcl_test)


######################
### Cmake flags
######################
# set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")


######################
### Packages
######################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  # msg
  sensor_msgs
  std_msgs
  # cv
  cv_bridge
  # pcl
  pcl_conversions
  #pcl_ros
  # msg generation
  message_generation
)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
# find_package(Ceres REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem program_options system)

######################
### Message generation
######################


######################
### Catkin
######################
catkin_package(
  INCLUDE_DIRS include
  # CATKIN_DEPENDS pcl_conversions pcl_ros roscpp rospy sensor_msgs std_msgs
  DEPENDS  PCL
)

include_directories(
  include
  ${CERES_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

#link_directories(
#    ${PCL_LIBRARY_DIRS}
#    ${OpenCV_LIBRARY_DIRS}
#)


######################
### Executable
######################
add_executable(find_line_1 src/find_line_1.cpp)
target_link_libraries(find_line_1 ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(process_sim_xiangliang_ra src/process_sim_xiangliang_ra.cpp)
target_link_libraries(process_sim_xiangliang_ra ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(mesh_sampling src/mesh_sampling.cpp)
target_link_libraries(mesh_sampling ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(pcd2txt src/pcd2txt.cpp)
target_link_libraries(pcd2txt ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(locate_weld src/locate_weld.cpp)
target_link_libraries(locate_weld ${catkin_LIBRARIES})

add_executable(density_calculate src/density_calculate.cpp)
target_link_libraries(density_calculate ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(read_and_show src/read_and_show.cpp)
target_link_libraries(read_and_show ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(box_girder_construction src/box_girder_construction.cpp)
target_link_libraries(box_girder_construction ${catkin_LIBRARIES} ${PCL_LIBRARIES})

#add_executable(ceres_test src/ceres_test.cpp)
#target_link_libraries(ceres_test ${catkin_LIBRARIES} ${CERES_LIBRARIES})

