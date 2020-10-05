cmake_minimum_required(VERSION 2.8)
project(learn_viorb_ros)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")

ADD_DEFINITIONS("-std=c++14")
SET( Eigen_DIR /usr/include/eigen3 )
#set (CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH}
#			/usr/local/opencv-3.4.5/share/OpenCV
#			/usr/share/OpenCV)
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../learn_viorb_lib/cmake_modules)

#set(OpenCV_DIR /home/te6000-16/opencv-3.4.5/build)
#set( OpenCV_FOUND 1 )

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Boost COMPONENTS system)

find_package(catkin REQUIRED COMPONENTS
  learn_viorb_lib
  roscpp
  tf
  std_msgs
  sensor_msgs
  image_transport
  image_geometry
  cv_bridge
  message_generation
  nav_msgs
  rosbag
)


catkin_package(CATKIN_DEPENDS message_runtime std_msgs)
#catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES learn_viorb_lib)

include_directories(
  ${roscpp_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/../learn_viorb_lib
  ${PROJECT_SOURCE_DIR}/../learn_viorb_lib/include
  ${PROJECT_SOURCE_DIR}/../learn_viorb_lib/include/ROS
  ${EIGEN3_INCLUDE_DIR}
  ${Pangolin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/../learn_viorb_lib/Thirdparty/DBoW3/
)

set(LIBS
  ${roscpp_LIBRARIES}
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
  ${Pangolin_LIBRARIES} 
  ${catkin_LIBRARIES}
  ${PCL_LIBRARY_DIRS}
  ${PROJECT_SOURCE_DIR}/../../devel_isolated/learn_viorb_lib/lib/liblearn_viorb_lib.so
  ${Boost_LIBRARIES}
)

if(USE_MAP_SAVE_LOAD)
ADD_DEFINITIONS(-DFUNC_MAP_SAVE_LOAD)
endif()

# Node for monocular camera
add_executable(VIO
src/ros_vio.cc
src/MsgSync/MsgSynchronizer.cpp
src/MsgSync/MsgSynchronizer.h
src/ROSPublisher.cpp
)


target_link_libraries(VIO
${LIBS}
)

# Node for monocular camera edited
add_executable(VIO_Stereo
        src/ros_vio_stereo.cc
        src/MsgSync/MsgSynchronizer.cpp
        src/MsgSync/MsgSynchronizer.h
        src/ROSPublisher.cpp
)

target_link_libraries(VIO_Stereo
${LIBS}
)

add_executable(VIO_RGBD
        src/ros_vio_rgbd.cc
        src/MsgSync/MsgSynchronizer.cpp
        src/MsgSync/MsgSynchronizer.h
        src/ROSPublisher.cpp
)

target_link_libraries(VIO_RGBD
${LIBS}
)

