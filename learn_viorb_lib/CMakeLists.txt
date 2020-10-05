cmake_minimum_required(VERSION 2.8)
project(learn_viorb_lib)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -g  -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -g  -march=native")

# Check C++14 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX14)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
   add_definitions(-DCOMPILEDWITHC14)
   message(STATUS "Using flag -std=c++14.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++14 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
add_subdirectory(Thirdparty/DBoW3)
add_subdirectory(Thirdparty/g2o)
#set (CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH}
#			/usr/local/opencv-3.4.5/share/OpenCV
#			/usr/share/OpenCV)
#set(OpenCV_DIR /home/te6000-16/opencv-3.4.5/build)
#set( OpenCV_FOUND 1 )

find_package(OpenCV REQUIRED)
find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Cholmod REQUIRED)

find_package(catkin REQUIRED)
catkin_package()

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${PROJECT_SOURCE_DIR}/src
${PROJECT_SOURCE_DIR}/src/ROS
${PROJECT_SOURCE_DIR}/include/ROS
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
${CHOLMOD_INCLUDE_DIR}
${PROJECT_SOURCE_DIR}/Thirdparty/DBoW3/src/
)

set(SRCS
src/System.cc
src/Tracking.cc
src/LocalMapping.cc
src/LoopClosing.cc
src/ORBextractor.cc
src/ORBmatcher.cc
src/FrameDrawer.cc
src/Converter.cc
src/MapPoint.cc
src/KeyFrame.cc
src/Map.cc
src/MapDrawer.cc
src/Optimizer.cc
src/PnPsolver.cc
src/Frame.cc
src/KeyFrameDatabase.cc
src/Sim3Solver.cc
src/Initializer.cc
src/Viewer.cc
src/ROS/IPublisherThread.cc
src/ROS/IMapPublisher.cc
src/ROS/DefaultImpls.cc

include/Frame.h
include/KeyFrame.h
include/Tracking.h
include/LocalMapping.h


src/IMU/configparam.h
src/IMU/configparam.cpp

src/IMU/imudata.h
src/IMU/imudata.cpp
src/IMU/IMUPreintegrator.h
src/IMU/IMUPreintegrator.cpp
src/IMU/so3.cpp
src/IMU/so3.h
src/IMU/NavState.h
src/IMU/NavState.cpp

src/IMU/g2otypes.h
src/IMU/g2otypes.cpp
)

set(LIBS
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
cholmod
DBoW3
g2o
${CHOLMOD_LIBRARIES}
${BLAS_LIBRARIES}
${LAPACK_LIBRARIES}
)

add_library(${PROJECT_NAME} SHARED ${SRCS})
target_link_libraries(${PROJECT_NAME} ${LIBS})

if (NOT DEFINED USE_MAP_SAVE_LOAD)
    set(USE_MAP_SAVE_LOAD OFF)
endif()
if(USE_MAP_SAVE_LOAD)
    message(STATUS "Compile With map save/load function")
    find_library(BOOST_SERIALIZATION boost_serialization)
    if (NOT BOOST_SERIALIZATION)
	message(FATAL_ERROR "Can't find libboost_serialization")
    endif()
    target_link_libraries(${PROJECT_NAME} ${BOOST_SERIALIZATION})
    target_compile_definitions(${PROJECT_NAME} PUBLIC FUNC_MAP_SAVE_LOAD=1)
endif()

## Build examples
# Build tools
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/tools)
#add_executable(bin_vocabulary
#tools/bin_vocabulary.cc)
#target_link_libraries(bin_vocabulary ${PROJECT_NAME})


