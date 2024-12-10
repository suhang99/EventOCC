cmake_minimum_required(VERSION 3.10)

project(evlc_screen VERSION 0.1)

option(STATIC_SETUP "Static setup" OFF)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED True)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
  add_compile_options(-O3)
endif()

if(STATIC_SETUP)
  add_definitions(-DSTATIC_SETUP)
endif()

set(CATKIN_DEPS 
  roscpp 
  evlc_msgs 
  evlc_common 
  rosbag 
  std_msgs 
  sensor_msgs 
  cv_bridge 
  image_transport
)

find_package(catkin REQUIRED COMPONENTS ${CATKIN_DEPS})
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS ${CATKIN_DEPS}
)

set(SOURCES 
  src/detector.cpp
  src/decoder.cpp
  src/edge_tracker.cpp
  src/marker.cpp
  src/optimizer.cpp
  src/preprocessor.cpp
  src/system.cpp
  src/tracker.cpp
  src/visualizer.cpp
)

add_library(${PROJECT_NAME} ${SOURCES})
target_include_directories(
  ${PROJECT_NAME} 
  PUBLIC 
  include 
  ${OpenCV_INCLUDE_DIRS} 
  ${catkin_INCLUDE_DIRS} 
)
target_link_libraries(
  ${PROJECT_NAME} 
  Eigen3::Eigen 
  ${CERES_LIBRARIES} 
  ${OpenCV_LIBS} 
  ${catkin_LIBRARIES} 
)

add_executable(${PROJECT_NAME}_node)
target_sources(${PROJECT_NAME}_node PRIVATE src/node.cpp)
target_include_directories(${PROJECT_NAME}_node PRIVATE include)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

install(
  TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
