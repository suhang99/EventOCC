cmake_minimum_required(VERSION 3.10)

project(evlc_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
  add_compile_options(-O3)
endif()

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

add_message_files(
    FILES
    Event.msg
    EventArray.msg
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)