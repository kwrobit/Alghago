cmake_minimum_required(VERSION 2.8.3)
project(alghago_algorithm)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  alghago_msgs
  suhan_math
)

catkin_package(
)

file(GLOB_RECURSE HeaderFiles "include/*.h")
add_custom_target(alghago_algorithm_headers SOURCES ${HeaderFiles})


include_directories(
  ${catkin_INCLUDE_DIRS}
  include/alghago_algorithm/
)


add_library(alghago_algorithm
  src/alghago_algorithm.cpp
)


add_executable(alghago_algorithm_node src/alghago_algorithm_node.cpp)

#add_dependencies(alghago_algorithm_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(alghago_algorithm_node
  alghago_algorithm
  ${catkin_LIBRARIES}
)


set(CMAKE_BUILD_TYPE Debug)
