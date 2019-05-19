project(spin)
cmake_minimum_required(VERSION 3.1)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_module)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  )


catkin_package()


add_executable(
  spin_node 
  spin_node.cpp)

target_link_libraries(spin_node
        PUBLIC
        ${catkin_LIBRARIES}
        )


include_directories(
  include ${catkin_INCLUDE_DIRS}
)