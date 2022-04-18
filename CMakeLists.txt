cmake_minimum_required(VERSION 3.19)
project(icp)

set(CMAKE_CXX_STANDARD 14)

add_definitions(${PCL_DEFINITIONS})
find_package(PCL REQUIRED)

include_directories(include ${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})

add_library(icp src/icp.cpp)

add_executable(icp_lib_test src/main.cpp)

target_link_libraries(icp_lib_test icp ${PCL_LIBRARIES} glog)