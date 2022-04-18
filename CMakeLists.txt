cmake_minimum_required(VERSION 3.19)
project(icp)

set(CMAKE_CXX_STANDARD 14)

add_definitions(${PCL_DEFINITIONS})
find_package(PCL REQUIRED)
find_package(Ceres REQUIRED)

include_directories(include)
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${CERES_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
link_directories(${CERES_LIBRARY_DIRS})

add_library(icp src/icp.cpp)

add_executable(icp_lib_test src/main.cpp)

target_link_libraries(icp_lib_test
    icp
    ${PCL_LIBRARIES}
    ${CERES_LIBRARIES}
    glog)