cmake_minimum_required(VERSION 3.16)
project(test)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
add_executable(client client.cpp)
target_link_libraries(client ${OpenCV_LIBS})


