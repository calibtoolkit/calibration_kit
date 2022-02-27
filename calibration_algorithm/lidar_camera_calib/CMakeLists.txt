cmake_minimum_required(VERSION 3.10.2)
project(lidar_camera_calib)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE release)
ENDIF()

add_compile_options(-std=c++14)

# Eigen, actually it's no need to find it'
find_package(Eigen3 REQUIRED)
find_package(Glog)
find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED)
message(STATUS "Using opencv version ${OpenCV_VERSION}")


find_package( PCL 1.7 REQUIRED)
add_definitions( ${PCL_DEFINITIONS} )
link_directories( ${PCL_LIBRARY_DIRS} )


include_directories(
    ${PROJECT_SOURCE_DIR}
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${YAML_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} main.cpp lidar_camera_calib.cc)
target_link_libraries(${PROJECT_NAME}
    ${EIGEN3_LIBS}
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    ${YAML_CPP_LIBRARIES}
    yaml-cpp
    gflags
    glog
)
