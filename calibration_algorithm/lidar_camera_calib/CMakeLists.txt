cmake_minimum_required(VERSION 3.12)

# Eigen, actually it's no need to find it'
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED)
message(STATUS "Using opencv version ${OpenCV_VERSION}")

find_package(PCL 1.7 REQUIRED)
link_directories( ${PCL_LIBRARY_DIRS} )

add_library(lidar_camera_calib OBJECT lidar_camera_calib.cc)

target_compile_features(lidar_camera_calib PRIVATE cxx_std_14)
target_compile_definitions(lidar_camera_calib PRIVATE ${PCL_DEFINITIONS})

target_include_directories(lidar_camera_calib PRIVATE
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${YAML_INCLUDE_DIRS}
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(lidar_camera_calib
    ${EIGEN3_LIBS}
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    yaml-cpp
)
