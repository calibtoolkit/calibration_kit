cmake_minimum_required(VERSION 3.12)

find_package(Eigen3 REQUIRED)

find_package(PCL 1.8 REQUIRED)
link_directories(${PCL_LIBRARY_DIRS})

add_library(lidar2lidar OBJECT lidar2lidar.cpp)
target_include_directories(lidar2lidar PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(lidar2lidar Eigen3::Eigen ${PCL_LIBRARIES})
target_compile_features(lidar2lidar PRIVATE cxx_std_14)

#add_executable(pcd_convert convert.cpp)
#target_link_libraries(pcd_convert Eigen3::Eigen ${PCL_LIBRARIES})
