cmake_minimum_required(VERSION 3.12)

set(CMAKE_CXX_FLAGS_RELEASE "-O3 -fPIC")

find_package(Boost REQUIRED COMPONENTS filesystem program_options system)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)

# set(EIGEN_INCLUDE_DIR "/usr/local/include/eigen3")

# catkin_package(
#     INCLUDE_DIRS include
#     LIBRARIES camera_model
#     CATKIN_DEPENDS roscpp std_msgs
# #    DEPENDS system_lib
#     )

# include_directories(
#     ${catkin_INCLUDE_DIRS}
#     )

add_library(camera_model OBJECT
    src/chessboard/Chessboard.cc
    src/calib/CameraCalibration.cc
    src/calib/StereoCameraCalibration.cc
    src/camera_models/Camera.cc
    src/camera_models/CameraFactory.cc
    src/camera_models/CostFunctionFactory.cc
    src/camera_models/PinholeCamera.cc
    src/camera_models/CataCamera.cc
    src/camera_models/EquidistantCamera.cc
    src/camera_models/ScaramuzzaCamera.cc
    src/sparse_graph/Transform.cc
    src/gpl/gpl.cc
    src/gpl/EigenQuaternionParameterization.cc)

target_compile_features(camera_model PRIVATE cxx_std_14)
target_include_directories(camera_model PRIVATE
        ${Boost_INCLUDE_DIRS}
        ${CERES_INCLUDE_DIRS}
        PUBLIC include
        )
target_link_libraries(camera_model PRIVATE
        ${Boost_LIBRARIES}
        ${OpenCV_LIBS}
        ${CERES_LIBRARIES}
        )
