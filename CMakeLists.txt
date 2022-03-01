cmake_minimum_required (VERSION 3.12)

# --- Fetch FTXUI --------------------------------------------------------------
include(FetchContent)

FetchContent_Declare(ftxui
        GIT_REPOSITORY https://github.com/ArthurSonzogni/ftxui
        GIT_TAG v2.0.0
        )

FetchContent_GetProperties(ftxui)
if(NOT ftxui_POPULATED)
  FetchContent_Populate(ftxui)
  add_subdirectory(${ftxui_SOURCE_DIR} ${ftxui_BINARY_DIR} EXCLUDE_FROM_ALL)
endif()

# ------------------------------------------------------------------------------

project(calibration_kit
  LANGUAGES CXX
  VERSION 0.0.1
)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE release)
ENDIF()

add_subdirectory(calibration_algorithm/lidar_camera_calib)

add_executable(calibration_kit src/main.cpp)
target_compile_features(calibration_kit PRIVATE cxx_std_17)

target_link_libraries(calibration_kit PRIVATE
  ftxui::screen
  ftxui::dom
  ftxui::component
  lidar_camera_calib
)

#install(TARGETS ftxui-starter RUNTIME DESTINATION "bin")
