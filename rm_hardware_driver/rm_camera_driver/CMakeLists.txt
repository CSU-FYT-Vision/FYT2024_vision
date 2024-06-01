cmake_minimum_required(VERSION 3.8)
project(rm_camera_driver)

## Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror -O3)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(OpenCV REQUIRED)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/daheng_camera.cpp
  src/recorder.cpp
  src/video_player.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC include ${OpenCV_INCLUDE_DIRS})

if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  target_link_directories(${PROJECT_NAME} PUBLIC lib/x86_64)
  install(
    DIRECTORY lib/x86_64
    DESTINATION lib
  )
#elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
#  target_link_directories(${PROJECT_NAME} PUBLIC hikSDK/lib/arm64)
#  install(
#    DIRECTORY hikSDK/lib/arm64/
#    DESTINATION lib
#  )
else()
  message(FATAL_ERROR "Unsupport host system architecture: ${CMAKE_HOST_SYSTEM_PROCESSOR}!")
endif()

target_link_libraries(${PROJECT_NAME}
  libgxiapi.so
  ${OpenCV_LIBS}
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN fyt::camera_driver::DahengCameraNode
  EXECUTABLE ${PROJECT_NAME}_node
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN fyt::camera_driver::VideoPlayerNode
  EXECUTABLE video_player_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_cpplint
    ament_cmake_uncrustify
  )
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
)
