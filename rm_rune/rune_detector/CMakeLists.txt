cmake_minimum_required(VERSION 3.10)
project(rune_detector)

## Use C++14
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror -O3)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenVINO REQUIRED COMPONENTS Runtime ONNX)
ament_auto_find_build_dependencies()

###########
## Build ##
###########

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${EIGEN3_LIBS}   openvino::frontend::onnx openvino::runtime)
target_include_directories(${PROJECT_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN fyt::rune::RuneDetectorNode
  EXECUTABLE rune_detector_node
)

#############
## Testing ##
#############


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_uncrustify
    ament_cmake_cpplint
  )
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest)
  ament_add_gtest(test_detector test/test_detector.cpp)
  target_link_libraries(test_detector ${PROJECT_NAME})

endif()


ament_auto_package(
  INSTALL_TO_SHARE
  model
  docs
)

