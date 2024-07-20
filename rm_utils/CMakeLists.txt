cmake_minimum_required(VERSION 3.5)
project(rm_utils)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find package
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(Ceres REQUIRED)
find_package(rcpputils REQUIRED)
find_package(fmt REQUIRED)

# include
include_directories(include)

add_library(fytlogger SHARED
  src/logger/logger_impl.cpp
  src/logger/writer.cpp
  src/logger/logger_pool.cpp
)
target_link_libraries(fytlogger fmt::fmt)
ament_target_dependencies(fytlogger fmt)

# create rm_utils lib
add_library(${PROJECT_NAME} SHARED
  src/math/utils.cpp
  src/math/pnp_solver.cpp
  src/math/trajectory_compensator.cpp
  src/math/manual_compensator.cpp
  src/math/extended_kalman_filter.cpp
  src/url_resolver.cpp
  src/heartbeat.cpp
)

set(dependencies
  rclcpp
  rcpputils
  Eigen3
  eigen3_cmake_module
  geometry_msgs
  Ceres
  OpenCV
)

ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_include_directories(${PROJECT_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
  ${CERES_LIBRARIES}
)
# Install include directories
install(DIRECTORY include/
  DESTINATION include
)

# Install libraries
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(TARGETS fytlogger
  EXPORT fytlogger
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# export lib lib
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_targets(fytlogger HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies} fmt)

# test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_uncrustify
    ament_cmake_cpplint
  )
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()