cmake_minimum_required(VERSION 3.8)
project(rm_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Armor.msg"
  "msg/Armors.msg"
  "msg/Target.msg"
  "msg/RuneTarget.msg"
  "msg/Point2d.msg"
  "msg/GimbalCmd.msg"
  "msg/ChassisCmd.msg"
  "msg/DebugLight.msg"
  "msg/DebugLights.msg"
  "msg/DebugArmor.msg"
  "msg/DebugArmors.msg"
  "msg/DebugRuneAngle.msg"
  "msg/Measurement.msg"
  "msg/JudgeSystemData.msg"
  "msg/OperatorCommand.msg"
  "msg/SerialReceiveData.msg"
  "srv/SetMode.srv"
  DEPENDENCIES
    std_msgs
    geometry_msgs
)


ament_package()
