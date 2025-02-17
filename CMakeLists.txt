cmake_minimum_required(VERSION 3.5)
project(r1_upper)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(comlib REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/M3508send.msg"
  "msg/M3508rec.msg"
  "msg/Cybergearrec.msg"
  "msg/Cybergearsend.msg"
 )

#  function(rosidl_get_typesupport_target var generate_interfaces_target typesupport_name)
#   if(NOT TARGET ${generate_interfaces_target})
#     message(FATAL_ERROR
#       "${generate_interfaces_target} is not a CMake target. Maybe rosidl_generate_interfaces was given a different target name?")
#   endif()

#   set(output_target "${generate_interfaces_target}__${typesupport_name}")

#   if(NOT TARGET ${output_target})
#     # CMake if() evaluates strings ending in `-NOTFOUND` as false
#     set(output_target "${output_target}-NOTFOUND")
#   endif()

#   set("${var}" "${output_target}" PARENT_SCOPE)
# endfunction()

ament_export_dependencies(rosidl_default_runtime)

link_directories("include/Motor/lib")

# # 设置库文件路径
set(LIBRARY_PATH "include/Motor/lib")

# 查找库文件
# echo 'export LD_LIBRARY_PATH=/home/coralab/R1_ws/src/r1_upper/include/Motor/lib:$LD_LIBRARY_PATH' >> ~/.bashrc
find_library(UNITREE_MOTOR_SDK NAMES libUnitreeMotorSDK_Linux64.so PATHS ${LIBRARY_PATH})
find_library(CONTROL_CAN NAMES libcontrolcan.so PATHS ${LIBRARY_PATH})

# Define EXTRA_LIBS variable
set(EXTRA_LIBS
  ${UNITREE_MOTOR_SDK}
  ${CONTROL_CAN}
)

set(dependencies
  rclcpp
  std_msgs
  rclcpp_action
  comlib
  rosidl_default_runtime
)

set(ARM_INCLUDE
include
include/Motor/lib
include/Motor/Cybergear
include/Public_function
include/Motor/M3508
include/shoot
include/Motor/Go/crc
include/Motor/Go/IOPort
include/Motor/Go/serialPort
include/Motor/Go/serialPort/include
include/Motor/Go/unitreeMotor
include/Motor/Go/unitreeMotor/include
include/r1_upper/msg
)

set(SOURCE
source/Public_function/Timer.cpp
source/shoot/Shoot.cpp
source/Motor/Go/GO_base.cpp
source/Motor/M3508/m3508_base.cpp
source/Motor/M3508/MotorPid.cpp
source/Motor/cybergear/can.cpp
source/Motor/cybergear/cybergear_base.cpp
source/Public_function/Public_function.cpp
)

set(TEST
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

# 添加执行目标
add_executable(mytest src/mytest.cpp ${TEST})
ament_target_dependencies(mytest ${dependencies} )
target_link_libraries(mytest  ${EXTRA_LIBS} )
target_include_directories(mytest PUBLIC ${ARM_INCLUDE})

add_executable(can_motor src/can_motor.cpp ${SOURCE})
ament_target_dependencies(can_motor ${dependencies} )
target_link_libraries(can_motor  ${EXTRA_LIBS} )
target_include_directories(can_motor PUBLIC ${ARM_INCLUDE})
target_link_libraries(can_motor "${cpp_typesupport_target}")

add_executable(manipulator src/manipulator.cpp ${SOURCE})
ament_target_dependencies(manipulator ${dependencies} )
target_link_libraries(manipulator ${EXTRA_LIBS} )
target_include_directories(manipulator PUBLIC ${ARM_INCLUDE})
target_link_libraries(manipulator "${cpp_typesupport_target}")

add_executable(new_manipulator src/new_manipulator.cpp ${SOURCE})
ament_target_dependencies(new_manipulator ${dependencies} )
target_link_libraries(new_manipulator ${EXTRA_LIBS} )
target_include_directories(new_manipulator PUBLIC ${ARM_INCLUDE})
target_link_libraries(new_manipulator "${cpp_typesupport_target}")

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(
  TARGETS mytest manipulator can_motor new_manipulator
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
