cmake_minimum_required(VERSION 3.8)
project(crane_msgs)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

set(msg_files
  "msg/CollisionPrimitive.msg"
  "msg/CollisionScene.msg"
  "msg/Payload.msg"
  "msg/PayloadEstimate.msg"
  "msg/PendulumState.msg"
  "msg/SolverHealth.msg"
  "msg/SupervisorStatus.msg"
  "msg/SwaySettled.msg"
  "msg/VelocityControllerHealth.msg"
)

set(srv_files
  "srv/PlanGrip.srv"
  "srv/PlanMotion.srv"
  "srv/SetMode.srv"
  "srv/SetPayload.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES geometry_msgs std_msgs trajectory_msgs
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(test_contract test/test_contract.cpp)
  target_link_libraries(test_contract "${PROJECT_NAME}__rosidl_generator_cpp")
  add_dependencies(test_contract ${PROJECT_NAME})
  ament_add_pytest_test(test_schema test/test_schema.py)
  ament_add_pytest_test(test_ros_contract test/test_ros_contract.py)
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
