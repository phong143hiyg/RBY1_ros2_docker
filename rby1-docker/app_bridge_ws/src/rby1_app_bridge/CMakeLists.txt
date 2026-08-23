cmake_minimum_required(VERSION 3.8)

project(rby1_app_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rby1_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(nlohmann_json REQUIRED)

include_directories(include)

add_executable(
  rby1_app_bridge_node
  src/main.cpp
)

ament_target_dependencies(
  rby1_app_bridge_node
  rclcpp
  rclcpp_action
  geometry_msgs
  sensor_msgs
  std_srvs
  rby1_msgs
)

target_link_libraries(
  rby1_app_bridge_node
  Boost::system
  nlohmann_json::nlohmann_json
)

install(
  TARGETS rby1_app_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(
    component_state_test
    test/component_state_test.cpp
  )
  target_include_directories(
    component_state_test
    PRIVATE include
  )
endif()

ament_package()
