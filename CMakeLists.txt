cmake_minimum_required(VERSION 3.22)
project(ds_dbw_joystick_demo)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ds_dbw_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
  message(STATUS "Enabling coverage testing")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage")
endif()

add_library(${PROJECT_NAME} SHARED
  "src/JoystickDemo.cpp"
)

target_link_libraries(${PROJECT_NAME}
  ${std_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${ds_dbw_msgs_TARGETS}
  rclcpp::rclcpp
  rclcpp_components::component
  rclcpp_components::component_manager
)

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "${PROJECT_NAME}::JoystickDemo"
  EXECUTABLE joy_demo
)

ament_package()
