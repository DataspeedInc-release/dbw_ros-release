cmake_minimum_required(VERSION 3.22)
project(ds_dbw_can)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(can_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ds_dbw_msgs REQUIRED)
find_package(dataspeed_can_msg_filters REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
  message(STATUS "Enabling coverage testing")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage")
endif()

add_library(${PROJECT_NAME} SHARED
  "src/DbwNode.cpp"
  "src/CanSocket.cpp"
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${dataspeed_can_msg_filters_INCLUDE_DIRS}>
)
target_link_libraries(${PROJECT_NAME}
  ${std_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${can_msgs_TARGETS}
  ${ds_dbw_msgs_TARGETS}
  ${dataspeed_can_msg_filters_TARGETS}
  rclcpp::rclcpp
  rclcpp_components::component
  rclcpp_components::component_manager
)

ament_export_dependencies(
  rclcpp
  rclcpp_components
  std_msgs
  can_msgs
  sensor_msgs
  ds_dbw_msgs
)

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "${PROJECT_NAME}::DbwNode"
  EXECUTABLE dbw_node
)

ament_export_include_directories(include)
install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY scripts
  DESTINATION lib/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(${PROJECT_NAME}_test_can_msg_recv "tests/test_can_msg_recv.cpp")
  target_include_directories(${PROJECT_NAME}_test_can_msg_recv PUBLIC "include")
  target_link_libraries(${PROJECT_NAME}_test_can_msg_recv
    ${std_msgs_TARGETS}
    rclcpp::rclcpp
  )

  ament_add_gtest(${PROJECT_NAME}_test_module_version "tests/test_module_version.cpp")
  target_include_directories(${PROJECT_NAME}_test_module_version PUBLIC "include")

  ament_add_gtest(${PROJECT_NAME}_test_platform_map "tests/test_platform_map.cpp")
  target_include_directories(${PROJECT_NAME}_test_platform_map PUBLIC "include")

  ament_add_gtest(${PROJECT_NAME}_test_platform_version "tests/test_platform_version.cpp")
  target_include_directories(${PROJECT_NAME}_test_platform_version PUBLIC "include")
endif()

ament_package()