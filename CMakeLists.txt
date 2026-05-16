cmake_minimum_required(VERSION 3.8)
project(rv2_control_signal_transport)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── dependencies ──────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rv2_interfaces REQUIRED)
find_package(rv2_server_control REQUIRED)

# ── keyboard_source (composable node) ─────────────────────────────────────────
add_library(keyboard_source_component SHARED
  src/keyboard_source_node.cpp
)
target_include_directories(keyboard_source_component PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(keyboard_source_component
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  std_msgs
  rv2_interfaces
  rv2_server_control
)
rclcpp_components_register_node(keyboard_source_component
  PLUGIN "KeyboardSourceNode"
  EXECUTABLE keyboard_source
)
install(TARGETS keyboard_source_component keyboard_source
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# ── public headers ────────────────────────────────────────────────────────────
install(DIRECTORY include/ DESTINATION include)
ament_export_include_directories(include)

# ── launch files ──────────────────────────────────────────────────────────────
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
