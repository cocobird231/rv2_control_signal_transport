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

# ── core transport library ────────────────────────────────────────────────────
# The transport core (control_signal_transport.h / control_signal_manager.h) is
# header-only and type-agnostic. The concrete control-signal types (Joy/Twist/
# String) are registered in control_signal_types.cpp via ControlSignalFactory.
# Build it as a shared library so the singleton registry is shared across all
# consumers in the same process.
add_library(control_signal_transport SHARED
  src/control_signal_factory.cpp
  src/control_signal_types.cpp
)
target_include_directories(control_signal_transport PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(control_signal_transport
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  rv2_interfaces
)

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
)
target_link_libraries(keyboard_source_component control_signal_transport)
rclcpp_components_register_node(keyboard_source_component
  PLUGIN "KeyboardSourceNode"
  EXECUTABLE keyboard_source
)
install(TARGETS keyboard_source_component keyboard_source
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# ── public headers + exported library target ──────────────────────────────────
install(DIRECTORY include/ DESTINATION include)
install(TARGETS control_signal_transport
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_libraries(control_signal_transport)
ament_export_dependencies(
  rclcpp rclcpp_components
  sensor_msgs geometry_msgs std_msgs
  rv2_interfaces)

# ── launch files ──────────────────────────────────────────────────────────────
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  # R1: LivenessState unit tests (design §4.5, L1-L18). Pure logic — no
  # rclcpp / interface dependencies on purpose (T2.1 check).
  ament_add_gtest(test_liveness_state
    test/r1/test_liveness_state.cpp
  )
  target_include_directories(test_liveness_state PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )

  # Unit tests for the transport primitives (Source/Sink + factory functions).
  ament_add_gtest(test_control_signal_transport
    test/test_control_signal_transport.cpp
  )
  target_include_directories(test_control_signal_transport PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${CMAKE_CURRENT_SOURCE_DIR}/test
  )
  ament_target_dependencies(test_control_signal_transport
    rclcpp sensor_msgs geometry_msgs std_msgs rv2_interfaces)
  target_link_libraries(test_control_signal_transport control_signal_transport)

  # Unit tests for ControlSignalManager (inter-CSM registration protocol).
  ament_add_gtest(test_control_signal_manager
    test/test_control_signal_manager.cpp
  )
  target_include_directories(test_control_signal_manager PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${CMAKE_CURRENT_SOURCE_DIR}/test
  )
  ament_target_dependencies(test_control_signal_manager
    rclcpp sensor_msgs geometry_msgs std_msgs rv2_interfaces)
  target_link_libraries(test_control_signal_manager control_signal_transport)
endif()

ament_package()
