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
find_package(r1_interfaces REQUIRED)

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

# ── R1 transport library ──────────────────────────────────────────────────────
# Header-only templates plus the Factory singleton and built-in type
# registrations (design §7.2). Shared library so the registry has exactly one
# definition per process (F5).
add_library(r1_control_signal_transport SHARED
  src/r1/control_signal_factory.cpp
  src/r1/control_signal_types.cpp
)
target_include_directories(r1_control_signal_transport PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(r1_control_signal_transport
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  r1_interfaces
)

# ── csm_master_node (standalone executable, §9/§2.1) ─────────────────────────
add_executable(csm_master_node
  src/r1/csm_master.cpp
)
target_include_directories(csm_master_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(csm_master_node rclcpp r1_interfaces)
install(TARGETS csm_master_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
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
install(TARGETS control_signal_transport r1_control_signal_transport
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_libraries(control_signal_transport r1_control_signal_transport)
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
    test/unit/test_liveness_state.cpp
  )
  target_include_directories(test_liveness_state PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )

  # R1: Info validation unit tests (design §3.3, V1-V11). Pure logic — only
  # the generated message header, no rclcpp init (T3.1 check).
  ament_add_gtest(test_info_validation
    test/unit/test_info_validation.cpp
  )
  target_include_directories(test_info_validation PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  ament_target_dependencies(test_info_validation r1_interfaces)

  # R1: Source/Sink transport unit tests (design §5.4 S1-S16 suite SourceTest,
  # §6.4 K1-K16 suite SinkTest) via the ManagerTestAccess tick simulation.
  ament_add_gtest(test_transport
    test/unit/test_transport.cpp
    TIMEOUT 300
  )
  target_include_directories(test_transport PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${CMAKE_CURRENT_SOURCE_DIR}/test
  )
  ament_target_dependencies(test_transport rclcpp sensor_msgs r1_interfaces)

  # R1: Factory unit tests (design §7.3, F1-F5); links the shared library so
  # F5 exercises the cross-TU singleton.
  ament_add_gtest(test_factory
    test/unit/test_factory.cpp
  )
  target_include_directories(test_factory PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${CMAKE_CURRENT_SOURCE_DIR}/test
  )
  ament_target_dependencies(test_factory
    rclcpp sensor_msgs geometry_msgs std_msgs r1_interfaces)
  target_link_libraries(test_factory r1_control_signal_transport)

  # R1: Manager integration tests (design §8.4, M1-M26).
  ament_add_gtest(test_manager
    test/integration/test_manager.cpp
    TIMEOUT 600
  )
  target_include_directories(test_manager PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${CMAKE_CURRENT_SOURCE_DIR}/test
  )
  ament_target_dependencies(test_manager
    rclcpp sensor_msgs geometry_msgs std_msgs r1_interfaces)
  target_link_libraries(test_manager r1_control_signal_transport)

  # R1: Handle contract unit tests (design §10.4, H1-H6).
  ament_add_gtest(test_handles
    test/unit/test_handles.cpp
    TIMEOUT 600
  )
  target_include_directories(test_handles PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${CMAKE_CURRENT_SOURCE_DIR}/test
  )
  ament_target_dependencies(test_handles
    rclcpp sensor_msgs geometry_msgs std_msgs r1_interfaces)
  target_link_libraries(test_handles r1_control_signal_transport)

  # R1: Handle retry lifecycle integration tests (design §10.4, H7-H8).
  ament_add_gtest(test_handles_lifecycle
    test/integration/test_handles_lifecycle.cpp
    TIMEOUT 600
  )
  target_include_directories(test_handles_lifecycle PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${CMAKE_CURRENT_SOURCE_DIR}/test
  )
  ament_target_dependencies(test_handles_lifecycle
    rclcpp sensor_msgs geometry_msgs std_msgs r1_interfaces)
  target_link_libraries(test_handles_lifecycle r1_control_signal_transport)

  # R1: CsmMaster integration tests (design §9.4, CM1-CM13; mock CSM bare nodes).
  ament_add_gtest(test_csm_master
    test/integration/test_csm_master.cpp
    TIMEOUT 600
  )
  target_include_directories(test_csm_master PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${CMAKE_CURRENT_SOURCE_DIR}/test
  )
  ament_target_dependencies(test_csm_master
    rclcpp sensor_msgs geometry_msgs std_msgs r1_interfaces)
  target_link_libraries(test_csm_master r1_control_signal_transport)

  # Unit tests for the transport primitives (Source/Sink + factory functions).
  ament_add_gtest(test_control_signal_transport
    test/unit/test_control_signal_transport.cpp
  )
  target_include_directories(test_control_signal_transport PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${CMAKE_CURRENT_SOURCE_DIR}/test
  )
  ament_target_dependencies(test_control_signal_transport
    rclcpp sensor_msgs geometry_msgs std_msgs rv2_interfaces)
  target_link_libraries(test_control_signal_transport control_signal_transport)

  # Integration tests for ControlSignalManager (inter-CSM registration protocol).
  ament_add_gtest(test_control_signal_manager
    test/integration/test_control_signal_manager.cpp
  )
  target_include_directories(test_control_signal_manager PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${CMAKE_CURRENT_SOURCE_DIR}/test
  )
  ament_target_dependencies(test_control_signal_manager
    rclcpp sensor_msgs geometry_msgs std_msgs rv2_interfaces)
  target_link_libraries(test_control_signal_manager control_signal_transport)

  # Append category labels without replacing ament's existing gtest labels.
  foreach(unit_test test_liveness_state test_info_validation test_transport
                    test_factory test_handles test_control_signal_transport)
    set_property(TEST ${unit_test} APPEND PROPERTY LABELS unit)
  endforeach()
  foreach(integration_test test_manager test_handles_lifecycle test_csm_master
                           test_control_signal_manager)
    set_property(TEST ${integration_test} APPEND PROPERTY LABELS integration)
  endforeach()
endif()

ament_package()
