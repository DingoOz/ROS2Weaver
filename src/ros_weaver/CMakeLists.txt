cmake_minimum_required(VERSION 3.8)
project(ros_weaver)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rosgraph_msgs REQUIRED)

# Qt5 setup
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

find_package(Qt5 REQUIRED COMPONENTS
  Core
  Gui
  Widgets
  Network
  Charts
)

# Include directories
include_directories(
  include
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Gui_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
)

# Source files
set(SOURCES
  src/main.cpp
  src/main_window.cpp
  src/canvas/weaver_canvas.cpp
  src/canvas/package_block.cpp
  src/canvas/connection_line.cpp
  src/canvas/node_group.cpp
  src/core/project.cpp
  src/core/ros_package_index.cpp
  src/core/code_generator.cpp
  src/core/external_editor.cpp
  src/core/topic_monitor.cpp
  src/core/system_discovery.cpp
  src/core/canvas_mapper.cpp
  src/core/theme_manager.cpp
  src/core/ollama_manager.cpp
  src/core/context_help.cpp
  src/core/data_lineage.cpp
  src/core/lineage_provider.cpp
  src/core/ai_tool_manager.cpp
  src/core/ai_context_provider.cpp
  src/core/undo/undo_command.cpp
  src/core/undo/undo_stack.cpp
  src/core/undo/commands/add_block_command.cpp
  src/core/undo/commands/remove_block_command.cpp
  src/core/undo/commands/move_block_command.cpp
  src/core/undo/commands/add_connection_command.cpp
  src/core/undo/commands/remove_connection_command.cpp
  src/core/undo/commands/add_group_command.cpp
  src/core/undo/commands/remove_group_command.cpp
  src/core/undo/commands/modify_group_command.cpp
  src/core/undo/commands/macro_command.cpp
  src/widgets/param_dashboard.cpp
  src/widgets/ai_permission_dialog.cpp
  src/widgets/lineage_dialog.cpp
  src/widgets/help_browser.cpp
  src/widgets/keyboard_shortcuts_dialog.cpp
  src/widgets/ollama_settings_widget.cpp
  src/widgets/system_prompt_dialog.cpp
  src/widgets/local_ai_status_widget.cpp
  src/widgets/llm_chat_widget.cpp
  src/widgets/output_panel.cpp
  src/widgets/topic_inspector.cpp
  src/widgets/ros_status_widget.cpp
  src/widgets/system_mapping_panel.cpp
  src/widgets/topic_viewer_panel.cpp
  src/widgets/topic_list_model.cpp
  src/widgets/tf_tree_panel.cpp
  src/widgets/plot_panel.cpp
  src/widgets/guided_tour.cpp
  src/widgets/topic_echo_dialog.cpp
  src/widgets/rosbag_workbench_panel.cpp
  src/widgets/timeline_widget.cpp
  src/widgets/topic_selector_widget.cpp
  src/widgets/playback_control_widget.cpp
  src/widgets/slam_config_widget.cpp
  src/widgets/bookmark_manager.cpp
  src/core/bag_manager.cpp
  src/core/bag_recorder.cpp
  src/core/playback_controller.cpp
  src/core/slam_pipeline_manager.cpp
  src/core/mcp_server.cpp
  src/core/mcp_manager.cpp
  src/core/mcp_providers.cpp
  src/wizards/package_wizard.cpp
  src/wizards/robot_wizard.cpp
  src/wizards/mcp_server_wizard.cpp
  src/widgets/mcp_explorer_panel.cpp
  src/widgets/ros_control_panel.cpp
  src/widgets/toast_notification.cpp
  src/widgets/typing_indicator.cpp
  src/widgets/command_palette.cpp
  src/widgets/empty_state.cpp
)

# Header files (for MOC)
set(HEADERS
  include/ros_weaver/main_window.hpp
  include/ros_weaver/canvas/weaver_canvas.hpp
  include/ros_weaver/canvas/package_block.hpp
  include/ros_weaver/canvas/connection_line.hpp
  include/ros_weaver/canvas/node_group.hpp
  include/ros_weaver/core/project.hpp
  include/ros_weaver/core/ros_package_index.hpp
  include/ros_weaver/core/code_generator.hpp
  include/ros_weaver/core/external_editor.hpp
  include/ros_weaver/core/topic_monitor.hpp
  include/ros_weaver/core/system_discovery.hpp
  include/ros_weaver/core/canvas_mapper.hpp
  include/ros_weaver/core/theme_manager.hpp
  include/ros_weaver/core/ollama_manager.hpp
  include/ros_weaver/core/context_help.hpp
  include/ros_weaver/core/data_lineage.hpp
  include/ros_weaver/core/lineage_provider.hpp
  include/ros_weaver/core/ai_tool_manager.hpp
  include/ros_weaver/core/ai_context_provider.hpp
  include/ros_weaver/core/undo/undo_stack.hpp
  include/ros_weaver/widgets/param_dashboard.hpp
  include/ros_weaver/widgets/ai_permission_dialog.hpp
  include/ros_weaver/widgets/lineage_dialog.hpp
  include/ros_weaver/widgets/help_browser.hpp
  include/ros_weaver/widgets/keyboard_shortcuts_dialog.hpp
  include/ros_weaver/widgets/ollama_settings_widget.hpp
  include/ros_weaver/widgets/system_prompt_dialog.hpp
  include/ros_weaver/widgets/local_ai_status_widget.hpp
  include/ros_weaver/widgets/llm_chat_widget.hpp
  include/ros_weaver/widgets/output_panel.hpp
  include/ros_weaver/widgets/topic_inspector.hpp
  include/ros_weaver/widgets/ros_status_widget.hpp
  include/ros_weaver/widgets/system_mapping_panel.hpp
  include/ros_weaver/widgets/topic_viewer_panel.hpp
  include/ros_weaver/widgets/topic_list_model.hpp
  include/ros_weaver/widgets/tf_tree_panel.hpp
  include/ros_weaver/widgets/plot_panel.hpp
  include/ros_weaver/widgets/guided_tour.hpp
  include/ros_weaver/widgets/topic_echo_dialog.hpp
  include/ros_weaver/widgets/rosbag_workbench_panel.hpp
  include/ros_weaver/widgets/timeline_widget.hpp
  include/ros_weaver/widgets/topic_selector_widget.hpp
  include/ros_weaver/widgets/playback_control_widget.hpp
  include/ros_weaver/widgets/slam_config_widget.hpp
  include/ros_weaver/widgets/bookmark_manager.hpp
  include/ros_weaver/core/bag_manager.hpp
  include/ros_weaver/core/bag_recorder.hpp
  include/ros_weaver/core/playback_controller.hpp
  include/ros_weaver/core/slam_pipeline_manager.hpp
  include/ros_weaver/core/mcp_types.hpp
  include/ros_weaver/core/mcp_server.hpp
  include/ros_weaver/core/mcp_manager.hpp
  include/ros_weaver/core/mcp_providers.hpp
  include/ros_weaver/wizards/package_wizard.hpp
  include/ros_weaver/wizards/robot_wizard.hpp
  include/ros_weaver/wizards/mcp_server_wizard.hpp
  include/ros_weaver/widgets/mcp_explorer_panel.hpp
  include/ros_weaver/widgets/ros_control_panel.hpp
  include/ros_weaver/widgets/toast_notification.hpp
  include/ros_weaver/widgets/typing_indicator.hpp
  include/ros_weaver/widgets/command_palette.hpp
  include/ros_weaver/widgets/empty_state.hpp
  include/ros_weaver/core/constants.hpp
)

# Create executable
add_executable(${PROJECT_NAME}
  ${SOURCES}
  ${HEADERS}
)

# Link libraries
target_link_libraries(${PROJECT_NAME}
  Qt5::Core
  Qt5::Gui
  Qt5::Widgets
  Qt5::Network
  Qt5::Charts
  yaml-cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  rcl_interfaces
  ament_index_cpp
  tf2
  tf2_ros
  tf2_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  rosbag2_cpp
  rosbag2_storage
  rosgraph_msgs
)

# Install targets
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files (if any)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

# Install example files
install(DIRECTORY
  examples
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Unit tests - Project serialization
  ament_add_gtest(test_project test/test_project.cpp src/core/project.cpp)
  target_include_directories(test_project PRIVATE include)
  target_link_libraries(test_project Qt5::Core Qt5::Gui)

  # Unit tests - Data lineage
  ament_add_gtest(test_data_lineage test/test_data_lineage.cpp src/core/data_lineage.cpp)
  target_include_directories(test_data_lineage PRIVATE include)
  target_link_libraries(test_data_lineage Qt5::Core Qt5::Gui)

  # Unit tests - Code generator (requires MOC for Q_OBJECT signals)
  ament_add_gtest(test_code_generator
    test/test_code_generator.cpp
    src/core/code_generator.cpp
    src/core/project.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/ros_weaver/core/code_generator.hpp
  )
  target_include_directories(test_code_generator PRIVATE include ${CMAKE_CURRENT_BINARY_DIR})
  target_link_libraries(test_code_generator Qt5::Core Qt5::Gui Qt5::Widgets)
  set_target_properties(test_code_generator PROPERTIES AUTOMOC ON)

  # Unit tests - Theme manager (requires MOC for Q_OBJECT signals)
  ament_add_gtest(test_theme_manager
    test/test_theme_manager.cpp
    src/core/theme_manager.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/ros_weaver/core/theme_manager.hpp
  )
  target_include_directories(test_theme_manager PRIVATE include ${CMAKE_CURRENT_BINARY_DIR})
  target_link_libraries(test_theme_manager Qt5::Core Qt5::Gui Qt5::Widgets)
  set_target_properties(test_theme_manager PROPERTIES AUTOMOC ON)

  # Unit tests - Lineage provider
  ament_add_gtest(test_lineage_provider
    test/test_lineage_provider.cpp
    src/core/lineage_provider.cpp
    src/core/data_lineage.cpp
    src/core/project.cpp
  )
  target_include_directories(test_lineage_provider PRIVATE include)
  target_link_libraries(test_lineage_provider Qt5::Core Qt5::Gui)

  # Unit tests - Undo stack (requires MOC for Q_OBJECT signals)
  ament_add_gtest(test_undo_stack
    test/test_undo_stack.cpp
    src/core/undo/undo_command.cpp
    src/core/undo/undo_stack.cpp
    src/core/undo/commands/macro_command.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/ros_weaver/core/undo/undo_stack.hpp
  )
  target_include_directories(test_undo_stack PRIVATE include ${CMAKE_CURRENT_BINARY_DIR})
  target_link_libraries(test_undo_stack Qt5::Core Qt5::Gui Qt5::Widgets)
  set_target_properties(test_undo_stack PROPERTIES AUTOMOC ON)

  # Lint tests (non-invasive ones only - skip copyright/cpplint/uncrustify for now)
  find_package(ament_cmake_xmllint REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_cppcheck REQUIRED)

  ament_xmllint()
  ament_lint_cmake()
  ament_cppcheck()
endif()

ament_package()
