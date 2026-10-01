cmake_minimum_required(VERSION 3.8)
project(ur3e_write_q)

find_package(ament_cmake REQUIRED)
find_package(moveit_ros_visualization REQUIRED)
find_package(rviz_common REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

add_library(q_motion_planning_display SHARED src/q_motion_planning_display.cpp)
target_compile_features(q_motion_planning_display PUBLIC cxx_std_17)
ament_target_dependencies(q_motion_planning_display rviz_common pluginlib rclcpp std_msgs)
target_link_libraries(q_motion_planning_display
  moveit_ros_visualization::moveit_motion_planning_rviz_plugin_core Qt5::Widgets)
pluginlib_export_plugin_description_file(rviz_common rviz_plugins.xml)
install(TARGETS q_motion_planning_display LIBRARY DESTINATION lib)

install(PROGRAMS scripts/draw_q.py scripts/plan_q.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
