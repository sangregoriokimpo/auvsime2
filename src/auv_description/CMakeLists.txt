cmake_minimum_required(VERSION 3.8)
project(auv_description)

# C++ settings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)

# --- Dependencies ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# Gazebo Harmonic (gz-sim 8)
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED)
find_package(gz-math7 REQUIRED)




# --- Plugin library ---
add_library(wasd_body_thrust_plugin SHARED
  src/wasd_body_thrust_plugin.cpp
)

ament_target_dependencies(wasd_body_thrust_plugin
  rclcpp
  geometry_msgs
  gz-sim8
  gz-plugin2
  gz-math7
)

# Make sure the .so is discoverable by Gazebo/ROS
install(TARGETS wasd_body_thrust_plugin
  EXPORT wasd_body_thrust_pluginTargets
  LIBRARY DESTINATION lib
)




# --- Keyboard teleop script ---
# (Put the file at: scripts/wasd_teleop.py and make it executable)
install(PROGRAMS
  scripts/wasd_teleop.py
  DESTINATION lib/${PROJECT_NAME}
)

# --- Share description assets (your original lines) ---
install(DIRECTORY urdf meshes launch worlds
  DESTINATION share/${PROJECT_NAME}
)

# --- Testing (unchanged) ---
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(wasd_body_thrust_pluginTargets)
ament_export_libraries(wasd_body_thrust_plugin)


ament_package()