find_package(ament_cmake_gtest REQUIRED)

ament_add_gtest(test_plugin test_plugin.cpp)
target_link_libraries(test_plugin
  mujoco_ros2_control
)
ament_target_dependencies(test_plugin
  rclcpp
)


ament_add_gtest(test_headless_init test_headless_init.cpp)
target_link_libraries(test_headless_init
  mujoco_ros2_control
)
ament_target_dependencies(test_headless_init
  rclcpp
  hardware_interface
)
