cmake_minimum_required(VERSION 3.22)
project(mujoco_3d_lidar)

# Link MuJoCo via the vendor package
find_package(mujoco_vendor REQUIRED)
find_package(mujoco_ros2_control REQUIRED)

message("mujoco_ros2_control_DIR = ${mujoco_ros2_control_DIR}")
set(mujoco_ros2_control_BIN_DIR ${mujoco_ros2_control_DIR}/../../../lib/mujoco_ros2_control)
cmake_path(SET mujoco_ros2_control_BIN_DIR NORMALIZE ${mujoco_ros2_control_DIR}/../../../lib/mujoco_ros2_control)
message("mujoco_ros2_control_BIN_DIR = ${mujoco_ros2_control_BIN_DIR}")

# We don't care about warnings from the external sources
set(mujoco_SILENCE_COMPILER_WARNINGS
  -Wno-missing-field-initializers
  -Wno-unused-parameter
  -Wno-sign-compare
  -Wno-psabi
)

set(SRCS
    src/register.cpp
    src/3dlidar.cpp
)

include_directories(include)

add_library(mujoco_plugin_lidar SHARED)
target_sources(mujoco_plugin_lidar PRIVATE ${SRCS})
target_link_libraries(mujoco_plugin_lidar PRIVATE mujoco::mujoco)
target_compile_options(mujoco_plugin_lidar PRIVATE ${mujoco_SILENCE_COMPILER_WARNINGS})

add_executable(list_plugins test/list_plugins.cpp)
target_compile_options(list_plugins PRIVATE ${mujoco_SILENCE_COMPILER_WARNINGS})
target_link_libraries(
  list_plugins PRIVATE
  mujoco::mujoco
)

add_executable(load_model test/load_model.cpp)
target_compile_options(load_model PRIVATE ${mujoco_SILENCE_COMPILER_WARNINGS})
target_link_libraries(
  load_model PRIVATE
  mujoco::mujoco
  glfw
)

install(
  TARGETS mujoco_plugin_lidar
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY DESTINATION ${mujoco_ros2_control_BIN_DIR}/mujoco_plugin)
install(TARGETS mujoco_plugin_lidar
  DESTINATION ${mujoco_ros2_control_BIN_DIR}/mujoco_plugin
  PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)

# Duplicate the install to be next to the simulate application so this works there, as well.
find_program(mujoco_SIMULATE NAMES simulate mujoco-simulate)
message("mujoco_SIMULATE = ${mujoco_SIMULATE}")

cmake_path(SET mujoco_BIN_PATH NORMALIZE ${mujoco_SIMULATE}/..)
#cmake_path(GET ${mujoco_SIMULATE} ROOT_DIRECTORY mujoco_BIN_PATH)
message("mujoco_BIN_PATH = ${mujoco_BIN_PATH}")

install(DIRECTORY DESTINATION ${mujoco_BIN_PATH}/mujoco_plugin)
install(TARGETS mujoco_plugin_lidar
  DESTINATION ${mujoco_BIN_PATH}/mujoco_plugin
  PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)

ament_package()
