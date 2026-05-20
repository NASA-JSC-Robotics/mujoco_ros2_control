cmake_minimum_required(VERSION 3.22)
project(mujoco_3d_lidar)

find_package(ament_cmake REQUIRED)
# Link MuJoCo via the vendor package
find_package(mujoco_vendor REQUIRED)

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

# Install in the local package
install(
  TARGETS mujoco_plugin_lidar
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
)

# Duplicate the install to be next to the simulate application so this works there, as well.
find_program(mujoco_SIMULATE NAMES simulate mujoco-simulate)
message("mujoco_SIMULATE = ${mujoco_SIMULATE}")
message("mujoco_BIN_PATH = ${mujoco_BIN_PATH}")
cmake_path(SET mujoco_BIN_PATH NORMALIZE ${mujoco_SIMULATE}/..)
install(DIRECTORY DESTINATION ${mujoco_BIN_PATH}/mujoco_plugin)
install(TARGETS mujoco_plugin_lidar DESTINATION ${mujoco_BIN_PATH}/mujoco_plugin)

ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(mujoco_vendor)
ament_package()
