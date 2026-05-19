#include <mujoco/mjplugin.h>
#include "mujoco_lidar_plugin/3dlidar.h"

namespace mujoco::plugin::lidar
{
mjPLUGIN_LIB_INIT
{
  Lidar::RegisterPlugin();
}

}  // namespace mujoco::plugin::lidar
