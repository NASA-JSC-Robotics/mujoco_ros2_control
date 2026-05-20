#ifndef MUJOCO_PLUGIN_SENSOR_LIDAR_H_
#define MUJOCO_PLUGIN_SENSOR_LIDAR_H_

#include <optional>
#include <vector>

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mjvisualize.h>

namespace mujoco::plugin::lidar
{

// The sensor has a number of parameters:
//  1. (int) Horizontal resolution.
//  2. (int) Vertical resolution.
//  3. (double) Horizontal field-of-view (fov_x), in degrees.
//  4. (double) Vertical field-of-view (fov_y), in degrees.
//  5. (double) Maximum range, in m.
//  6. (double) Minimum range, in m (optional, defaults to 0).
//  7. (double) Rate at which to update sensor readings, in seconds (optional, defaults to 0).

class Lidar
{
public:
  static Lidar* Create(const mjModel* m, mjData* d, int instance);
  Lidar(Lidar&&) = default;
  ~Lidar() = default;

  void Reset(const mjModel* m, int instance);
  void Compute(const mjModel* m, mjData* d, int instance);
  void Visualize(const mjModel* m, mjData* d, const mjvOption* opt, mjvScene* scn, int instance);

  static void RegisterPlugin();

private:
  Lidar(const mjModel* m, mjData* d, int instance, int resolution[2], mjtNum azimuth_range[2],
        mjtNum elevation_range[2], mjtNum max_range, mjtNum min_range, mjtNum update_rate);
  std::vector<mjtNum> vectors_;
  std::vector<mjtNum> rotated_vectors_;

  int resolution_[2];         // horizontal and vertical resolution
  mjtNum fov_[2];             // horizontal and vertical field of view, in degrees
  mjtNum max_range_;          // max range of lidar
  mjtNum min_range_;          // min range of lidar
  mjtNum update_period_;      // Update period to run
  mjtNum last_compute_time_;  // Sim time of the last reading
};

}  // namespace mujoco::plugin::lidar

#endif  // MUJOCO_PLUGIN_SENSOR_LIDAR_H_
