#ifndef MUJOCO_PLUGIN_SENSOR_LIDAR_H_
#define MUJOCO_PLUGIN_SENSOR_LIDAR_H_

#include <optional>
#include <vector>

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mjvisualize.h>

namespace mujoco::plugin::lidar {

// The sensor has 6 parameters:
//  2. (int) Horizontal resolution.
//  3. (int) Vertical resolution.
//  4. (double) Horizontal field-of-view (fov_x), in degrees.
//  5. (double) Vertical field-of-view (fov_y), in degrees.

class Lidar {
public:
  static Lidar *Create(const mjModel *m, mjData *d, int instance);
  Lidar(Lidar &&) = default;
  ~Lidar() = default;

  void Reset(const mjModel *m, int instance);
  void Compute(const mjModel *m, mjData *d, int instance);
  void Visualize(const mjModel *m, mjData *d, const mjvOption *opt,
                 mjvScene *scn, int instance);

  static void RegisterPlugin();

  int resolution_[2]; // horizontal and vertical resolution
  mjtNum fov_[2];     // horizontal and vertical field of view, in degrees
  mjtNum max_range_;  // max range of lidar

private:
  Lidar(const mjModel *m, mjData *d, int instance, int *size,
        mjtNum *azimuth_range, mjtNum *elevation_range, mjtNum max_range);
  std::vector<mjtNum> vectors_;
  std::vector<mjtNum> rotated_vectors_;
};

} // namespace mujoco::plugin::lidar

#endif // MUJOCO_PLUGIN_SENSOR_LIDAR_H_
