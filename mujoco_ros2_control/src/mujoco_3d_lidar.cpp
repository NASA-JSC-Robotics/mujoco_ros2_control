/**
 * Copyright (c) 2025, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * All rights reserved.
 *
 * This software is licensed under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

#include "mujoco_ros2_control/mujoco_3d_lidar.hpp"
#include <limits>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include "mujoco_ros2_control/utils.hpp"

namespace mujoco_ros2_control
{

// Evenly spaced numbers over a specified interval.
void LinSpace(float lower, float upper, int n, std::vector<float>& array)
{
  if (static_cast<int>(array.size()) < n)
  {
    array.resize(n);
  }
  float increment = n > 1 ? (upper - lower) / static_cast<float>(n - 1) : 0.0F;
  for (int i = 0; i < n; ++i)
  {
    array[i] = lower;
    lower += increment;
  }
}

void ComputeVectors(Lidar3dConfig& lidar)
{
  lidar.vectors.clear();
  lidar.vectors.reserve(lidar.resolution[0] * lidar.resolution[1]);
  std::vector<float> azmuthAngles(lidar.resolution[0]);
  std::vector<float> elevationAngles(lidar.resolution[1]);

  if (lidar.resolution[0] > 1)
  {
    LinSpace(static_cast<float>(lidar.azimuth_range[0]), static_cast<float>(lidar.azimuth_range[1]),
             lidar.resolution[0], azmuthAngles);
  }
  else
  {
    azmuthAngles.push_back(0.0);
  }
  if (lidar.resolution[1] > 1)
  {
    LinSpace(static_cast<float>(lidar.elevation_range[0]), static_cast<float>(lidar.elevation_range[1]),
             lidar.resolution[1], elevationAngles);
  }
  else
  {
    elevationAngles.push_back(static_cast<float>(lidar.elevation_range[0]));
  }

  for (int32_t e = 0; e < lidar.resolution[1]; ++e)
  {
    for (int32_t a = 0; a < lidar.resolution[0]; ++a)
    {
      geometry_msgs::msg::Vector3 vec;
      vec.x = (std::cos(azmuthAngles[a]) * std::cos(elevationAngles[e]));
      vec.y = (std::sin(azmuthAngles[a]) * std::cos(elevationAngles[e]));
      vec.z = (std::sin(elevationAngles[e]));
      lidar.vectors.push_back(vec);
    }
  }
}

bool IsBigEndian(void)
{
  union
  {
    uint32_t i;
    char c[4];
  } bint = { 0x01020304 };

  return bint.c[0] == 1;
}

// Checks that a plugin config attribute exists.
bool CheckAttr(const std::string& input)
{
  char* end;
  std::string value = input;
  value.erase(std::remove_if(value.begin(), value.end(), isspace), value.end());
  value.erase(std::remove(value.begin(), value.end(), '\"'), value.end());
  strtod(value.c_str(), &end);
  return end == value.data() + value.size();
}

// Converts a string into a numeric vector
template <typename T>
void ReadVector(std::vector<T>& output, const std::string& input)
{
  std::stringstream ss(input);
  std::string item;
  char delim = ' ';
  while (getline(ss, item, delim))
  {
    CheckAttr(item);
    output.push_back(static_cast<T>(strtod(item.c_str(), nullptr)));
  }
}

/**
 * Construct a Lidar3dConfig object given a string sensor name and hardware_info object to parse.
 */
std::optional<Lidar3dConfig> get_lidar_config(const hardware_interface::HardwareInfo& hardware_info,
                                              const std::string& name)
{
  const auto sensor_info_maybe = get_sensor_from_info(hardware_info, name);
  if (!sensor_info_maybe.has_value())
  {
    return std::nullopt;
  }
  const auto sensor_info = sensor_info_maybe.value();

  auto get_parameter = [&](const std::string& key) -> std::optional<std::string> {
    if (auto it = sensor_info.parameters.find(key); it != sensor_info.parameters.end())
    {
      return it->second;
    }
    return std::nullopt;
  };

  Lidar3dConfig sensor;

  auto frame_name = get_parameter("frame_name");
  sensor.frame_name = frame_name.value_or("");

  auto lidar_topic = get_parameter("lidar_topic");
  sensor.lidar_topic = lidar_topic.value_or("");

  return sensor;
}

Mujoco3dLidar::Mujoco3dLidar(rclcpp::Node::SharedPtr node, std::recursive_mutex* sim_mutex, mjData* mujoco_data,
                             mjModel* mujoco_model, double lidar_publish_rate)
  : node_(node)
  , sim_mutex_(sim_mutex)
  , mj_data_(mujoco_data)
  , mj_model_(mujoco_model)
  , lidar_publish_rate_(lidar_publish_rate)
{
}

bool Mujoco3dLidar::register_lidars(const hardware_interface::HardwareInfo& hardware_info)
{
  lidar_sensors_.clear();
  for (int i = 0; i < mj_model_->nsensor; ++i)
  {
    // Skip sensors not controlled by plugin.
    if (mj_model_->sensor_type[i] != mjtSensor::mjSENS_PLUGIN)
    {
      continue;
    }
    // Grab the name of the sensor, which is required.
    const auto sensor_name_maybe = mj_id2name(mj_model_, mjtObj::mjOBJ_SENSOR, i);
    if (sensor_name_maybe == nullptr)
    {
      RCLCPP_WARN(node_->get_logger(), "Cannot find a name for lidar sensor at index: '%d' skipping!", i);
      continue;
    }

    auto new_data_maybe = get_lidar_config(hardware_info, sensor_name_maybe);
    if (!new_data_maybe.has_value())
    {
      RCLCPP_ERROR(node_->get_logger(), "Failed to parse required configuration from ros2_control xacro: '%s'",
                   sensor_name_maybe);
      return false;
    }
    auto lidar_config = new_data_maybe.value();

    if (lidar_config.frame_name.empty())
    {
      RCLCPP_ERROR(node_->get_logger(), "frame_name must be specified for sensor '%s'", sensor_name_maybe);
      return false;
    }

    // Pull the remaining configuration directly out of the mujoco plugin, where validation happens
    int plugin_instance = mj_model_->sensor_plugin[i];
    auto get_plugin_config = [&](const char* key) -> std::string {
      return std::string(mj_getPluginConfig(mj_model_, plugin_instance, key));
    };

    // resolution
    ReadVector(lidar_config.resolution, get_plugin_config("resolution"));
    if (lidar_config.resolution.size() != 2 || lidar_config.resolution[0] <= 0 || lidar_config.resolution[1] <= 0)
    {
      RCLCPP_ERROR(node_->get_logger(), "Invalid resolution provided for sensor '%s'", sensor_name_maybe);
      return false;
    }
    lidar_config.is_3d = lidar_config.resolution[1] > 1;

    // horizontal field of view
    ReadVector(lidar_config.azimuth_range, get_plugin_config("azimuth_range"));

    // vertical field of view
    ReadVector(lidar_config.elevation_range, get_plugin_config("elevation_range"));
    if (lidar_config.elevation_range.size() == 1)
    {
      lidar_config.elevation_range.push_back(lidar_config.elevation_range[0]);
    }
    else if (lidar_config.elevation_range.empty())
    {
      lidar_config.elevation_range = { 0.0, 0.0 };
    }

    // max/min ranges
    std::string max_range_str = get_plugin_config("max_range");
    lidar_config.range_max = max_range_str.empty() ? 1000.0 : std::atof(max_range_str.c_str());
    std::string min_range_str = get_plugin_config("min_range");
    lidar_config.range_min = min_range_str.empty() ? 0.0 : std::atof(min_range_str.c_str());
    if (lidar_config.range_max <= lidar_config.range_min)
    {
      RCLCPP_ERROR(node_->get_logger(), "Invalid ranges for sensor '%s'", sensor_name_maybe);
      return false;
    }

    // Setup remaining msg params and publisher for the sensor
    lidar_config.name = sensor_name_maybe;
    lidar_config.sensor_id = i;
    lidar_config.sensor_adr = mj_model_->sensor_adr[i];

    // Setup message and publisher
    if (lidar_config.is_3d)
    {
      lidar_config.lidar_topic = !lidar_config.lidar_topic.empty() ? lidar_config.lidar_topic : "/points";

      ComputeVectors(lidar_config);
      sensor_msgs::PointCloud2Modifier modifier(lidar_config.point_cloud_msg);

      modifier.setPointCloud2Fields(3, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
                                    sensor_msgs::msg::PointField::FLOAT32, "z", 1,
                                    sensor_msgs::msg::PointField::FLOAT32);

      // Configure the static parameters of the pointcloud message
      lidar_config.point_cloud_msg.header.frame_id = lidar_config.frame_name;
      lidar_config.point_cloud_msg.width = lidar_config.resolution[0];
      lidar_config.point_cloud_msg.height = lidar_config.resolution[1];
      lidar_config.point_cloud_msg.point_step = 12;
      lidar_config.point_cloud_msg.is_bigendian = IsBigEndian();
      lidar_config.point_cloud_msg.is_dense = false;
      lidar_config.point_cloud_msg.row_step =
          lidar_config.point_cloud_msg.point_step * lidar_config.point_cloud_msg.width;
      lidar_config.point_cloud_msg.data.resize(lidar_config.point_cloud_msg.row_step *
                                               lidar_config.point_cloud_msg.height);

      lidar_config.pointcloud_pub = node_->create_publisher<sensor_msgs::msg::PointCloud2>(lidar_config.lidar_topic, 1);
    }
    else
    {
      lidar_config.lidar_topic = !lidar_config.lidar_topic.empty() ? lidar_config.lidar_topic : "/scan";

      float angle_increment = static_cast<float>(lidar_config.azimuth_range[1] - lidar_config.azimuth_range[0]) /
                              static_cast<float>(lidar_config.resolution[0] - 1);

      // Configure the static parameters of the laserscan message
      lidar_config.laser_scan_msg.header.frame_id = lidar_config.frame_name;
      lidar_config.laser_scan_msg.time_increment = 0.0;  // Does this matter?
      lidar_config.laser_scan_msg.angle_min = static_cast<float>(lidar_config.azimuth_range[0]);
      lidar_config.laser_scan_msg.angle_max = static_cast<float>(lidar_config.azimuth_range[1]);
      lidar_config.laser_scan_msg.angle_increment = angle_increment;
      lidar_config.laser_scan_msg.range_min = static_cast<float>(lidar_config.range_min);
      lidar_config.laser_scan_msg.range_max = static_cast<float>(lidar_config.range_max);
      lidar_config.laser_scan_msg.ranges.resize(lidar_config.resolution[0] * lidar_config.resolution[1]);
      lidar_config.laser_scan_msg.intensities.resize(0);
      lidar_config.laser_scan_msg.scan_time = 1.0f / static_cast<float>(lidar_publish_rate_);

      lidar_config.scan_pub = node_->create_publisher<sensor_msgs::msg::LaserScan>(lidar_config.lidar_topic, 1);
    }

    // Note that we have added the sensor
    RCLCPP_INFO(node_->get_logger(), "Adding lidar sensor: '%s', idx: %d", lidar_config.name.c_str(),
                lidar_config.sensor_id);
    RCLCPP_INFO(node_->get_logger(), "         sensor_adr: '%d'", lidar_config.sensor_adr);
    RCLCPP_INFO(node_->get_logger(), "         frame_name: '%s'", lidar_config.frame_name.c_str());
    RCLCPP_INFO(node_->get_logger(), "      azimuth_range: %.4f - %.4f", lidar_config.azimuth_range[0],
                lidar_config.azimuth_range[1]);
    RCLCPP_INFO(node_->get_logger(), "    elevation_range: %.4f - %.4f", lidar_config.elevation_range[0],
                lidar_config.elevation_range[1]);
    RCLCPP_INFO(node_->get_logger(), "         resolution: %d - %d", lidar_config.resolution[0],
                lidar_config.resolution[1]);
    RCLCPP_INFO(node_->get_logger(), "          range_min: %f", lidar_config.range_min);
    RCLCPP_INFO(node_->get_logger(), "          range_max: %f", lidar_config.range_max);
    RCLCPP_INFO(node_->get_logger(), "              topic: %s", lidar_config.lidar_topic.c_str());

    lidar_sensors_.push_back(lidar_config);
  }

  // Allocate space for copied sensor data
  mj_lidar_data_.resize(mj_model_->nsensordata);

  return true;
}

void Mujoco3dLidar::init()
{
  // Start the background thread process
  publish_lidar_ = true;
  processing_thread_ = std::thread(&Mujoco3dLidar::update_loop, this);
}

void Mujoco3dLidar::close()
{
  publish_lidar_ = false;
  if (processing_thread_.joinable())
  {
    processing_thread_.join();
  }
}

void Mujoco3dLidar::update_loop()
{
  RCLCPP_INFO(node_->get_logger(), "Starting the lidar processing loop, publishing at %f Hz", lidar_publish_rate_);

  rclcpp::Rate rate(lidar_publish_rate_);
  while (rclcpp::ok() && publish_lidar_)
  {
    update();
    rate.sleep();
  }
}

void Mujoco3dLidar::update()
{
  // Step 1: Lock the sim and copy only the sensordata
  {
    std::unique_lock<std::recursive_mutex> lock(*sim_mutex_);
    std::memcpy(mj_lidar_data_.data(), mj_data_->sensordata, mj_lidar_data_.size() * sizeof(mjtNum));

    // Also grab when the scan was last completed
    for (Lidar3dConfig& lidar : lidar_sensors_)
    {
      int plugin_inst = mj_model_->sensor_plugin[lidar.sensor_id];
      lidar.last_compute_time = mj_data_->plugin_state[mj_model_->plugin_stateadr[plugin_inst]];
    }
  }

  // Step 2: Process and publish the appropriate type
  for (Lidar3dConfig& lidar : lidar_sensors_)
  {
    // If the scan is invalid or stale skip publishing
    // TODO: Just remove this when this is reworked as a plugin
    if (lidar.last_compute_time < 0.0 || lidar.last_compute_time == lidar.last_published_time)
    {
      continue;
    }
    lidar.last_published_time = lidar.last_compute_time;

    // Convert the update time to a ROS time stamp
    int32_t sec = static_cast<int32_t>(std::floor(lidar.last_compute_time));
    uint32_t nsec = static_cast<uint32_t>((lidar.last_compute_time - sec) * 1e9);
    rclcpp::Time stamp(sec, nsec, RCL_ROS_TIME);

    // Handle 3d lidar sensors
    if (lidar.is_3d && (static_cast<int>(lidar.vectors.size()) == lidar.resolution[0] * lidar.resolution[1]))
    {
      sensor_msgs::PointCloud2Iterator<float> iterX(lidar.point_cloud_msg, "x");
      sensor_msgs::PointCloud2Iterator<float> iterY(lidar.point_cloud_msg, "y");
      sensor_msgs::PointCloud2Iterator<float> iterZ(lidar.point_cloud_msg, "z");
      for (int i = 0; i < lidar.resolution[0] * lidar.resolution[1]; ++i)
      {
        float dist = static_cast<float>(mj_lidar_data_[lidar.sensor_adr + i]);
        if ((dist >= lidar.range_min) && (dist <= lidar.range_max))
        {
          *iterX = static_cast<float>(lidar.vectors[i].x * dist);
          *iterY = static_cast<float>(lidar.vectors[i].y * dist);
          *iterZ = static_cast<float>(lidar.vectors[i].z * dist);
        }
        else
        {
          *iterX = std::numeric_limits<float>::quiet_NaN();
          *iterY = std::numeric_limits<float>::quiet_NaN();
          *iterZ = std::numeric_limits<float>::quiet_NaN();
        }

        // Increment the iterators
        ++iterX;
        ++iterY;
        ++iterZ;
      }
      lidar.point_cloud_msg.header.stamp = stamp;
      lidar.pointcloud_pub->publish(lidar.point_cloud_msg);
    }
    else if (!lidar.is_3d)
    {
      for (int i = 0; i < lidar.resolution[0] * lidar.resolution[1]; ++i)
      {
        lidar.laser_scan_msg.ranges[i] = static_cast<float>(mj_lidar_data_[lidar.sensor_adr + i]);
      }

      // Publish
      lidar.laser_scan_msg.header.stamp = stamp;
      lidar.scan_pub->publish(lidar.laser_scan_msg);
    }
  }
}

}  // namespace mujoco_ros2_control
