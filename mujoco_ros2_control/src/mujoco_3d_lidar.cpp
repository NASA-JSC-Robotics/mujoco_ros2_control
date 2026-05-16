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
#include "mujoco_ros2_control/utils.hpp"

namespace mujoco_ros2_control
{

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
    printf("Item : %s\n", item.c_str());
    output.push_back(strtod(item.c_str(), nullptr));
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

  std::optional<std::string> frame_name = get_parameter("frame_name");
  std::optional<std::string> laserscan_topic = get_parameter("laserscan_topic");

  // resolution
  int resolution[2] = { 1, 1 };
  std::optional<std::string> res_str = get_parameter("resolution");
  if (!res_str.has_value())
  {
    fprintf(stderr, "Resolution must be specified\n");
    return std::nullopt;
  }

  int res = sscanf(res_str.value().c_str(), "%d %d", &resolution[0], &resolution[1]);
  if (res != 2)
  {
    fprintf(stderr, "Both horizontal and vertical resolutions must be specified");
    return std::nullopt;
    ;
  }
  if (resolution[0] <= 0 || resolution[1] <= 0)
  {
    fprintf(stderr, "Horizontal and vertical resolutions must be positive %d , %d\n", resolution[0], resolution[1]);
    return std::nullopt;
    ;
  }

  // horizontal field of view
  std::vector<double> azimuth_range;
  std::optional<std::string> azimuth_range_str = get_parameter("azimuth_range");
  if (!azimuth_range_str.has_value())
  {
    fprintf(stderr, "Azimuth Range must be specified\n");
    return std::nullopt;
  }
  ReadVector(azimuth_range, azimuth_range_str.value());
  if (azimuth_range.size() != 2)
  {
    fprintf(stderr, "Both minimum and maximum azimuth angles must be specified");
    return std::nullopt;
  }
  if (azimuth_range[0] < -M_PI)
  {
    fprintf(stderr, "`azimuth_range minimum` must be greater than or equal to -pi");
    return std::nullopt;
  }
  if (azimuth_range[1] > M_PI)
  {
    fprintf(stderr, "`azimuth_range maximum` must be less than or equal to pi");
    return std::nullopt;
  }
  if (azimuth_range[0] >= azimuth_range[1])
  {
    fprintf(stderr, "`azimuth_range minimum` must less than 'azimuth_range maximum");
    return std::nullopt;
  }


  // vertical field of view
  std::vector<double> elevation_range;
  std::optional<std::string> elevation_range_str = get_parameter("elevation_range");
  if (elevation_range_str.has_value())
  {
    ReadVector(elevation_range, elevation_range_str.value().c_str());
    if (elevation_range.size() != 2)
    {
      if (elevation_range.size() == 0)
      {
        fprintf(stderr, "Mininimum elevation angle must be specified");
        return std::nullopt;
      }
      if (elevation_range.size() == 1)
      {
        if (resolution[1] > 1)
        {
          fprintf(stderr, "When elevation resolution is greater than 1, maximum "
                          "elevation must be specified\n");
          return std::nullopt;
        }
        else
        {
          elevation_range[1] = elevation_range[0];
        }
      }
    }
    if (elevation_range[0] < -M_PI)
    {
      fprintf(stderr, "`elevation_range minimum` must be greater than or equal to -pi\n");
      return std::nullopt;
    }
    if (elevation_range[1] > M_PI)
    {
      fprintf(stderr, "`elevation_range maximum` must be less than or equal to pi\n");
      return std::nullopt;
    }
    if ((resolution[1] > 1) && (elevation_range[0] >= elevation_range[1]))
    {
      fprintf(stderr, "`elevation_range minimum` must less than 'elevation_range maximum\n");
      return std::nullopt;
    }
  }
  else if (resolution[1] <= 1)
  {
    elevation_range = { 0.0, 0.0 };
  }
  else
  {
    if (resolution[1] > 1)
    {
      fprintf(stderr, "When elevation resolution is greater than 1, elevation range must be specified\n");
      return std::nullopt;
    }
  }

  std::optional<std::string> max_range_str = get_parameter("range_max");
  std::optional<std::string> min_range_str = get_parameter("range_min");
  double min_range = min_range_str.has_value() ? std::atof(min_range_str.value().c_str()) : 0.0;
  double max_range = max_range_str.has_value() ? std::atof(max_range_str.value().c_str()) : 1000.0;

  if (max_range <= 0.0)
  {
    fprintf(stderr, "Lidar max range must be greater than 0.0\n");
    return std::nullopt;
  }
  if (min_range < 0.0)
  {
    fprintf(stderr, "Lidar min range must be greater or equal to 0.0\n");
    return std::nullopt;
  }

  if (max_range <= min_range)
  {
    fprintf(stderr, "Lidar max range must be less than min range\n");
    return std::nullopt;
  }

  if (!frame_name.has_value())
  {
    fprintf(stderr, "Frame name must be specified\n");
    return std::nullopt;
  }

  // Otherwise construct and return a new LidarData object
  Lidar3dConfig lidar_sensor;
  lidar_sensor.frame_name = frame_name.value();
  lidar_sensor.azimuth_range = azimuth_range;
  lidar_sensor.elevation_range = elevation_range;
  lidar_sensor.resolution = { resolution[0], resolution[1] };
  lidar_sensor.range_min = min_range;
  lidar_sensor.range_max = max_range;

  if (lidar_sensor.resolution[1] <= 1)
  {
    lidar_sensor.laserscan_topic = laserscan_topic.has_value() ? laserscan_topic.value() : "/scan";

    float angle_increment = static_cast<float>(lidar_sensor.azimuth_range[1] - lidar_sensor.azimuth_range[0]) /
                            static_cast<float>(lidar_sensor.resolution[0] - 1);

    // Configure the static parameters of the laserscan message
    lidar_sensor.laser_scan_msg.header.frame_id = lidar_sensor.frame_name;
    lidar_sensor.laser_scan_msg.time_increment = 0.0;  // Does this matter?
    lidar_sensor.laser_scan_msg.angle_min = static_cast<float>(lidar_sensor.azimuth_range[0]);
    lidar_sensor.laser_scan_msg.angle_max = static_cast<float>(lidar_sensor.azimuth_range[1]);
    lidar_sensor.laser_scan_msg.angle_increment = angle_increment;
    lidar_sensor.laser_scan_msg.range_min = static_cast<float>(lidar_sensor.range_min);
    lidar_sensor.laser_scan_msg.range_max = static_cast<float>(lidar_sensor.range_max);
    lidar_sensor.laser_scan_msg.ranges.resize(lidar_sensor.resolution[0] * lidar_sensor.resolution[1]);
    lidar_sensor.laser_scan_msg.intensities.resize(0);
  }

  return lidar_sensor;
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
    fprintf(stderr, "********************* %s : %d %d\n", __FUNCTION__, __LINE__, mj_model_->sensor_type[i]);
    // Skip sensors not controlled by plugin.
    if (mj_model_->sensor_type[i] != mjtSensor::mjSENS_PLUGIN)
    {
      continue;
    }
    fprintf(stderr, "********************* %s : %d \n", __FUNCTION__, __LINE__);
    // Grab the name of the sensor, which is required.
    const auto sensor_name_maybe = mj_id2name(mj_model_, mjtObj::mjOBJ_SENSOR, i);

    if (sensor_name_maybe == nullptr)
    {
      RCLCPP_WARN(node_->get_logger(), "Cannot find a name for lidar sensor at index: '%d' skipping!", i);
      continue;
    }

    fprintf(stderr, "********************* %s : %d \n", __FUNCTION__, __LINE__);
    RCLCPP_INFO(node_->get_logger(), "Adding lidar sensor: '%s', idx: %d", sensor_name_maybe, i);

    std::optional<Lidar3dConfig> new_data_maybe = get_lidar_config(hardware_info, sensor_name_maybe);
    if (!new_data_maybe.has_value())
    {
      RCLCPP_ERROR(node_->get_logger(), "Failed to parse required configuration from ros2_control xacro: '%s'",
                   sensor_name_maybe);
      return false;
    }

    fprintf(stderr, "********************* %s : %d \n", __FUNCTION__, __LINE__);
    Lidar3dConfig lidar_config = new_data_maybe.value();

    // Setup remaining msg params and publisher for the sensor
    lidar_config.name = sensor_name_maybe;
    lidar_config.sensor_id = i;
    lidar_config.scan_pub = node_->create_publisher<sensor_msgs::msg::LaserScan>(lidar_config.laserscan_topic, 1);

    // We may someday want to compute this on the fly, but since everything is fixed this should be fine for now.
    lidar_config.laser_scan_msg.scan_time = 1.0f / static_cast<float>(lidar_publish_rate_);

    // Note that we have added the sensor
    RCLCPP_INFO(node_->get_logger(), "Adding lidar sensor: '%s', idx: %d", lidar_config.name.c_str(),
                lidar_config.sensor_id);
    RCLCPP_INFO(node_->get_logger(), "         frame_name: '%s'", lidar_config.frame_name.c_str());
    RCLCPP_INFO(node_->get_logger(), "      azimuth_range: %.4f - %.4f", lidar_config.azimuth_range[0],
                lidar_config.azimuth_range[1]);
    RCLCPP_INFO(node_->get_logger(), "    elevation_range: %.4f - %.4f", lidar_config.elevation_range[0],
                lidar_config.elevation_range[1]);
    RCLCPP_INFO(node_->get_logger(), "         resolution: %d - %d", lidar_config.resolution[0],
                lidar_config.resolution[1]);
    RCLCPP_INFO(node_->get_logger(), "          range_min: %f", lidar_config.range_min);
    RCLCPP_INFO(node_->get_logger(), "          range_max: %f", lidar_config.range_max);
  
    lidar_sensors_.push_back(lidar_config);
  }

  return true;
}

void Mujoco3dLidar::init()
{
  // Start the rendering thread process
  publish_lidar_ = true;
  rendering_thread_ = std::thread(&Mujoco3dLidar::update_loop, this);
}

void Mujoco3dLidar::close()
{
  publish_lidar_ = false;
  if (rendering_thread_.joinable())
  {
    rendering_thread_.join();
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
  }
  for(Lidar3dConfig& lidar : lidar_sensors_)
  {
    for (int i = 0; i < lidar.resolution[0] * lidar.resolution[1]; ++i)
    {
      lidar.laser_scan_msg.ranges[i] = static_cast<float>(mj_data_->sensordata[lidar.sensor_id + i]);
    }
/*
    // Apply range limits to the copied data
    std::transform(lidar.laser_scan_msg.ranges.begin(), lidar.laser_scan_msg.ranges.end(),
                  lidar.laser_scan_msg.ranges.begin(), [&](auto range) {
                    return (range < lidar.range_min || range > lidar.range_max) ? -1.0 : range;
                  });
*/
    lidar.laser_scan_msg.header.stamp = node_->now();
    lidar.scan_pub->publish(lidar.laser_scan_msg);
  }
}

}  // namespace mujoco_ros2_control
