// Copyright 2026 PAL Robotics S.L.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MUJOCO_ROS2_CONTROL_PLUGINS__CONTACT_PUBLISHER_PLUGIN_HPP_
#define MUJOCO_ROS2_CONTROL_PLUGINS__CONTACT_PUBLISHER_PLUGIN_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <mujoco_ros2_control_msgs/msg/contact_pair_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_publisher.hpp>

#include "mujoco_ros2_control_plugins/mujoco_ros2_control_plugins_base.hpp"

namespace mujoco_ros2_control_plugins
{

/**
 * @brief Plugin that publishes which MuJoCo bodies are currently colliding.
 *
 * Publishes mujoco_ros2_control_msgs/msg/ContactPairArray on the configurable ``topic``
 * parameter. Every MuJoCo contact between the same two bodies is aggregated into one entry, so
 * a box resting on the floor on four corners is reported as a single body pair with
 * ``num_contacts == 4``.
 *
 * Parameters
 * ----------
 *   body_names   - The bodies that participate in collision reporting. A contact is published
 *                  only when *both* of its bodies are listed, so any body left out of the list
 *                  is free to collide with anything. Empty (the default) means every body in
 *                  the model participates.
 *   topic        - Output topic name. Defaults to "contact_pairs".
 *   publish_rate - Publish frequency in Hz. Defaults to 50.0.
 *
 * Listing the robot links plus the parts of the environment that matter (walls, fixtures) is
 * therefore enough to monitor self-collisions and environment collisions, while props the robot
 * is meant to touch are handled by simply not listing them.
 *
 * Contacts whose participant is a flex body carry no geom, and therefore no owning body to name,
 * so they are skipped.
 */
class ContactPublisherPlugin : public MuJoCoROS2ControlPluginBase
{
public:
  ContactPublisherPlugin() = default;
  ~ContactPublisherPlugin() override = default;

  bool init(rclcpp::Node::SharedPtr node, const mjModel* model, mjData* data) override;
  void update(const mjModel* model, mjData* data) override;
  void cleanup() override;

private:
  using ContactPairArray = mujoco_ros2_control_msgs::msg::ContactPairArray;

  /// All contacts between one body pair, accumulated over a single update().
  struct PairAggregate
  {
    int body_ids[2]{ -1, -1 };
    int num_contacts{ 0 };
    mjtNum min_distance{ 0.0 };
  };

  /// Caches every body name and marks the bodies listed in `body_names`. Returns false (logging
  /// an error) if a requested body name is not in the model.
  bool resolve_participating_bodies(const std::vector<std::string>& body_names);

  /// True if `body_id` takes part in collision reporting. Both bodies of a contact must, or the
  /// contact is not published.
  bool is_participating(int body_id) const
  {
    return all_bodies_participate_ || participating_bodies_[static_cast<std::size_t>(body_id)] != 0;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_{ rclcpp::get_logger("ContactPublisherPlugin") };

  const mjModel* model_{ nullptr };

  rclcpp::Publisher<ContactPairArray>::SharedPtr publisher_raw_;
  std::unique_ptr<realtime_tools::RealtimePublisher<ContactPairArray>> publisher_;

  /// Body names indexed by body id, resolved once in init() so update() never touches the
  /// model's name table.
  std::vector<std::string> body_names_by_id_;

  /// Per-body-id participation flag; only meaningful when all_bodies_participate_ is false.
  std::vector<uint8_t> participating_bodies_;
  bool all_bodies_participate_{ true };

  rclcpp::Duration publish_period_{ 0, 0 };
  rclcpp::Time last_publish_time_{ 0, 0, RCL_ROS_TIME };

  /// Reused across update() calls so a steady-state contact set allocates nothing.
  std::vector<PairAggregate> aggregates_;
  ContactPairArray message_;
};

}  // namespace mujoco_ros2_control_plugins

#endif  // MUJOCO_ROS2_CONTROL_PLUGINS__CONTACT_PUBLISHER_PLUGIN_HPP_
