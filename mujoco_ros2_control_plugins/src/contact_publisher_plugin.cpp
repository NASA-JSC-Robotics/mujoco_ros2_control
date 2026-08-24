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

#include "contact_publisher_plugin.hpp"

#include <algorithm>
#include <string>

#include <pluginlib/class_list_macros.hpp>

namespace mujoco_ros2_control_plugins
{
bool ContactPublisherPlugin::resolve_participating_bodies(const std::vector<std::string>& body_names)
{
  body_names_by_id_.resize(static_cast<std::size_t>(model_->nbody));
  for (int body_id = 0; body_id < model_->nbody; ++body_id)
  {
    const char* name = mj_id2name(model_, mjOBJ_BODY, body_id);
    body_names_by_id_[static_cast<std::size_t>(body_id)] = name ? name : "";
  }

  all_bodies_participate_ = body_names.empty();
  participating_bodies_.assign(static_cast<std::size_t>(model_->nbody), 0);
  if (all_bodies_participate_)
  {
    return true;
  }

  for (const auto& requested_name : body_names)
  {
    const int body_id = mj_name2id(model_, mjOBJ_BODY, requested_name.c_str());
    if (body_id == -1)
    {
      RCLCPP_ERROR(logger_, "body_names entry '%s' is not a body in this MuJoCo model.", requested_name.c_str());
      return false;
    }
    participating_bodies_[static_cast<std::size_t>(body_id)] = 1;
  }

  // A contact needs both of its bodies listed, so a single-entry list can never match anything.
  // That is always a configuration mistake rather than a deliberate "publish nothing".
  const auto listed = std::count(participating_bodies_.begin(), participating_bodies_.end(), 1);
  if (listed < 2)
  {
    RCLCPP_WARN(logger_,
                "body_names lists a single body ('%s'); a contact is only published when both of its bodies "
                "are listed, so no contact will ever match.",
                body_names.front().c_str());
  }
  return true;
}

bool ContactPublisherPlugin::init(rclcpp::Node::SharedPtr node, const mjModel* model, mjData* /*data*/)
{
  node_ = node;
  model_ = model;
  logger_ = node_->get_logger().get_child(node->get_sub_namespace());

  if (!node_->has_parameter("body_names"))
  {
    node_->declare_parameter("body_names", std::vector<std::string>{});
  }
  if (!node_->has_parameter("topic"))
  {
    node_->declare_parameter("topic", std::string("contact_pairs"));
  }
  if (!node_->has_parameter("publish_rate"))
  {
    node_->declare_parameter("publish_rate", 50.0);
  }

  const std::vector<std::string> body_names = node_->get_parameter("body_names").as_string_array();
  const std::string topic = node_->get_parameter("topic").as_string();
  const double publish_rate = node_->get_parameter("publish_rate").as_double();

  if (publish_rate <= 0.0)
  {
    RCLCPP_ERROR(logger_, "publish_rate must be > 0, got %f.", publish_rate);
    return false;
  }

  if (!resolve_participating_bodies(body_names))
  {
    return false;
  }

  publish_period_ = rclcpp::Duration::from_seconds(1.0 / publish_rate);
  last_publish_time_ = node_->get_clock()->now();

  publisher_raw_ = node_->create_publisher<ContactPairArray>(topic, rclcpp::SystemDefaultsQoS());
  publisher_ = std::make_unique<realtime_tools::RealtimePublisher<ContactPairArray>>(publisher_raw_);

  const std::string participants =
      all_bodies_participate_ ? std::string("any two bodies") : std::to_string(body_names.size()) + " listed bodies";
  RCLCPP_INFO(logger_,
              "ContactPublisherPlugin initialized: publishing to '%s' at %.1f Hz, reporting contacts "
              "between %s.",
              publisher_raw_->get_topic_name(), publish_rate, participants.c_str());

  return true;
}

void ContactPublisherPlugin::update(const mjModel* /*model*/, mjData* data)
{
  const rclcpp::Time now = node_->get_clock()->now();
  if (now - last_publish_time_ < publish_period_)
  {
    return;
  }
  last_publish_time_ = now;

  // Contacts live in mjData's arena in MuJoCo 3.x and mjModel::nconmax is a legacy field left at
  // -1, so there is no static bound to reserve from at init. data->ncon is the exact one: a pair
  // needs at least one contact, so it can never be exceeded. Both calls are a no-op compare once
  // the contact set has peaked, and clear() keeps the capacity between cycles.
  aggregates_.clear();
  aggregates_.reserve(static_cast<std::size_t>(data->ncon));
  message_.contact_pairs.reserve(static_cast<std::size_t>(data->ncon));

  for (int i = 0; i < data->ncon; ++i)
  {
    const mjContact& contact = data->contact[i];

    // Flex participants carry no geom, and therefore no owning body that could be named.
    if (contact.geom[0] < 0 || contact.geom[1] < 0)
    {
      continue;
    }

    // Order the pair by body id so that a-vs-b and b-vs-a aggregate into the same entry.
    const int body_a = model_->geom_bodyid[contact.geom[0]];
    const int body_b = model_->geom_bodyid[contact.geom[1]];
    const int first = std::min(body_a, body_b);
    const int second = std::max(body_a, body_b);

    // Both ends must be listed: a body left out of body_names is free to collide with anything.
    if (!is_participating(first) || !is_participating(second))
    {
      continue;
    }

    const auto it = std::find_if(aggregates_.begin(), aggregates_.end(), [first, second](const PairAggregate& entry) {
      return entry.body_ids[0] == first && entry.body_ids[1] == second;
    });
    if (it == aggregates_.end())
    {
      PairAggregate entry;
      entry.body_ids[0] = first;
      entry.body_ids[1] = second;
      entry.num_contacts = 1;
      entry.min_distance = contact.dist;
      aggregates_.push_back(entry);
    }
    else
    {
      ++it->num_contacts;
      it->min_distance = std::min(it->min_distance, contact.dist);
    }
  }

  message_.header.stamp = now;
  message_.contact_pairs.resize(aggregates_.size());
  for (std::size_t i = 0; i < aggregates_.size(); ++i)
  {
    const PairAggregate& entry = aggregates_[i];
    auto& out = message_.contact_pairs[i];
    for (std::size_t side = 0; side < 2; ++side)
    {
      out.body_ids[side] = entry.body_ids[side];
      out.body_names[side] = body_names_by_id_[static_cast<std::size_t>(entry.body_ids[side])];
    }
    out.num_contacts = entry.num_contacts;
    out.min_distance = entry.min_distance;
  }

#if REALTIME_TOOLS_VERSION_MAJOR > 2
  publisher_->try_publish(message_);
#else
  publisher_->tryPublish(message_);
#endif
}

void ContactPublisherPlugin::cleanup()
{
  RCLCPP_INFO(logger_, "ContactPublisherPlugin cleanup.");
  publisher_.reset();
  publisher_raw_.reset();
  node_.reset();
}

}  // namespace mujoco_ros2_control_plugins

// Export the plugin
PLUGINLIB_EXPORT_CLASS(mujoco_ros2_control_plugins::ContactPublisherPlugin,
                       mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase)
