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

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <map>
#include <set>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <mujoco/mujoco.h>
#include <mujoco_ros2_control_msgs/msg/contact_pair_array.hpp>
#include <rclcpp/rclcpp.hpp>

#include "contact_publisher_plugin.hpp"

namespace
{
// Two boxes overlapping each other, both sunk into the ground plane (which belongs to the world
// body), plus a sphere floating well clear of everything. In the initial configuration the
// colliding body pairs are therefore (world, box_a), (world, box_b) and (box_a, box_b), each of
// them backed by several MuJoCo contact points.
constexpr const char* kMjcf = R"(
<mujoco model="contact_publisher_test">
  <worldbody>
    <geom name="ground" type="plane" size="5 5 0.1"/>
    <body name="box_a" pos="0 0 0.05">
      <freejoint/>
      <geom name="box_a_geom" type="box" size="0.1 0.1 0.1"/>
    </body>
    <body name="box_b" pos="0.15 0 0.05">
      <freejoint/>
      <geom name="box_b_geom" type="box" size="0.1 0.1 0.1"/>
    </body>
    <body name="floater" pos="0 0 5">
      <freejoint/>
      <geom name="floater_geom" type="sphere" size="0.1"/>
    </body>
  </worldbody>
</mujoco>
)";

using ContactPairArray = mujoco_ros2_control_msgs::msg::ContactPairArray;

/// The published pairs as a comparable set of (first name, second name).
std::set<std::pair<std::string, std::string>> pairNames(const ContactPairArray& msg)
{
  std::set<std::pair<std::string, std::string>> pairs;
  for (const auto& entry : msg.contact_pairs)
  {
    pairs.emplace(entry.body_names[0], entry.body_names[1]);
  }
  return pairs;
}
}  // namespace

class ContactPublisherPluginTest : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    if (!rclcpp::ok())
    {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestCase()
  {
    if (rclcpp::ok())
    {
      rclcpp::shutdown();
    }
  }

  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("contact_publisher_test_node");
    plugin_node_ = node_->create_sub_node("contact_publisher_plugin");

    executor_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(node_);
    spin_thread_ = std::thread([this]() { executor_->spin(); });

    char error[1024] = { 0 };
    mjSpec* spec = mj_parseXMLString(kMjcf, nullptr, error, sizeof(error));
    ASSERT_NE(spec, nullptr) << error;

    model_ = mj_compile(spec, nullptr);
    if (model_ == nullptr)
    {
      const char* ce = mjs_getError(spec);
      mj_deleteSpec(spec);
      FAIL() << (ce ? ce : "mj_compile failed");
    }
    mj_deleteSpec(spec);

    data_ = mj_makeData(model_);
    ASSERT_NE(data_, nullptr);
    // Collision detection runs as part of the forward dynamics, so this fills data_->contact
    // for the model's initial configuration.
    mj_forward(model_, data_);
    ASSERT_GT(data_->ncon, 0) << "Test model is expected to start out in collision";
  }

  void TearDown() override
  {
    executor_->cancel();
    if (spin_thread_.joinable())
    {
      spin_thread_.join();
    }
    executor_.reset();
    plugin_node_.reset();
    node_.reset();
    mj_deleteData(data_);
    data_ = nullptr;
    mj_deleteModel(model_);
    model_ = nullptr;
  }

  /// Replaces the fixture's model with one compiled from `mjcf`, for tests that need a scene
  /// other than the small default one.
  void loadModel(const std::string& mjcf)
  {
    mj_deleteData(data_);
    mj_deleteModel(model_);
    char error[1024] = { 0 };
    mjSpec* spec = mj_parseXMLString(mjcf.c_str(), nullptr, error, sizeof(error));
    ASSERT_NE(spec, nullptr) << error;
    model_ = mj_compile(spec, nullptr);
    mj_deleteSpec(spec);
    ASSERT_NE(model_, nullptr);
    data_ = mj_makeData(model_);
    mj_forward(model_, data_);
  }

  /// Moves the body driven by a free joint to `pos`, e.g. to take it out of collision.
  void setFreeJointPosition(const std::string& body_name, const mjtNum pos[3])
  {
    const int body_id = mj_name2id(model_, mjOBJ_BODY, body_name.c_str());
    ASSERT_GE(body_id, 0) << body_name;
    for (int i = 0; i < model_->njnt; ++i)
    {
      if (model_->jnt_bodyid[i] == body_id && model_->jnt_type[i] == mjJNT_FREE)
      {
        const int qadr = model_->jnt_qposadr[i];
        data_->qpos[qadr + 0] = pos[0];
        data_->qpos[qadr + 1] = pos[1];
        data_->qpos[qadr + 2] = pos[2];
        return;
      }
    }
    FAIL() << body_name << " is not driven by a free joint";
  }

  /// Pre-declares and sets a parameter on plugin_node_ before init(), matching how launch
  /// parameter files set plugin parameters ahead of load_mujoco_plugins().
  template <typename T>
  void setParam(const std::string& name, const T& value)
  {
    if (!plugin_node_->has_parameter(name))
    {
      plugin_node_->declare_parameter(name, rclcpp::ParameterValue(value));
    }
    plugin_node_->set_parameter(rclcpp::Parameter(name, value));
  }

  /// Subscribes to `topic` on plugin_node_ (matching the publisher's node) and spins until a
  /// message arrives or the timeout elapses. Returns nullptr on timeout.
  ContactPairArray::SharedPtr waitForMessage(const std::string& topic,
                                             mujoco_ros2_control_plugins::ContactPublisherPlugin& plugin)
  {
    std::atomic<bool> received{ false };
    ContactPairArray::SharedPtr last_msg;
    auto sub = plugin_node_->create_subscription<ContactPairArray>(topic, 10, [&](ContactPairArray::SharedPtr msg) {
      last_msg = msg;
      received = true;
    });

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (!received && std::chrono::steady_clock::now() < deadline)
    {
      plugin.update(model_, data_);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return last_msg;
  }

  mjModel* model_{ nullptr };
  mjData* data_{ nullptr };
  rclcpp::Node::SharedPtr node_;
  rclcpp::Node::SharedPtr plugin_node_;

private:
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spin_thread_;
};

TEST_F(ContactPublisherPluginTest, InitSucceeds)
{
  mujoco_ros2_control_plugins::ContactPublisherPlugin plugin;
  EXPECT_TRUE(plugin.init(plugin_node_, model_, data_));
  plugin.cleanup();
}

TEST_F(ContactPublisherPluginTest, PublishesEveryCollidingBodyPairByDefault)
{
  mujoco_ros2_control_plugins::ContactPublisherPlugin plugin;
  ASSERT_TRUE(plugin.init(plugin_node_, model_, data_));

  auto msg = waitForMessage("contact_pairs", plugin);
  ASSERT_NE(msg, nullptr) << "No ContactPairArray received";

  const std::set<std::pair<std::string, std::string>> expected{ { "box_a", "box_b" },
                                                                { "world", "box_a" },
                                                                { "world", "box_b" } };
  EXPECT_EQ(pairNames(*msg), expected);

  // Body ids and names must describe the same bodies, and the pair must be ordered by body id.
  for (const auto& entry : msg->contact_pairs)
  {
    EXPECT_LT(entry.body_ids[0], entry.body_ids[1]);
    for (std::size_t side = 0; side < 2; ++side)
    {
      const char* name = mj_id2name(model_, mjOBJ_BODY, entry.body_ids[side]);
      EXPECT_EQ(entry.body_names[side], name ? name : "");
    }
  }

  plugin.cleanup();
}

TEST_F(ContactPublisherPluginTest, AggregatesAllContactsOfAPairIntoOneEntry)
{
  mujoco_ros2_control_plugins::ContactPublisherPlugin plugin;
  ASSERT_TRUE(plugin.init(plugin_node_, model_, data_));

  auto msg = waitForMessage("contact_pairs", plugin);
  ASSERT_NE(msg, nullptr);

  // Box-on-plane alone yields several contact points, so aggregating must collapse more MuJoCo
  // contacts than there are published pairs, while accounting for every one of them.
  ASSERT_GT(data_->ncon, static_cast<int>(msg->contact_pairs.size()));
  int total_contacts = 0;
  for (const auto& entry : msg->contact_pairs)
  {
    EXPECT_GT(entry.num_contacts, 0);
    // Every pair in this model genuinely overlaps, so the deepest contact must be penetrating.
    EXPECT_LT(entry.min_distance, 0.0);
    total_contacts += entry.num_contacts;
  }
  EXPECT_EQ(total_contacts, data_->ncon);

  plugin.cleanup();
}

TEST_F(ContactPublisherPluginTest, BothBodiesMustBeListedForAPairToBePublished)
{
  setParam("body_names", std::vector<std::string>{ "world", "box_a" });

  mujoco_ros2_control_plugins::ContactPublisherPlugin plugin;
  ASSERT_TRUE(plugin.init(plugin_node_, model_, data_));

  auto msg = waitForMessage("contact_pairs", plugin);
  ASSERT_NE(msg, nullptr);

  // box_b is not listed, so both of its contacts are dropped even though their other end
  // (world, box_a) is listed.
  const std::set<std::pair<std::string, std::string>> expected{ { "world", "box_a" } };
  EXPECT_EQ(pairNames(*msg), expected);

  plugin.cleanup();
}

TEST_F(ContactPublisherPluginTest, OmittedBodiesAreFreeToCollideWithAnything)
{
  // The canonical setup: list the bodies that matter, omit the ground they are allowed to rest
  // on. Only the box-on-box contact survives.
  setParam("body_names", std::vector<std::string>{ "box_a", "box_b" });

  mujoco_ros2_control_plugins::ContactPublisherPlugin plugin;
  ASSERT_TRUE(plugin.init(plugin_node_, model_, data_));

  auto msg = waitForMessage("contact_pairs", plugin);
  ASSERT_NE(msg, nullptr);

  const std::set<std::pair<std::string, std::string>> expected{ { "box_a", "box_b" } };
  EXPECT_EQ(pairNames(*msg), expected);

  plugin.cleanup();
}

TEST_F(ContactPublisherPluginTest, SingleListedBodyCanNeverMatchAPair)
{
  // Both ends must be listed, so one entry matches nothing. The plugin warns about this at
  // init, but still comes up and publishes empty arrays.
  setParam("body_names", std::vector<std::string>{ "box_a" });

  mujoco_ros2_control_plugins::ContactPublisherPlugin plugin;
  ASSERT_TRUE(plugin.init(plugin_node_, model_, data_));

  auto msg = waitForMessage("contact_pairs", plugin);
  ASSERT_NE(msg, nullptr);
  EXPECT_TRUE(msg->contact_pairs.empty());

  plugin.cleanup();
}

TEST_F(ContactPublisherPluginTest, PublishesEmptyArrayWhenParticipatingBodiesAreClear)
{
  const mjtNum airborne[3] = { 0.0, 0.0, 10.0 };
  setFreeJointPosition("box_b", airborne);
  mj_forward(model_, data_);
  ASSERT_GT(data_->ncon, 0) << "box_a should still be colliding with the ground";

  setParam("body_names", std::vector<std::string>{ "box_b", "floater" });

  mujoco_ros2_control_plugins::ContactPublisherPlugin plugin;
  ASSERT_TRUE(plugin.init(plugin_node_, model_, data_));

  auto msg = waitForMessage("contact_pairs", plugin);
  ASSERT_NE(msg, nullptr) << "A collision-free state must still publish, with an empty array";
  EXPECT_TRUE(msg->contact_pairs.empty());

  plugin.cleanup();
}

TEST_F(ContactPublisherPluginTest, AggregatesManyPairsConsistentlyAcrossCycles)
{
  // A grid of overlapping boxes produces hundreds of contacts over dozens of body pairs, which
  // is where aggregating them is easiest to get wrong.
  std::string mjcf = "<mujoco><worldbody><geom name=\"ground\" type=\"plane\" size=\"9 9 .1\"/>";
  constexpr int kBoxes = 36;
  for (int i = 0; i < kBoxes; ++i)
  {
    mjcf += "<body name=\"box_" + std::to_string(i) + "\" pos=\"" + std::to_string((i % 6) * 0.15) + " " +
            std::to_string((i / 6) * 0.15) + " 0.05\"><freejoint/>" + "<geom type=\"box\" size=\".1 .1 .1\"/></body>";
  }
  mjcf += "</worldbody></mujoco>";
  loadModel(mjcf);
  ASSERT_GT(data_->ncon, 100) << "Scene should be contact-rich enough to stress the lookup";

  // Brute-force the expected aggregation straight from mjData, independent of the plugin.
  std::map<std::pair<int, int>, int> expected;
  for (int i = 0; i < data_->ncon; ++i)
  {
    const mjContact& c = data_->contact[i];
    if (c.geom[0] < 0 || c.geom[1] < 0)
      continue;
    const int a = model_->geom_bodyid[c.geom[0]];
    const int b = model_->geom_bodyid[c.geom[1]];
    ++expected[{ std::min(a, b), std::max(a, b) }];
  }
  ASSERT_GT(expected.size(), 20u) << "Scene should span many distinct body pairs";

  mujoco_ros2_control_plugins::ContactPublisherPlugin plugin;
  ASSERT_TRUE(plugin.init(plugin_node_, model_, data_));

  // Two publishes: anything left behind by the first cycle would show up as a wrong count in
  // the second.
  for (int cycle = 0; cycle < 2; ++cycle)
  {
    auto msg = waitForMessage("contact_pairs", plugin);
    ASSERT_NE(msg, nullptr) << "cycle " << cycle;

    std::map<std::pair<int, int>, int> got;
    for (const auto& entry : msg->contact_pairs)
    {
      EXPECT_LT(entry.body_ids[0], entry.body_ids[1]);
      got[{ entry.body_ids[0], entry.body_ids[1] }] = entry.num_contacts;
    }
    EXPECT_EQ(got, expected) << "cycle " << cycle;

    int total = 0;
    for (const auto& entry : msg->contact_pairs)
      total += entry.num_contacts;
    EXPECT_EQ(total, data_->ncon) << "every contact must land in exactly one pair, cycle " << cycle;
  }

  plugin.cleanup();
}

TEST_F(ContactPublisherPluginTest, InitRejectsUnknownBodyNameInFilter)
{
  setParam("body_names", std::vector<std::string>{ "nonexistent_body" });

  mujoco_ros2_control_plugins::ContactPublisherPlugin plugin;
  EXPECT_FALSE(plugin.init(plugin_node_, model_, data_));
}

TEST_F(ContactPublisherPluginTest, InitRejectsNonPositivePublishRate)
{
  setParam("publish_rate", 0.0);

  mujoco_ros2_control_plugins::ContactPublisherPlugin plugin;
  EXPECT_FALSE(plugin.init(plugin_node_, model_, data_));
}
