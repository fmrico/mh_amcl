// Copyright 2022 Intelligent Robotics Lab
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


#include "gtest/gtest.h"

#include "mh_amcl/MapMatcher.hpp"

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class MapMatcherTest : public mh_amcl::MapMatcher
{
public:
  MapMatcherTest(const nav_msgs::msg::OccupancyGrid & map, rclcpp::Node::SharedPtr node)
  : node_(node), MapMatcher(map)
  {
    pubs_.resize(NUM_LEVEL_SCALE_COSTMAP);
    for (int i = 0; i < NUM_LEVEL_SCALE_COSTMAP; i++) {
      pubs_[i] = node_->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "grid_" + std::to_string(i), 100);
    }
  }

  void publish_maps()
  {
    for (int i = 0; i < NUM_LEVEL_SCALE_COSTMAP; i++) {
      if (pubs_[i]->get_subscription_count() > 0) {
        nav_msgs::msg::OccupancyGrid grid = mh_amcl::toMsg(*costmaps_[i]);
        grid.header.frame_id = "map";
        grid.header.stamp = node_->now();
        pubs_[i]->publish(grid);
      }
    }
  }

  rclcpp::Node::SharedPtr node_;
  std::vector<rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr> pubs_;
};

TEST(test1, test_match)
{
  auto test_node = rclcpp::Node::make_shared("test_node");
  sensor_msgs::msg::LaserScan scan;
  nav_msgs::msg::OccupancyGrid grid;

  auto scan_sub = test_node->create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", 100, [&scan] (const sensor_msgs::msg::LaserScan::SharedPtr msg) {
      scan = *msg;
    });
  auto grid_sub = test_node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    [&grid] (const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
      grid = *msg;
    });  

  auto start = test_node->now();
  rclcpp::Rate rate(100ms);
  while ((test_node->now() - start).seconds() < 5.0) {
    rclcpp::spin_some(test_node);
    rate.sleep();
  }

  ASSERT_FALSE(scan.ranges.empty());
  ASSERT_FALSE(grid.data.empty());

  MapMatcherTest map_matcher(grid, test_node);
  const auto & tfs = map_matcher.get_matchs(scan);

  for (const auto & transform : tfs) {
    const auto & x = transform.transform.getOrigin().x();
    const auto & y = transform.transform.getOrigin().y();
    double roll, pitch, yaw;
    tf2::Matrix3x3(transform.transform.getRotation()).getRPY(roll, pitch, yaw);
     std::cerr << "(" << x << ", " << y << ", " << yaw << ")" << std::endl;
  }
  // start = test_node->now();
  // while ((test_node->now() - start).seconds() < 30.0) {
  //   map_matcher.publish_maps();
  //   rclcpp::spin_some(test_node);
  //   rate.sleep();
  // }

}

int main(int argc, char * argv[])
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
  return 0;
}
