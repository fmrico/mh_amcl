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




TEST(test1, test_match)
{
  auto test_node = rclcpp::Node::make_shared("test_node");
  mh_amcl::MapMatcher map_matcher;

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
  while ((test_node->now() - start).seconds() < 5.0) {
    rclcpp::spin_some(test_node);
  }

  ASSERT_FALSE(scan.ranges.empty());
  ASSERT_FALSE(grid.data.empty());

  auto scan_pc = map_matcher.PCfromScan(scan);
  auto grid_pc = map_matcher.PCfromGrid(grid);

  mh_amcl::state2d init;
  init.t(0) = -2.0;
  init.t(1) = -0.5;
  init.theta = 0.0;

  mh_amcl::state2d st = map_matcher.pc_match(scan_pc, grid_pc, init);
}

int main(int argc, char * argv[])
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
  return 0;
}
