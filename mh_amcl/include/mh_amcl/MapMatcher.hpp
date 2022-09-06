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

#ifndef MH_AMCL__MAPMATCHER_HPP_
#define MH_AMCL__MAPMATCHER_HPP_

#include <list>
#include <memory>
#include <vector>

#include "tf2/transform_datatypes.h"
#include "tf2/LinearMath/Transform.h"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "nav2_costmap_2d/costmap_2d.hpp"

#include "rclcpp/rclcpp.hpp"


namespace mh_amcl
{

typedef struct
{
  float weight;
  tf2::Transform transform;
} TransformWeighted;

bool operator<(const TransformWeighted & tw1, const TransformWeighted & tw2);

class MapMatcher
{
public:
  explicit MapMatcher(const nav_msgs::msg::OccupancyGrid & map);
  std::list<TransformWeighted> get_matchs(const sensor_msgs::msg::LaserScan & scan);

protected:
  static const int NUM_LEVEL_SCALE_COSTMAP = 4;

  std::shared_ptr<nav2_costmap_2d::Costmap2D>
  half_scale(std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap_in);
  std::vector<tf2::Vector3> laser2points(const sensor_msgs::msg::LaserScan & scan);
  std::list<TransformWeighted> get_matchs(
    int scale, const std::vector<tf2::Vector3> & scan,
    float min_x, float min_y, float max_y, float max_x);
  float match(
    int scale, const nav2_costmap_2d::Costmap2D & costmap,
    const std::vector<tf2::Vector3> & scan, tf2::Transform & transform);

  std::vector<std::shared_ptr<nav2_costmap_2d::Costmap2D>> costmaps_;
};

nav_msgs::msg::OccupancyGrid toMsg(const nav2_costmap_2d::Costmap2D & costmap);

}  // namespace mh_amcl

#endif  // MH_AMCL__MAPMATCHER_HPP_
