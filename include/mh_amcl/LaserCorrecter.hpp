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

#ifndef MH_AMCL__LASERCORRECTER_HPP_
#define MH_AMCL__LASERCORRECTER_HPP_

#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

#include "mh_amcl/Types.hpp"
#include "mh_amcl/Correcter.hpp"

#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"


namespace mh_amcl
{

class LaserCorrecter : public Correcter<sensor_msgs::msg::LaserScan, nav2_costmap_2d::Costmap2D>
{
public:
  LaserCorrecter(
    nav2_util::LifecycleNode::SharedPtr node, const std::string & topic,
    std::shared_ptr<nav2_costmap_2d::Costmap2D> & map);

  void correct(std::vector<Particle> & particles, rclcpp::Time & update_time) override;

protected:
  tf2::Transform get_tranform_to_read(const sensor_msgs::msg::LaserScan & scan, int index) const;
  double get_error_distance_to_obstacle(
    const tf2::Transform & map2bf, const tf2::Transform & bf2laser,
    const tf2::Transform & laser2point, const sensor_msgs::msg::LaserScan & scan,
    const nav2_costmap_2d::Costmap2D & costmap, double o) const;
  unsigned char get_cost(
    const tf2::Transform & transform,
    const nav2_costmap_2d::Costmap2D & costmap) const;

private:
  double distance_perception_error_;
};

}  // namespace mh_amcl

#endif  // MH_AMCL__LASERCORRECTER_HPP_
