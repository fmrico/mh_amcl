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


#include "mh_amcl/Types.hpp"
#include "mh_amcl/LaserCorrecter.hpp"

#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/transform_datatypes.h>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mh_amcl
{

LaserCorrecter::LaserCorrecter(
    nav2_util::LifecycleNode::SharedPtr node, const std::string & topic,
    std::shared_ptr<nav2_costmap_2d::Costmap2D> & map)
: Correcter<sensor_msgs::msg::LaserScan, nav2_costmap_2d::Costmap2D>(node, topic, map)
{
  if (!node->has_parameter("distance_perception_error")) {
    node->declare_parameter("distance_perception_error", 0.05);
  }

  node->get_parameter("distance_perception_error", distance_perception_error_);
}

void
LaserCorrecter::correct(std::vector<Particle> & particles, rclcpp::Time & update_time)
{
  if (last_perception_ == nullptr) {
    return;
  }
  if (map_ == nullptr) {
    return;
  }

  tf2::Stamped<tf2::Transform> bf2laser;

  std::string error;
  if (tf_buffer_.canTransform(
      last_perception_->header.frame_id, "base_footprint",
      tf2_ros::fromMsg(last_perception_->header.stamp), &error))
  {
    auto bf2laser_msg = tf_buffer_.lookupTransform(
      "base_footprint", last_perception_->header.frame_id,
      tf2_ros::fromMsg(last_perception_->header.stamp));
    tf2::fromMsg(bf2laser_msg, bf2laser);
  } else {
    RCLCPP_WARN(
      node_->get_logger(), "Timeout while waiting TF %s -> base_footprint [%s]",
      last_perception_->header.frame_id.c_str(), error.c_str());
    return;
  }


  const double o = distance_perception_error_;

  static const float inv_sqrt_2pi = 0.3989422804014327;
  const double normal_comp_1 = inv_sqrt_2pi / o;

  for (auto & p : particles) {
    p.possible_hits += static_cast<float>(last_perception_->ranges.size());
  }

  for (int j = 0; j < last_perception_->ranges.size(); j++) {
    if (std::isnan(last_perception_->ranges[j]) || std::isinf(last_perception_->ranges[j])) {continue;}

    tf2::Transform laser2point = get_tranform_to_read(*last_perception_, j);

    for (int i = 0; i < particles.size(); i++) {
      auto & p = particles[i];

      double calculated_distance = get_error_distance_to_obstacle(
        p.pose, bf2laser, laser2point, *last_perception_, *map_, o);

      if (!std::isinf(calculated_distance)) {
        const double a = calculated_distance / o;
        const double normal_comp_2 = std::exp(-0.5 * a * a);

        double prob = std::clamp(normal_comp_1 * normal_comp_2, 0.0, 1.0);
        p.prob = std::max(p.prob + prob, 0.000001);

        p.hits += prob;
      }
    }
  }

  update_time = last_perception_->header.stamp;
  last_perception_ = nullptr;
}

tf2::Transform
LaserCorrecter::get_tranform_to_read(const sensor_msgs::msg::LaserScan & scan, int index) const
{
  double dist = scan.ranges[index];
  double angle = scan.angle_min + static_cast<double>(index) * scan.angle_increment;

  tf2::Transform ret;

  double x = dist * cos(angle);
  double y = dist * sin(angle);

  ret.setOrigin({x, y, 0.0});
  ret.setRotation({0.0, 0.0, 0.0, 1.0});

  return ret;
}

double
LaserCorrecter::get_error_distance_to_obstacle(
  const tf2::Transform & map2bf, const tf2::Transform & bf2laser,
  const tf2::Transform & laser2point, const sensor_msgs::msg::LaserScan & scan,
  const nav2_costmap_2d::Costmap2D & costmap, double o) const
{
  if (std::isinf(laser2point.getOrigin().x()) || std::isnan(laser2point.getOrigin().x())) {
    return std::numeric_limits<double>::infinity();
  }

  tf2::Transform map2laser = map2bf * bf2laser;
  tf2::Transform map2point = map2laser * laser2point;
  tf2::Transform map2point_aux = map2point;
  tf2::Transform uvector;
  tf2::Vector3 unit = laser2point.getOrigin() / laser2point.getOrigin().length();

  if (get_cost(map2point, costmap) == nav2_costmap_2d::LETHAL_OBSTACLE) {return 0.0;}

  float dist = costmap.getResolution();
  while (dist < (3.0 * o)) {
    uvector.setOrigin(unit * dist);
    // For positive
    map2point = map2point_aux * uvector;
    auto cost = get_cost(map2point, costmap);

    if (cost == nav2_costmap_2d::LETHAL_OBSTACLE) {return dist;}

    // For negative
    uvector.setOrigin(uvector.getOrigin() * -1.0);
    map2point = map2point_aux * uvector;
    cost = get_cost(map2point, costmap);

    if (cost == nav2_costmap_2d::LETHAL_OBSTACLE) {return dist;}
    dist = dist + costmap.getResolution();
  }

  return std::numeric_limits<double>::infinity();
}

unsigned char
LaserCorrecter::get_cost(
  const tf2::Transform & transform, const nav2_costmap_2d::Costmap2D & costmap) const
{
  unsigned int mx, my;
  if (costmap.worldToMap(transform.getOrigin().x(), transform.getOrigin().y(), mx, my)) {
    return costmap.getCost(mx, my);
  } else {
    return nav2_costmap_2d::NO_INFORMATION;
  }
}

}  // namespace mh_amcl
