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
#include "mh_amcl/Utils.hpp"
#include "mh_amcl/LaserCorrecter.hpp"

#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/transform_datatypes.h>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "octomap/octomap.h"

#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mh_amcl
{

LaserCorrecter::LaserCorrecter(
    const std::string & name, nav2_util::LifecycleNode::SharedPtr node,
    std::shared_ptr<octomap::OcTree> & map)
: Correcter<sensor_msgs::msg::LaserScan, octomap::OcTree>(name, node, map)
{
  if (!node->has_parameter(name + ".distance_perception_error")) {
    node->declare_parameter(name + ".distance_perception_error", 0.05);
  }

  node->get_parameter(name + ".distance_perception_error", distance_perception_error_);
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

  // for (auto & p : particles) {
  //   p.possible_hits += static_cast<float>(last_perception_->ranges.size());
  // }

  for (int j = 0; j < last_perception_->ranges.size(); j++) {
    if (std::isnan(last_perception_->ranges[j])) {continue;}

    tf2::Vector3 laser2point_u = get_perception_unit_vector(*last_perception_, j);

    for (int i = 0; i < particles.size(); i++) {
      auto & p = particles[i];

      double calculated_distance = get_distance_to_obstacle(
        p.pose, bf2laser, laser2point_u, *last_perception_, *map_);

      if (std::isinf(last_perception_->ranges[j]) && std::isinf(calculated_distance)) {
        p.prob = std::max(p.prob * map_->getProbHit(), 0.000001);

        p.possible_hits += 1.0;
        p.hits += 1.0;
      } else {
        double diff = abs(last_perception_->ranges[j] - calculated_distance);
        const double a = diff / o;
        const double normal_comp_2 = std::exp(-0.5 * a * a);

        double prob = std::clamp(normal_comp_1 * normal_comp_2, 0.0, 1.0);
        p.prob = std::max(p.prob + prob, 0.000001);
        
        p.possible_hits += 1.0;
        // p.hits += 1.0;

        if (prob >  map_->getProbHit()) {
          p.hits += 1.0;
        }
      }
    }
  }

  update_time = last_perception_->header.stamp;
  // last_perception_ = nullptr;
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

tf2::Vector3
LaserCorrecter::get_perception_unit_vector(const sensor_msgs::msg::LaserScan & scan, int index) const
{
  double dist = 1.0;
  double angle = scan.angle_min + static_cast<double>(index) * scan.angle_increment;

  tf2::Transform ret;

  double x = dist * cos(angle);
  double y = dist * sin(angle);

  return {x, y, 0.0};
}

double
LaserCorrecter::get_distance_to_obstacle(
  const tf2::Transform & map2bf, const tf2::Transform & bf2laser,
  const tf2::Vector3 unit_vector, const sensor_msgs::msg::LaserScan & scan,
  const octomap::OcTree & octomap) const
{
  tf2::Transform map2laser = map2bf * bf2laser;
  tf2::Transform map2laser_rot = map2laser;
  map2laser_rot.setOrigin({0.0, 0.0, 0.0});

  tf2::Vector3 map2point_unit = map2laser_rot * unit_vector;

  tf2::Vector3 & laser_pos = map2laser.getOrigin();

  octomap::point3d hit;
  bool is_obstacle = octomap.castRay(
    {static_cast<float>(laser_pos.x()), static_cast<float>(laser_pos.y()), static_cast<float>(laser_pos.z())},
    {static_cast<float>(map2point_unit.x()), static_cast<float>(map2point_unit.y()), static_cast<float>(map2point_unit.z())},
    hit, true, 
    scan.range_max);

  if (!is_obstacle) {
    return std::numeric_limits<double>::infinity(); 
  } else {
    return hit.distance(
      {
        static_cast<float>(map2laser.getOrigin().x()),
        static_cast<float>(map2laser.getOrigin().y()),
        static_cast<float>(map2laser.getOrigin().z())
      });
  }
}

double
LaserCorrecter::get_occupancy(
  const tf2::Transform & transform, const octomap::OcTree & octomap) const
{
  const auto & transl = transform.getOrigin();
  octomap::OcTreeNode * node = octomap.search(transl.x(), transl.y(), transl.z());
  
  if (node == NULL) {
    return octomap.getClampingThresMin();
  }

  return node->getOccupancy();
}

}  // namespace mh_amcl
