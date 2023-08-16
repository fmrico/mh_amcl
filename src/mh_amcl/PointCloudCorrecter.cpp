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
#include "mh_amcl/PointCloudCorrecter.hpp"

#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/transform_datatypes.h>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_types_conversion.h"

#include "octomap/octomap.h"

#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"


namespace mh_amcl
{

PointCloudCorrecter::PointCloudCorrecter(
  const std::string & name, nav2_util::LifecycleNode::SharedPtr node,
  std::shared_ptr<octomap::OcTree> & map)
: Correcter<sensor_msgs::msg::PointCloud2, octomap::OcTree>(name, node, map)
{
  if (!node->has_parameter(name + ".distance_perception_error")) {
    node->declare_parameter(name + ".distance_perception_error", 0.05);
  }
  if (!node->has_parameter(name + ".max_perception_distance")) {
    node->declare_parameter(name + ".max_perception_distance", 2.0);
  }
  if (!node->has_parameter(name + ".point_step")) {
    node->declare_parameter(name + ".point_step", 1);
  }
  node->get_parameter(name + ".distance_perception_error", distance_perception_error_);
  node->get_parameter(name + ".max_perception_distance", max_perception_distance_);
  node->get_parameter(name + ".point_step", point_step_);
}

void
PointCloudCorrecter::correct(std::vector<Particle> & particles, rclcpp::Time & update_time)
{
  if (last_perception_ == nullptr) {
    return;
  }
  if (map_ == nullptr) {
    return;
  }

  tf2::Stamped<tf2::Transform> bf2sensor;
  std::string error;
  if (tf_buffer_.canTransform(
      last_perception_->header.frame_id, "base_footprint",
      tf2_ros::fromMsg(last_perception_->header.stamp), &error))
  {
    auto bf2sensor_msg = tf_buffer_.lookupTransform(
      "base_footprint", last_perception_->header.frame_id,
      tf2_ros::fromMsg(last_perception_->header.stamp));
    tf2::fromMsg(bf2sensor_msg, bf2sensor);
  } else {
    RCLCPP_WARN(
      node_->get_logger(), "Timeout while waiting TF %s -> base_footprint [%s]",
      last_perception_->header.frame_id.c_str(), error.c_str());
    return;
  }

  const double o = distance_perception_error_;

  static const float inv_sqrt_2pi = 0.3989422804014327;
  const double normal_comp_1 = inv_sqrt_2pi / o;

  pointcloud_.clear();
  pcl::fromROSMsg(*last_perception_, pointcloud_);

  int point_count = 0;

  for (const auto & point : pointcloud_.points) {
    if (std::isnan(point.x)) {continue;}

    if (point_count++ % point_step_ != 0) {
      continue;
    }

    tf2::Vector3 sensor2point(point.x, point.y, point.z);
    tf2::Vector3 sensor2point_u = sensor2point / sensor2point.length();

    for (int i = 0; i < particles.size(); i++) {
      auto & p = particles[i];

      double calculated_distance = get_distance_to_obstacle(
        p.pose, bf2sensor, sensor2point_u, *last_perception_, *map_);

      if (std::isinf(point.x) && std::isinf(calculated_distance)) {
        p.prob = std::max(p.prob + map_->getProbHit(), 0.000001);

        p.possible_hits += 1.0;
        p.hits += 1.0;
      } else {
        double diff = abs(sensor2point.length() - calculated_distance);
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
  last_perception_ = nullptr;
}


double
PointCloudCorrecter::get_distance_to_obstacle(
  const tf2::Transform & map2bf, const tf2::Transform & bf2sensor,
  const tf2::Vector3 unit_vector, const sensor_msgs::msg::PointCloud2 & scan,
  const octomap::OcTree & octomap) const
{
  tf2::Transform map2sensor = map2bf * bf2sensor;
  tf2::Transform map2sensor_rot = map2sensor;
  map2sensor_rot.setOrigin({0.0, 0.0, 0.0});

  tf2::Vector3 map2point_unit = map2sensor_rot * unit_vector;

  tf2::Vector3 & sensor_pos = map2sensor.getOrigin();

  octomap::point3d hit;
  bool is_obstacle = octomap.castRay(
    {static_cast<float>(sensor_pos.x()), static_cast<float>(sensor_pos.y()), static_cast<float>(sensor_pos.z())},
    {static_cast<float>(map2point_unit.x()), static_cast<float>(map2point_unit.y()), static_cast<float>(map2point_unit.z())},
    hit, true, 
    max_perception_distance_);

  if (!is_obstacle) {
    return std::numeric_limits<double>::infinity(); 
  } else {
    return hit.distance(
      {
        static_cast<float>(map2sensor.getOrigin().x()),
        static_cast<float>(map2sensor.getOrigin().y()),
        static_cast<float>(map2sensor.getOrigin().z())
      });
  }
}

double
PointCloudCorrecter::get_occupancy(
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
