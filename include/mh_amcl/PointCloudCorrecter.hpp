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

#ifndef MH_AMCL__POINTCLOUDCORRECTER_HPP_
#define MH_AMCL__POINTCLOUDCORRECTER_HPP_

#include "octomap/octomap.h"

#include "sensor_msgs/msg/point_cloud2.hpp"

#include "mh_amcl/Types.hpp"
#include "mh_amcl/Correcter.hpp"

#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_types_conversion.h"

#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"


namespace mh_amcl
{

class PointCloudCorrecter : public Correcter<sensor_msgs::msg::PointCloud2, octomap::OcTree>
{
public:
  PointCloudCorrecter(
    const std::string & name, nav2_util::LifecycleNode::SharedPtr node,
    std::shared_ptr<octomap::OcTree> & map);

  void correct(std::vector<Particle> & particles, rclcpp::Time & update_time) override;

protected:
  tf2::Transform get_tranform_to_read(const sensor_msgs::msg::PointCloud2 & scan, int index) const;
  double get_distance_to_obstacle(
    const tf2::Transform & map2bf, const tf2::Transform & bf2laser,
    const tf2::Vector3 unit_vector, const sensor_msgs::msg::PointCloud2 & scan,
    const octomap::OcTree & costmap) const;
  double get_occupancy(
    const tf2::Transform & transform,
    const octomap::OcTree & costmap) const;

private:
  double distance_perception_error_;
  double max_perception_distance_;
  int point_step_;

  pcl::PointCloud<pcl::PointXYZ> pointcloud_;
};

}  // namespace mh_amcl

#endif  // MH_AMCL__POINTCLOUDCORRECTER_HPP_
