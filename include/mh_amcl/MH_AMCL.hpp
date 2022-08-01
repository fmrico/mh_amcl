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

#ifndef MH_AMCL__MH_AMCL_HPP_
#define MH_AMCL__MH_AMCL_HPP_

#include <mutex>
#include <vector>
#include <list>
#include <memory>

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/transform_datatypes.h"

#include "geometry_msgs/msg/pose_array.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

#include "nav2_costmap_2d/costmap_2d.hpp"

#include "mh_amcl/ParticlesDistribution.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace mh_amcl
{

class MH_AMCL_Node : public rclcpp_lifecycle::LifecycleNode
{
public:
  MH_AMCL_Node();

  void init();

  using CallbackReturnT =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_shutdown(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_error(const rclcpp_lifecycle::State & state);

protected:
  void predict();
  void correct();
  void reseed();
  void publish_particles();
  void publish_position();

private:
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_map_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_laser_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_init_pose_;

  rclcpp::CallbackGroup::SharedPtr correct_cg_;
  rclcpp::CallbackGroup::SharedPtr others_cg_;

  rclcpp::TimerBase::SharedPtr predict_timer_;
  rclcpp::TimerBase::SharedPtr correct_timer_;
  rclcpp::TimerBase::SharedPtr reseed_timer_;
  rclcpp::TimerBase::SharedPtr publish_particles_timer_;
  rclcpp::TimerBase::SharedPtr publish_position_timer_;

  std::list<std::shared_ptr<ParticlesDistribution>> particles_population_;

  tf2::BufferCore tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  tf2::Stamped<tf2::Transform> odom2prevbf_;
  bool valid_prev_odom2bf_ {false};

  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap_;
  sensor_msgs::msg::LaserScan::UniquePtr last_laser_;

  void map_callback(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr & msg);
  void laser_callback(sensor_msgs::msg::LaserScan::UniquePtr lsr_msg);
  void initpose_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr & pose_msg);
  int counter_ {0};

  std::mutex m_correct_;
  std::mutex m_others_;
};

}  // namespace mh_amcl

#endif  // MH_AMCL__MH_AMCL_HPP_
