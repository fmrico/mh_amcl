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


#ifndef MH_AMCL__PARTICLESDISTRIBUTION_HPP_
#define MH_AMCL__PARTICLESDISTRIBUTION_HPP_

#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/transform_datatypes.h>

#include <vector>
#include <random>

#include "sensor_msgs/msg/laser_scan.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "nav2_costmap_2d/costmap_2d.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace mh_amcl
{

typedef struct
{
  tf2::Transform pose;
  double prob;
} Particle;

typedef enum TColor
{
  RED, GREEN, BLUE, WHITE, GREY, DARK_GREY, BLACK, YELLOW, ORANGE, BROWN, PINK,
  LIME_GREEN, PURPLE, CYAN, MAGENTA, NUM_COLORS
} Color;

std_msgs::msg::ColorRGBA
getColor(Color color_id, double alpha = 1.0);

class ParticlesDistribution
{
public:
  explicit ParticlesDistribution(rclcpp_lifecycle::LifecycleNode::SharedPtr parent_node);

  void init(const tf2::Transform & pose_init);
  void predict(const tf2::Transform & movement);
  void correct_once(
    const sensor_msgs::msg::LaserScan & scan, const nav2_costmap_2d::Costmap2D & costmap);
  void reseed();

  using CallbackReturnT =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State & state);

  void publish_particles(const std_msgs::msg::ColorRGBA & color) const;

protected:
  rclcpp_lifecycle::LifecycleNode::SharedPtr parent_node_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    pub_particles_;

  tf2::Transform add_noise(const tf2::Transform & dm);
  tf2::Transform get_tranform_to_read(const sensor_msgs::msg::LaserScan & scan, int index);
  double get_error_distance_to_obstacle(
    const tf2::Transform & map2bf, const tf2::Transform & bf2laser,
    const tf2::Transform & laser2point, const sensor_msgs::msg::LaserScan & scan,
    const nav2_costmap_2d::Costmap2D & costmap, double o);
  unsigned char get_cost(
    const tf2::Transform & transform,
    const nav2_costmap_2d::Costmap2D & costmap);
  void normalize();

  std::random_device rd_;
  std::mt19937 generator_;

  static const int NUM_PART = 200;
  std::vector<Particle> particles_;

  tf2::BufferCore tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  tf2::Stamped<tf2::Transform> bf2laser_;
  bool bf2laser_init_ {false};

  // AMCL Parameters
  int max_particles_;
  int min_particles_;
  double init_pos_x_;
  double init_pos_y_;
  double init_pos_yaw_;
  double init_error_x_;
  double init_error_y_;
  double init_error_yaw_;
  double translation_noise_;
  double rotation_noise_;
  double distance_perception_error_;
  double reseed_percentage_losers_;
  double reseed_percentage_winners_;
};

}  // namespace mh_amcl

#endif  // MH_AMCL__PARTICLESDISTRIBUTION_HPP_
