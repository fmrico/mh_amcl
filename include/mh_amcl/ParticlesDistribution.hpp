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

#include "grid_map_msgs/msg/grid_map.hpp"
#include "grid_map_ros/grid_map_ros.hpp"

#include "mh_amcl/Types.hpp"
#include "mh_amcl/Correcter.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace mh_amcl
{

std_msgs::msg::ColorRGBA
getColor(Color color_id, double alpha = 1.0);

class ParticlesDistribution
{
public:
  explicit ParticlesDistribution(rclcpp_lifecycle::LifecycleNode::SharedPtr parent_node);

  void init(const tf2::Transform & pose_init);
  void predict(const tf2::Transform & movement, std::shared_ptr<grid_map::GridMap> gridmap);
  void correct_once(const std::list<CorrecterBase*> & correcters, rclcpp::Time & update_time);
  void reseed();
  const std::vector<mh_amcl::Particle> & get_particles() const {return particles_;}

  using CallbackReturnT =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State & state);

  void publish_particles(int base_idx, const std_msgs::msg::ColorRGBA & color) const;
  tf2::WithCovarianceStamped<tf2::Transform> get_pose() const {return pose_;}
  float get_quality() {return quality_;}
  void merge(ParticlesDistribution & other);

protected:
  rclcpp_lifecycle::LifecycleNode::SharedPtr parent_node_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    pub_particles_;

  tf2::Transform add_noise(const tf2::Transform & dm);

  void normalize();
  double normalize_angle(double angle);
  void update_pose(tf2::WithCovarianceStamped<tf2::Transform> & pose);
  void update_covariance(tf2::WithCovarianceStamped<tf2::Transform> & pose);

  tf2::WithCovarianceStamped<tf2::Transform> pose_;

  std::random_device rd_;
  std::mt19937 generator_;

  std::vector<Particle> particles_;
  float quality_;

  tf2::BufferCore tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  tf2::Stamped<tf2::Transform> bf2laser_;
  bool bf2laser_init_ {false};

  // AMCL Parameters
  int max_particles_;
  int min_particles_;
  double init_pos_x_;
  double init_pos_y_;
  double init_pos_z_;
  double init_pos_yaw_;
  double init_pos_pitch_;
  double init_pos_roll_;
  double init_error_x_;
  double init_error_y_;
  double init_error_z_;
  double init_error_yaw_;
  double init_error_pitch_;
  double init_error_roll_;
  double translation_noise_;
  double rotation_noise_;
  double reseed_percentage_losers_;
  double reseed_percentage_winners_;
  float good_hypo_thereshold_;
  float low_q_hypo_thereshold_;
  int particles_step_;
};

double weighted_mean(const std::vector<double> & v, const std::vector<double> & w);
double angle_weighted_mean(const std::vector<double> & v, const std::vector<double> & w);
double mean(const std::vector<double> & v);
double angle_mean(const std::vector<double> & v);
double covariance(
  const std::vector<double> & v1, const std::vector<double> & v2,
  bool v1_is_angle = false, bool v2_is_angle = false);

}  // namespace mh_amcl

#endif  // MH_AMCL__PARTICLESDISTRIBUTION_HPP_
