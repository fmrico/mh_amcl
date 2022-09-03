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

#include <Eigen/Dense>
#include <Eigen/LU>

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
#include "nav2_msgs/msg/particle_cloud.hpp"

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "mh_amcl/ParticlesDistribution.hpp"
#include "mh_amcl/MapMatcher.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_util/lifecycle_node.hpp"

namespace mh_amcl
{

class MH_AMCL_Node : public nav2_util::LifecycleNode
{
public:
  explicit MH_AMCL_Node(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  void init();

  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_error(const rclcpp_lifecycle::State & state) override;

protected:
  void predict();
  void correct();
  void reseed();
  void publish_particles();
  void publish_position();
  void manage_hypotesis();

  void get_distances(
    const geometry_msgs::msg::Pose & pose1, const geometry_msgs::msg::Pose & pose2,
    double & dist_xy, double & dist_theta);
  unsigned char get_cost(const geometry_msgs::msg::Pose & pose);
  geometry_msgs::msg::Pose toMsg(const tf2::Transform & tf);

private:
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_map_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_laser_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_init_pose_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav2_msgs::msg::ParticleCloud>::SharedPtr particles_pub_;

  rclcpp::TimerBase::SharedPtr predict_timer_;
  rclcpp::TimerBase::SharedPtr correct_timer_;
  rclcpp::TimerBase::SharedPtr reseed_timer_;
  rclcpp::TimerBase::SharedPtr hypotesys_timer_;
  rclcpp::TimerBase::SharedPtr publish_particles_timer_;
  rclcpp::TimerBase::SharedPtr publish_position_timer_;

  int max_hypotheses_;
  bool multihypothesis_;
  float min_candidate_weight_;
  double min_candidate_distance_;
  double min_candidate_angle_;
  float low_q_hypo_thereshold_;
  float very_low_q_hypo_thereshold_;
  double hypo_merge_distance_;
  double hypo_merge_angle_;
  float good_hypo_thereshold_;
  float min_hypo_diff_winner_;

  rclcpp::Time last_time_;

  std::list<std::shared_ptr<ParticlesDistribution>> particles_population_;
  std::shared_ptr<ParticlesDistribution> current_amcl_;
  float current_amcl_q_;

  tf2::BufferCore tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  tf2::Stamped<tf2::Transform> odom2prevbf_;
  bool valid_prev_odom2bf_ {false};

  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap_;
  sensor_msgs::msg::LaserScan::UniquePtr last_laser_;
  std::shared_ptr<mh_amcl::MapMatcher> matcher_;

  void map_callback(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr & msg);
  void laser_callback(sensor_msgs::msg::LaserScan::UniquePtr lsr_msg);
  void initpose_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr & pose_msg);
  int counter_ {0};
};

}  // namespace mh_amcl

#endif  // MH_AMCL__MH_AMCL_HPP_
