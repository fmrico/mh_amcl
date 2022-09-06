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


#include <Eigen/Dense>
#include <Eigen/LU>

#include <algorithm>
#include <cmath>
#include <list>

#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/transform_datatypes.h"
#include "tf2/convert.h"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/msg/particle_cloud.hpp"
#include "nav2_msgs/msg/particle.hpp"

#include "nav2_costmap_2d/costmap_2d_publisher.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

#include "mh_amcl/MH_AMCL.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace mh_amcl
{

using std::placeholders::_1;
using namespace std::chrono_literals;

MH_AMCL_Node::MH_AMCL_Node(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("mh_amcl", "", options),
  tf_buffer_(),
  tf_listener_(tf_buffer_)
{
  sub_laser_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", rclcpp::QoS(100).best_effort(), std::bind(&MH_AMCL_Node::laser_callback, this, _1));
  sub_map_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    "map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&MH_AMCL_Node::map_callback, this, _1));
  sub_init_pose_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", 100, std::bind(&MH_AMCL_Node::initpose_callback, this, _1));
  pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("amcl_pose", 1);
  particles_pub_ = create_publisher<nav2_msgs::msg::ParticleCloud>("particle_cloud", 1);

  declare_parameter<int>("max_hypotheses", 5);
  declare_parameter<bool>("multihypothesis", true);
  declare_parameter<float>("min_candidate_weight", 0.5f);
  declare_parameter<double>("min_candidate_distance", 0.5);
  declare_parameter<double>("min_candidate_angle", M_PI_2);
  declare_parameter<float>("low_q_hypo_thereshold", 0.25f);
  declare_parameter<float>("very_low_q_hypo_thereshold", 0.1);
  declare_parameter<double>("hypo_merge_distance", 0.2);
  declare_parameter<double>("hypo_merge_angle", 0.3);
  declare_parameter<float>("good_hypo_thereshold", 0.6);
  declare_parameter<float>("min_hypo_diff_winner", 0.2);
}

using CallbackReturnT =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturnT
MH_AMCL_Node::on_configure(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Configuring...");

  get_parameter("multihypothesis", multihypothesis_);
  get_parameter("max_hypotheses", max_hypotheses_);
  get_parameter("min_candidate_weight", min_candidate_weight_);
  get_parameter("min_candidate_distance", min_candidate_distance_);
  get_parameter("min_candidate_angle", min_candidate_angle_);
  get_parameter("low_q_hypo_thereshold", low_q_hypo_thereshold_);
  get_parameter("very_low_q_hypo_thereshold", very_low_q_hypo_thereshold_);
  get_parameter("hypo_merge_distance", hypo_merge_distance_);
  get_parameter("hypo_merge_angle", hypo_merge_angle_);
  get_parameter("good_hypo_thereshold", good_hypo_thereshold_);
  get_parameter("min_hypo_diff_winner", min_hypo_diff_winner_);


  current_amcl_ = std::make_shared<ParticlesDistribution>(shared_from_this());
  current_amcl_q_ = 1.0;

  particles_population_.push_back(current_amcl_);

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

  std::list<CallbackReturnT> ret;
  for (auto & particles : particles_population_) {
    ret.push_back(particles->on_configure(state));
  }

  if (std::count(ret.begin(), ret.end(), CallbackReturnT::SUCCESS) == ret.size()) {
    RCLCPP_INFO(get_logger(), "Configured");
    return CallbackReturnT::SUCCESS;
  }

  RCLCPP_ERROR(get_logger(), "Error configuring");
  return CallbackReturnT::FAILURE;
}

CallbackReturnT
MH_AMCL_Node::on_activate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Activating...");

  bool use_sim_time;
  get_parameter("use_sim_time", use_sim_time);

  if (use_sim_time) {
    RCLCPP_INFO(get_logger(), "use_sim_time = true");
  } else {
    RCLCPP_INFO(get_logger(), "use_sim_time = false");
  }

  predict_timer_ = create_wall_timer(10ms, std::bind(&MH_AMCL_Node::predict, this));
  correct_timer_ = create_wall_timer(100ms, std::bind(&MH_AMCL_Node::correct, this));
  reseed_timer_ = create_wall_timer(3s, std::bind(&MH_AMCL_Node::reseed, this));
  hypotesys_timer_ = create_wall_timer(3s, std::bind(&MH_AMCL_Node::manage_hypotesis, this));

  publish_particles_timer_ = create_wall_timer(
    100ms, std::bind(&MH_AMCL_Node::publish_particles, this));
  publish_position_timer_ = create_wall_timer(
    30ms, std::bind(&MH_AMCL_Node::publish_position, this));

  std::list<CallbackReturnT> ret;
  for (auto & particles : particles_population_) {
    ret.push_back(particles->on_activate(state));
  }

  if (std::count(ret.begin(), ret.end(), CallbackReturnT::SUCCESS) == ret.size()) {
    RCLCPP_INFO(get_logger(), "Activated");

    RCLCPP_INFO(get_logger(), "Creating Bond Init");
    createBond();
    RCLCPP_INFO(get_logger(), "Creating Bond Finish");
    return CallbackReturnT::SUCCESS;
  }

  RCLCPP_ERROR(get_logger(), "Error activating");
  return CallbackReturnT::FAILURE;
}

CallbackReturnT
MH_AMCL_Node::on_deactivate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Deactivating...");

  predict_timer_ = nullptr;
  correct_timer_ = nullptr;
  reseed_timer_ = nullptr;
  publish_particles_timer_ = nullptr;
  publish_position_timer_ = nullptr;
  hypotesys_timer_ = nullptr;

  std::list<CallbackReturnT> ret;
  for (auto & particles : particles_population_) {
    ret.push_back(particles->on_deactivate(state));
  }

  if (std::count(ret.begin(), ret.end(), CallbackReturnT::SUCCESS) == ret.size()) {
    RCLCPP_INFO(get_logger(), "Deactivated");
    return CallbackReturnT::SUCCESS;
  }

  // destroy bond connection
  destroyBond();

  RCLCPP_ERROR(get_logger(), "Error deactivating");
  return CallbackReturnT::FAILURE;
}

CallbackReturnT
MH_AMCL_Node::on_cleanup(const rclcpp_lifecycle::State & state)
{
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
MH_AMCL_Node::on_shutdown(const rclcpp_lifecycle::State & state)
{
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
MH_AMCL_Node::on_error(const rclcpp_lifecycle::State & state)
{
  return CallbackReturnT::SUCCESS;
}

void
MH_AMCL_Node::publish_particles()
{
  auto start = now();

  Color color = RED;
  int i = 0;
  for (const auto & particles : particles_population_) {
    color = static_cast<Color>((color + 1) % NUM_COLORS);
    particles->publish_particles(i++, getColor(color));
  }
  RCLCPP_DEBUG_STREAM(get_logger(), "Publish [" << (now() - start).seconds() << " secs]");
}

void
MH_AMCL_Node::predict()
{
  auto start = now();

  geometry_msgs::msg::TransformStamped odom2bf_msg;
  std::string error;
  if (tf_buffer_.canTransform("odom", "base_footprint", tf2::TimePointZero, &error)) {
    odom2bf_msg = tf_buffer_.lookupTransform("odom", "base_footprint", tf2::TimePointZero);

    last_time_ = odom2bf_msg.header.stamp;

    tf2::Stamped<tf2::Transform> odom2bf;
    tf2::fromMsg(odom2bf_msg, odom2bf);

    if (valid_prev_odom2bf_) {
      tf2::Transform bfprev2bf = odom2prevbf_.inverse() * odom2bf;

      for (auto & particles : particles_population_) {
        particles->predict(bfprev2bf);
      }
    }

    valid_prev_odom2bf_ = true;
    odom2prevbf_ = odom2bf;
  }

  RCLCPP_DEBUG_STREAM(get_logger(), "Predict [" << (now() - start).seconds() << " secs]");
}

void
MH_AMCL_Node::map_callback(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr & msg)
{
  costmap_ = std::make_shared<nav2_costmap_2d::Costmap2D>(*msg);
  matcher_ = std::make_shared<mh_amcl::MapMatcher>(*msg);
}

void
MH_AMCL_Node::laser_callback(sensor_msgs::msg::LaserScan::UniquePtr lsr_msg)
{
  last_laser_ = std::move(lsr_msg);
  counter_ = 0;
}

void
MH_AMCL_Node::correct()
{
  auto start = now();

  if (last_laser_ == nullptr || last_laser_->ranges.empty() || costmap_ == nullptr) {
    return;
  }

  for (auto & particles : particles_population_) {
    particles->correct_once(*last_laser_, *costmap_);
  }

  last_time_ = last_laser_->header.stamp;

  RCLCPP_DEBUG_STREAM(get_logger(), "Correct [" << (now() - start).seconds() << " secs]");
}

void
MH_AMCL_Node::reseed()
{
  auto start = now();

  for (auto & particles : particles_population_) {
    particles->reseed();
  }

  RCLCPP_DEBUG_STREAM(
    get_logger(), "==================Reseed [" << (now() - start).seconds() << " secs]");
}

void
MH_AMCL_Node::initpose_callback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr & pose_msg)
{
  if (pose_msg->header.frame_id == "map") {
    tf2::Transform pose;
    pose.setOrigin(
      {
        pose_msg->pose.pose.position.x,
        pose_msg->pose.pose.position.y,
        pose_msg->pose.pose.position.z});

    pose.setRotation(
      {
        pose_msg->pose.pose.orientation.x,
        pose_msg->pose.pose.orientation.y,
        pose_msg->pose.pose.orientation.z,
        pose_msg->pose.pose.orientation.w});

    particles_population_.clear();
    current_amcl_q_ = 1.0;
    current_amcl_ = std::make_shared<ParticlesDistribution>(shared_from_this());

    particles_population_.push_back(current_amcl_);
    current_amcl_->on_configure(get_current_state());
    current_amcl_->init(pose);

    if (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      current_amcl_->on_activate(get_current_state());
    }
  } else {
    RCLCPP_WARN(
      get_logger(), "Not possible to init particles in frame %s",
      pose_msg->header.frame_id.c_str());
  }
}

void
MH_AMCL_Node::publish_position()
{
  if (costmap_ == nullptr || last_laser_ == nullptr) {
    return;
  }

  geometry_msgs::msg::PoseWithCovarianceStamped pose = current_amcl_->get_pose();

  // Publish pose
  if (pose_pub_->get_subscription_count() > 0) {
    pose.header.frame_id = "map";
    pose.header.stamp = last_time_;
    pose_pub_->publish(pose);
  }

  // Publish particle cloud
  if (particles_pub_->get_subscription_count() > 0) {
    nav2_msgs::msg::ParticleCloud particles_msgs;
    particles_msgs.header.frame_id = "map";
    particles_msgs.header.stamp = last_time_;

    for (const auto & particle : current_amcl_->get_particles()) {
      nav2_msgs::msg::Particle p;
      p.pose.position.x = particle.pose.getOrigin().x();
      p.pose.position.y = particle.pose.getOrigin().y();
      p.pose.position.z = particle.pose.getOrigin().z();
      p.pose.orientation = tf2::toMsg(particle.pose.getRotation());
      particles_msgs.particles.push_back(p);
    }

    particles_pub_->publish(particles_msgs);
  }

  // Publish tf map -> odom

  tf2::Transform map2robot;
  tf2::Stamped<tf2::Transform> robot2odom;
  const auto & tpos = pose.pose.pose.position;
  const auto & tor = pose.pose.pose.orientation;
  map2robot.setOrigin({tpos.x, tpos.y, tpos.z});
  map2robot.setRotation({tor.x, tor.y, tor.z, tor.w});

  std::string error;
  if (tf_buffer_.canTransform("base_footprint", "odom", tf2::TimePointZero, &error)) {
    auto robot2odom_msg = tf_buffer_.lookupTransform("base_footprint", "odom", tf2::TimePointZero);
    tf2::fromMsg(robot2odom_msg, robot2odom);

    auto map2odom = map2robot * robot2odom;

    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = "map";
    transform.header.stamp = last_time_;
    transform.child_frame_id = "odom";

    transform.transform = tf2::toMsg(map2odom);

    tf_broadcaster_->sendTransform(transform);
  } else {
    RCLCPP_WARN(get_logger(), "Timeout TFs [%s]", error.c_str());
  }
}

geometry_msgs::msg::Pose
MH_AMCL_Node::toMsg(const tf2::Transform & tf)
{
  geometry_msgs::msg::Pose ret;
  ret.position.x = tf.getOrigin().x();
  ret.position.y = tf.getOrigin().y();
  ret.position.z = tf.getOrigin().z();
  ret.orientation.x = tf.getRotation().x();
  ret.orientation.y = tf.getRotation().y();
  ret.orientation.z = tf.getRotation().z();
  ret.orientation.w = tf.getRotation().w();

  return ret;
}

void
MH_AMCL_Node::manage_hypotesis()
{
  if (last_laser_ == nullptr || costmap_ == nullptr || matcher_ == nullptr) {return;}

  if (!multihypothesis_) {return;}

  const auto & tfs = matcher_->get_matchs(*last_laser_);

  // Create new Hypothesis
  for (const auto & transform : tfs) {
    if (transform.weight > min_candidate_weight_) {
      bool covered = false;

      for (const auto & distr : particles_population_) {
        const auto & pose = distr->get_pose().pose.pose;
        geometry_msgs::msg::Pose posetf = toMsg(transform.transform);

        double dist, difft;
        get_distances(pose, posetf, dist, difft);

        if (dist < min_candidate_distance_ && difft < min_candidate_angle_) {
          covered = true;
        }
      }

      if (!covered && particles_population_.size() < max_hypotheses_) {
        auto aux_distr = std::make_shared<ParticlesDistribution>(shared_from_this());
        aux_distr->on_configure(get_current_state());
        aux_distr->init(transform.transform);
        aux_distr->on_activate(get_current_state());
        particles_population_.push_back(aux_distr);
      }
    }

    if (particles_population_.size() == max_hypotheses_) {break;}
  }

  auto it = particles_population_.begin();
  while (it != particles_population_.end()) {
    bool low_quality = (*it)->get_quality() < low_q_hypo_thereshold_;
    bool very_low_quality = (*it)->get_quality() < very_low_q_hypo_thereshold_;
    bool max_hypo_reached = particles_population_.size() == max_hypotheses_;
    bool in_free = get_cost((*it)->get_pose().pose.pose) == nav2_costmap_2d::FREE_SPACE;

    if (particles_population_.size() > 1 && (!in_free || very_low_quality || (low_quality && max_hypo_reached))) {
      it = particles_population_.erase(it);
      if (current_amcl_ == *it) {
        current_amcl_ = particles_population_.front();
        current_amcl_q_ = low_q_hypo_thereshold_ + 0.1;
      }
    } else {
      ++it;
    }
  }

  auto it1 = particles_population_.begin();
  auto it2 = particles_population_.begin();
  while (it1 != particles_population_.end()) {
    while (it2 != particles_population_.end()) {
      if (*it1 == *it2) {
        ++it2;
        continue;
      }

      double dist_xy, dist_t;
      get_distances((*it1)->get_pose().pose.pose, (*it2)->get_pose().pose.pose, dist_xy, dist_t);
      if (dist_xy < hypo_merge_distance_ && dist_t < hypo_merge_angle_) {
        (*it1)->merge(**it2);
        it2 = particles_population_.erase(it2);
        if (current_amcl_ == *it2) {
          current_amcl_ = particles_population_.front();
          current_amcl_q_ = current_amcl_->get_quality();
        }
      } else {
        ++it2;
      }
    }
    ++it1;
  }

  current_amcl_q_ = current_amcl_->get_quality();

  for (const auto & amcl : particles_population_) {
    if (amcl->get_quality() > good_hypo_thereshold_ &&
      amcl->get_quality() > (current_amcl_q_ + min_hypo_diff_winner_))
    {
      current_amcl_q_ = amcl->get_quality();
      current_amcl_ = amcl;
    }
  }

  bool is_selected = false;
  for (const auto & amcl : particles_population_) {
    if (amcl == current_amcl_) {
      is_selected = true;
    }
  }

  if (!is_selected) {
    current_amcl_ = particles_population_.front();
    current_amcl_q_ = current_amcl_->get_quality();
  }

  std::cerr << "=====================================" << std::endl;
  for (const auto & amcl : particles_population_) {
    if (amcl == current_amcl_) {
      std::cerr << "->\t";
    }
    std::cerr << amcl->get_quality() << std::endl;
  }
  std::cerr << "=====================================" << std::endl;
}

unsigned char
MH_AMCL_Node::get_cost(const geometry_msgs::msg::Pose & pose)
{
  unsigned int i, j;
  costmap_->worldToMap(pose.position.x, pose.position.y, i, j);

  if (i > 0 && j > 0 && i < costmap_->getSizeInCellsX() && j < costmap_->getSizeInCellsY()) {
    return costmap_->getCost(i, j);
  } else {
    return nav2_costmap_2d::NO_INFORMATION;
  }
}

void
MH_AMCL_Node::get_distances(
  const geometry_msgs::msg::Pose & pose1, const geometry_msgs::msg::Pose & pose2,
  double & dist_xy, double & dist_theta)
{
  double diff_x = pose1.position.x - pose2.position.x;
  double diff_y = pose1.position.y - pose2.position.y;
  dist_xy = sqrt(diff_x * diff_x + diff_y * diff_y);

  tf2::Quaternion q1(pose1.orientation.x, pose1.orientation.y,
    pose1.orientation.z, pose1.orientation.w);
  tf2::Quaternion q2(pose2.orientation.x, pose2.orientation.y,
    pose2.orientation.z, pose2.orientation.w);

  double roll1, pitch1, yaw1;
  double roll2, pitch2, yaw2;
  tf2::Matrix3x3(q1).getRPY(roll1, pitch1, yaw1);
  tf2::Matrix3x3(q2).getRPY(roll2, pitch2, yaw2);

  dist_theta = fabs(atan2(sin(yaw1 - yaw2), cos(yaw1 - yaw2)));
}

}  // namespace mh_amcl
