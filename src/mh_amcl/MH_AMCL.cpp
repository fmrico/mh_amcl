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

#include "mh_amcl/LaserCorrecter.hpp"
#include "mh_amcl/PointCloudCorrecter.hpp"
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
  sub_gridmap_ = create_subscription<grid_map_msgs::msg::GridMap>(
    "grid_map_map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&MH_AMCL_Node::gridmap_callback, this, _1));

  sub_octomap_ = create_subscription<octomap_msgs::msg::Octomap>(
    "octomap_map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&MH_AMCL_Node::octomap_callback, this, _1));


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

  std::vector<std::string> correction_sources;
  declare_parameter("correction_sources", correction_sources);

  std::vector<std::string> matchers;
  declare_parameter("matchers", matchers);

  gridmap_ = std::make_shared<grid_map::GridMap>();
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

  std::vector<std::string> correction_sources;
  get_parameter("correction_sources", correction_sources);

  for (const auto & correction_source : correction_sources) {
    std::string correction_source_type;
    declare_parameter(correction_source + ".type", correction_source_type);
    get_parameter(correction_source + ".type", correction_source_type);

    CorrecterBase * correcter;
    if (correction_source_type == "laser") {
      correcter = new LaserCorrecter(correction_source, shared_from_this(), octomap_);
    }
    if (correction_source_type == "pointcloud") {
      correcter = new PointCloudCorrecter(correction_source, shared_from_this(), octomap_);
    }

    correcter->type_ = correction_source_type;
    correcters_.push_back(correcter);
    RCLCPP_INFO(
      get_logger(), "Created corrected [%s] (type [%s])",
      correction_source.c_str(), correction_source_type.c_str());
  }

  std::vector<std::string> matchers;
  get_parameter("matchers", matchers);

  for (const auto & matcher : matchers) {
    std::string matchers_type;
    declare_parameter(matcher + ".type", matchers_type);
    get_parameter(matcher + ".type", matchers_type);

    MapMatcherBase * new_matcher;
    if (matchers_type == "matcher2d") {
      new_matcher = new MapMatcher(matcher, shared_from_this());
    }

    new_matcher->type_ = matchers_type;
    matchers_.push_back(new_matcher);
    RCLCPP_INFO(
      get_logger(), "Created matcher [%s] (type [%s])",
      matcher.c_str(), matchers_type.c_str());
  }

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

  reentrant_1_cg_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  other_cg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  predict_timer_ = create_wall_timer(10ms, std::bind(&MH_AMCL_Node::predict, this), reentrant_1_cg_);
  correct_timer_ = create_wall_timer(500ms, std::bind(&MH_AMCL_Node::correct, this), reentrant_1_cg_);  
  reseed_timer_ = create_wall_timer(3s, std::bind(&MH_AMCL_Node::reseed, this), other_cg_);
  hypotesis_timer_ = create_wall_timer(3s, std::bind(&MH_AMCL_Node::manage_hypotesis, this), other_cg_);

  publish_particles_timer_ = create_wall_timer(
    100ms, std::bind(&MH_AMCL_Node::publish_particles, this),other_cg_);
  publish_position_timer_ = create_wall_timer(
    30ms, std::bind(&MH_AMCL_Node::publish_position, this), other_cg_);
  publish_position_tf_timer_ = create_wall_timer(
    10ms, std::bind(&MH_AMCL_Node::publish_position_tf, this), other_cg_);

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
  hypotesis_timer_ = nullptr;

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
        particles->predict(bfprev2bf, gridmap_);
      }
    }

    valid_prev_odom2bf_ = true;
    odom2prevbf_ = odom2bf;
  }

  publish_position_tf();

  RCLCPP_DEBUG_STREAM(get_logger(), "Predict [" << (now() - start).seconds() << " secs]");
}


void
MH_AMCL_Node::gridmap_callback(const grid_map_msgs::msg::GridMap::ConstSharedPtr & msg)
{
  grid_map::GridMapRosConverter::fromMessage(*msg, *gridmap_);
}

void
MH_AMCL_Node::octomap_callback(const octomap_msgs::msg::Octomap::ConstSharedPtr & msg)
{
  octomap::OcTree * ao = dynamic_cast<octomap::OcTree*>(octomap_msgs::msgToMap(*msg));
  octomap_ = std::shared_ptr<octomap::OcTree>(ao);
  octomap_->setProbHit(0.7);
  octomap_->setProbMiss(0.4);
  octomap_->setClampingThresMax(0.97);
  octomap_->setClampingThresMin(0.12);
}

void
MH_AMCL_Node::correct()
{
  std::scoped_lock l(correct_mutex_);
  auto start = now();

  for (auto & particles : particles_population_) {
    particles->correct_once(correcters_, last_time_);
  }

  for (auto & correcter : correcters_) {
    correcter->clear_perception();
  }

  RCLCPP_DEBUG_STREAM(get_logger(), "Correct [" << (now() - start).seconds() << " secs]");
}

void
MH_AMCL_Node::reseed()
{
  std::scoped_lock l2(correct_mutex_);

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
  tf2::WithCovarianceStamped<tf2::Transform> input_pose = current_amcl_->get_pose();

  geometry_msgs::msg::PoseStamped poseStamped;
  tf2::toMsg(static_cast<tf2::Transform>(input_pose), poseStamped.pose);


  geometry_msgs::msg::PoseWithCovarianceStamped pose;
  pose.pose.pose = poseStamped.pose;

  for (size_t i = 0; i < 6; ++i) {
    for (size_t j = 0; j < 6; ++j) {
      size_t index = i * 6 + j; // Calculate the index in the 1D array
      pose.pose.covariance[index] = input_pose.cov_mat_[i][j];
    }
  }

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
}

void
MH_AMCL_Node::publish_position_tf()
{
  tf2::Transform map2robot = static_cast<tf2::Transform>(current_amcl_->get_pose());
  tf2::Stamped<tf2::Transform> robot2odom;

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
  if (matchers_.empty()) {return;}

  if (multihypothesis_) {
    std::list<TransformWeighted> tfs;
    for (auto matcher : matchers_) {
      tfs.merge(matcher->get_matchs());
    }

    // Create new Hypothesis
    for (const auto & transform : tfs) {
      if (transform.weight > min_candidate_weight_) {
        bool covered = false;

        for (const auto & distr : particles_population_) {
          const tf2::Transform pose1 = static_cast<tf2::Transform>(distr->get_pose());          
          tf2::Transform pose2 = transform.transform;

          double dist = pose1.getOrigin().distance2(pose2.getOrigin());
          double difft = 2.0 * std::acos(std::abs(pose1.getRotation().dot(pose2.getRotation())));

          if (dist < min_candidate_distance_ && difft < min_candidate_angle_) {
            covered = true;
          }
        }

        if (!covered && particles_population_.size() < max_hypotheses_) {
          std::cerr << "Create new Particle Distribution " << std::endl;
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

      /*grid_map::Position grid_map_pos((*it)->get_pose().pose.pose.position.x, (*it)->get_pose().pose.pose.position.y);
      std::cerr << "Asking for the pose (" << (*it)->get_pose().pose.pose.position.x << ", " <<
        (*it)->get_pose().pose.pose.position.y << ")" ;

      for (const auto & layer :  gridmap_->getLayers()) {
        std::cerr << "L [" << layer << "]" << std::endl;
      }

      std::cerr << "[" << gridmap_->getSize().x() <<" x " <<  gridmap_->getSize().y() << "]" << std::endl;

      bool in_free;
      try {
        float value = gridmap_->atPosition("occupancy", grid_map_pos);
        in_free = value < 5.0;
        std::cerr << " = " << value << std::endl;
      } catch(std::out_of_range e) {
        std::cerr << " Exception" << e.what() << std::endl;
        in_free = false;
      }
      */
      bool in_free = true;

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
        
        const tf2::Transform pose1 = static_cast<tf2::Transform>((*it1)->get_pose());          
        const tf2::Transform pose2 = static_cast<tf2::Transform>((*it2)->get_pose());          

        double dist_xy = pose1.getOrigin().distance2(pose2.getOrigin());
        double dist_t = 2.0 * std::acos(std::abs(pose1.getRotation().dot(pose2.getRotation())));

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
  }

  std::cerr << "=====================================" << std::endl;
  for (const auto & amcl : particles_population_) {
    if (amcl == current_amcl_) {
      RCLCPP_INFO_STREAM(get_logger(), "Quality ->\t" << amcl->get_quality());
    } else {
      RCLCPP_INFO_STREAM(get_logger(), amcl->get_quality());
    }
  }
  std::cerr << "=====================================" << std::endl;
}

}  // namespace mh_amcl
