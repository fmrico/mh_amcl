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


#include <algorithm>
#include <cmath>
#include <list>
#include <mutex>

#include <Eigen/Dense>
#include <Eigen/LU> 

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
  correct_cg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  others_cg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptions options_c, options_o;
  options_c.callback_group = correct_cg_;
  options_o.callback_group = others_cg_;

  sub_laser_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", 100, std::bind(&MH_AMCL_Node::laser_callback, this, _1), options_c);
  sub_map_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    "map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&MH_AMCL_Node::map_callback, this, _1), options_o);
  sub_init_pose_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", 100, std::bind(&MH_AMCL_Node::initpose_callback, this, _1), options_o);
  pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("amcl_pose", 1);
  particles_pub_ = create_publisher<nav2_msgs::msg::ParticleCloud>("particle_cloud", 1);
}

using CallbackReturnT =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturnT
MH_AMCL_Node::on_configure(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Configuring...");
  particles_population_.push_back(std::make_shared<ParticlesDistribution>(shared_from_this()));

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

  predict_timer_ = create_wall_timer(10ms, std::bind(&MH_AMCL_Node::predict, this), others_cg_);
  correct_timer_ = create_wall_timer(100ms, std::bind(&MH_AMCL_Node::correct, this), correct_cg_);
  reseed_timer_ = create_wall_timer(3s, std::bind(&MH_AMCL_Node::reseed, this), others_cg_);
  hypotesys_timer_ = create_wall_timer(3s, std::bind(&MH_AMCL_Node::manage_hypotesis, this), others_cg_);

  publish_particles_timer_ = create_wall_timer(
    100ms, std::bind(&MH_AMCL_Node::publish_particles, this), others_cg_);
  publish_position_timer_ = create_wall_timer(
    30ms, std::bind(&MH_AMCL_Node::publish_position, this), others_cg_);

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
  std::scoped_lock lock_c(m_others_);

  auto start = now();

  geometry_msgs::msg::TransformStamped odom2bf_msg;
  std::string error;
  if (tf_buffer_.canTransform("odom", "base_footprint", tf2::TimePointZero, &error)) {
    odom2bf_msg = tf_buffer_.lookupTransform("odom", "base_footprint", tf2::TimePointZero);

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
  std::scoped_lock lock_c(m_correct_);

  auto start = now();

  if (last_laser_ == nullptr || last_laser_->ranges.empty() || costmap_ == nullptr) {
    return;
  }

  for (auto & particles : particles_population_) {
    particles->correct_once(*last_laser_, *costmap_);
  }
  RCLCPP_DEBUG_STREAM(get_logger(), "Correct [" << (now() - start).seconds() << " secs]");
}

void
MH_AMCL_Node::reseed()
{
  std::scoped_lock lock_c(m_correct_);
  std::scoped_lock lock_o(m_others_);

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
  std::scoped_lock lock_c(m_correct_);
  std::scoped_lock lock_o(m_others_);

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
    particles_population_.push_back(std::make_shared<ParticlesDistribution>(shared_from_this()));

    (*particles_population_.begin())->on_configure(get_current_state());
    (*particles_population_.begin())->init(pose);

    if (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      (*particles_population_.begin())->on_activate(get_current_state());
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

  std::shared_ptr<ParticlesDistribution> selected_distrib = nullptr;
  float max_quality = -1.0;
  for (const auto & distrib : particles_population_) {
    auto quality = distrib->get_quality();
    // std::cerr << "\t*Q: " << quality << std::endl;
    if (quality > max_quality) {
      max_quality = quality;
      selected_distrib = distrib;
    }
  }

  // std::cerr << "Selected distrib with Q: " << max_quality << std::endl;

  assert(distrib != nullptr);

  geometry_msgs::msg::PoseWithCovarianceStamped pose = selected_distrib->get_pose();
  // std::cerr << "Selected: (" << pose.pose.pose.position.x << ", " << pose.pose.pose.position.y <<
  //   ")" << std::endl;

  // Publish pose
  if (pose_pub_->get_subscription_count() > 0) {
    pose.header.frame_id = "map";
    pose.header.stamp = now();
    pose_pub_->publish(pose);
  }

  // Publish particle cloud
  if (particles_pub_->get_subscription_count() > 0) {
    nav2_msgs::msg::ParticleCloud particles_msgs;
    particles_msgs.header.frame_id = "map";
    particles_msgs.header.stamp = now();

    for (const auto & particle : selected_distrib->get_particles()) {
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
    transform.header.stamp = now();
    transform.child_frame_id = "odom";

    transform.transform = tf2::toMsg(map2odom);

    // std::cerr << "Published: (" << map2robot.getOrigin().x() << ", " <<
    //   map2robot.getOrigin().y() << ")" << std::endl;

    tf_broadcaster_->sendTransform(transform);
  } else {
    RCLCPP_WARN(get_logger(), "Timeout TFs [%s]", error.c_str());
  }
}

void
MH_AMCL_Node::manage_hypotesis()
{
  if (last_laser_ == nullptr || costmap_ == nullptr || matcher_ == nullptr) {return;}

  std::scoped_lock lock_c(m_correct_);
  std::scoped_lock lock_o(m_others_);

  // std::cerr << "================> manage_hypotesis" << std::endl;
  const auto & tfs = matcher_->get_matchs(*last_laser_);

  tf2::Transform selected;
  bool new_distr = false;


  for (const auto & transform : tfs) {
    // std::cerr << "================> " << transform.weight << std::endl;
    if (transform.weight > 0.8) {
      // std::cerr << "================> hypotesis candidate" << std::endl;

      bool covered = false;

      for (const auto & distr : particles_population_) {
        const auto & pose = distr->get_pose().pose.pose;
        double diff_x = pose.position.x - transform.transform.getOrigin().x();
        double diff_y = pose.position.y - transform.transform.getOrigin().y();
        double dist = sqrt(diff_x * diff_x + diff_y * diff_y);

        double tfroll, tfpitch, tfyaw;
        tf2::Matrix3x3(transform.transform.getRotation()).getRPY(tfroll, tfpitch, tfyaw);

        tf2::Quaternion q(pose.orientation.x, pose.orientation.y,
          pose.orientation.z, pose.orientation.w);

        double droll, dpitch, dyaw;
        tf2::Matrix3x3(q).getRPY(droll, dpitch, dyaw);
        double difft = fabs(dyaw - tfyaw);
        while (difft > 2.0 * M_PI) {difft = difft - 2.0 * M_PI;}
        while (difft < -2.0 * M_PI) {difft = difft + 2.0 * M_PI;}
        // std::cerr << "================> " << dist << "   " << difft << std::endl;
        if (dist < 0.5 && difft < 1.0) {
          covered = true;
        }
      }

      if (!covered) {
        selected = transform.transform;
        new_distr = true;
        break;
      }
    }
  }

  if (new_distr) {
    auto aux_distr = std::make_shared<ParticlesDistribution>(shared_from_this());
    aux_distr->on_configure(get_current_state());
    aux_distr->init(selected);
    aux_distr->on_activate(get_current_state());
    particles_population_.push_back(aux_distr);
  }
}

double
MH_AMCL_Node::pdf(const geometry_msgs::msg::Pose & pose,
    geometry_msgs::msg::PoseWithCovariance & distrib)
{
  Eigen::MatrixXf x = fromMsg(pose);
  Eigen::MatrixXf mu = fromMsg(distrib.pose);
  Eigen::MatrixXf E = fromCovMatrix(distrib.covariance);

  std::cerr << "===============> x " << std::endl<< x << std::endl;
  std::cerr << "===============> mu " << std::endl<< mu << std::endl;
  std::cerr << "===============> E " << std::endl<< E << std::endl;
  std::cerr << "===============> (x - mu).transpose() " << std::endl<< (x - mu).transpose() << std::endl;
  std::cerr << "===============> E.inverse() " << std::endl<< E.inverse() << std::endl;
  std::cerr << "===============> (x - mu) " << std::endl<< (x - mu) << std::endl;

  auto val1 = (x - mu).transpose() * E.inverse() * (x - mu);
  std::cerr << "===============> 1 " << val1 << std::endl;
  double pdf_value_1 = exp(-0.5 * val1(0, 0));
  std::cerr << "===============> pdf_value_1 " << pdf_value_1 << std::endl;
  std::cerr << "===============> E.determinant() " << E.determinant() << std::endl;
  double pdf_value_2 = powf(2.0 * M_PI, 6) * E.determinant();
  std::cerr << "===============> pdf_value_2 " << pdf_value_2 << std::endl;
  return pdf_value_1 / pdf_value_2;
}

Eigen::MatrixXf
MH_AMCL_Node::fromMsg(const geometry_msgs::msg::Pose & pose)
{
  Eigen::MatrixXf ret(3, 1);
  ret(0, 0) = pose.position.x;
  ret(1, 0) = pose.position.y;
  
  const auto & or1 = pose.orientation;
  tf2::Quaternion q(or1.x, or1.y, or1.z, or1.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  ret(3, 0) = yaw;

  return ret;
}

Eigen::MatrixXf
MH_AMCL_Node::fromCovMatrix(const std::array<double, 36> & cov)
{
  Eigen::MatrixXf ret(3, 3);
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      int ri = i;
      int rj = j;

      if (i == 3) {ri = 5;}
      if (j == 3) {rj = 5;}

      ret(i, j) = cov[ri * 6 + rj];
    }
  }
  return ret;
}

Eigen::MatrixXf
MH_AMCL_Node::fromStdevs(const std::vector<double> & cov)
{
  Eigen::MatrixXf ret(3, 3);
  for (int i = 0; i < 3; i++) {
    ret(i, i) = cov[i] * cov[i];
  }
  return ret;
}

}  // namespace mh_amcl
