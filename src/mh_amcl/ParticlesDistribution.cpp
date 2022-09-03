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


#include <tf2_ros/buffer_interface.h>

#include <random>
#include <cmath>
#include <algorithm>
#include <numeric>

#include "visualization_msgs/msg/marker_array.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

#include "mh_amcl/ParticlesDistribution.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace mh_amcl
{

using namespace std::chrono_literals;

ParticlesDistribution::ParticlesDistribution(
  rclcpp_lifecycle::LifecycleNode::SharedPtr parent_node)
: parent_node_(parent_node),
  tf_buffer_(),
  tf_listener_(tf_buffer_),
  rd_(),
  generator_(rd_())
{
  pub_particles_ = parent_node->create_publisher<visualization_msgs::msg::MarkerArray>(
    "poses", 1000);

  if (!parent_node->has_parameter("max_particles")) {
    parent_node->declare_parameter<int>("max_particles", 200lu);
  }
  if (!parent_node->has_parameter("min_particles")) {
    parent_node->declare_parameter<int>("min_particles", 30lu);
  }
  if (!parent_node->has_parameter("init_pos_x")) {
    parent_node->declare_parameter("init_pos_x", 0.0);
  }
  if (!parent_node->has_parameter("init_pos_y")) {
    parent_node->declare_parameter("init_pos_y", 0.0);
  }
  if (!parent_node->has_parameter("init_pos_yaw")) {
    parent_node->declare_parameter("init_pos_yaw", 0.0);
  }
  if (!parent_node->has_parameter("init_error_x")) {
    parent_node->declare_parameter("init_error_x", 0.1);
  }
  if (!parent_node->has_parameter("init_error_y")) {
    parent_node->declare_parameter("init_error_y", 0.1);
  }
  if (!parent_node->has_parameter("init_error_yaw")) {
    parent_node->declare_parameter("init_error_yaw", 0.05);
  }
  if (!parent_node->has_parameter("translation_noise")) {
    parent_node->declare_parameter("translation_noise", 0.01);
  }
  if (!parent_node->has_parameter("rotation_noise")) {
    parent_node->declare_parameter("rotation_noise", 0.01);
  }
  if (!parent_node->has_parameter("distance_perception_error")) {
    parent_node->declare_parameter("distance_perception_error", 0.05);
  }
  if (!parent_node->has_parameter("reseed_percentage_losers")) {
    parent_node->declare_parameter("reseed_percentage_losers", 0.8);
  }
  if (!parent_node->has_parameter("reseed_percentage_winners")) {
    parent_node->declare_parameter("reseed_percentage_winners", 0.03);
  }
  if (!parent_node->has_parameter("good_hypo_thereshold")) {
    parent_node->declare_parameter("good_hypo_thereshold", 0.6);
  }
  if (!parent_node->has_parameter("good_hypo_thereshold")) {
    parent_node->declare_parameter("low_q_hypo_thereshold", 0.25f);
  }
  if (!parent_node->has_parameter("particles_step")) {
    parent_node->declare_parameter<int>("particles_step", 30);
  }
}

using CallbackReturnT =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturnT
ParticlesDistribution::on_configure(const rclcpp_lifecycle::State & state)
{
  parent_node_->get_parameter("max_particles", max_particles_);
  parent_node_->get_parameter("min_particles", min_particles_);
  parent_node_->get_parameter("init_pos_x", init_pos_x_);
  parent_node_->get_parameter("init_pos_y", init_pos_y_);
  parent_node_->get_parameter("init_pos_yaw", init_pos_yaw_);
  parent_node_->get_parameter("init_error_x", init_error_x_);
  parent_node_->get_parameter("init_error_y", init_error_y_);
  parent_node_->get_parameter("init_error_yaw", init_error_yaw_);
  parent_node_->get_parameter("translation_noise", translation_noise_);
  parent_node_->get_parameter("rotation_noise", rotation_noise_);
  parent_node_->get_parameter("distance_perception_error", distance_perception_error_);
  parent_node_->get_parameter("reseed_percentage_losers", reseed_percentage_losers_);
  parent_node_->get_parameter("reseed_percentage_winners", reseed_percentage_winners_);
  parent_node_->get_parameter("low_q_hypo_thereshold", low_q_hypo_thereshold_);
  parent_node_->get_parameter("good_hypo_thereshold", good_hypo_thereshold_);
  parent_node_->get_parameter("particles_step", particles_step_);

  tf2::Transform init_pose;
  init_pose.setOrigin(tf2::Vector3(init_pos_x_, init_pos_y_, 0.0));

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, init_pos_yaw_);
  init_pose.setRotation(q);

  init(init_pose);
  quality_ = 0.25;

  return CallbackReturnT::SUCCESS;
}

void
ParticlesDistribution::update_pose(geometry_msgs::msg::PoseWithCovarianceStamped & pose)
{
  // How many particles we use to determine the pose. They are sorted by prob in last reseed
  size_t particles_used = particles_.size();

  std::vector<double> vpx(particles_used, 0.0);
  std::vector<double> vpy(particles_used, 0.0);
  std::vector<double> vpz(particles_used, 0.0);
  std::vector<double> vrr(particles_used, 0.0);
  std::vector<double> vrp(particles_used, 0.0);
  std::vector<double> vry(particles_used, 0.0);
  std::vector<double> w(particles_used, 0.0);

  for (int i = 0; i < particles_used; i++) {
    const auto & pose = particles_[i].pose.getOrigin();
    vpx[i] = pose.x();
    vpy[i] = pose.y();
    vpz[i] = pose.z();

    double troll, tpitch, tyaw;
    tf2::Matrix3x3(particles_[i].pose.getRotation()).getRPY(troll, tpitch, tyaw);
    vrr[i] = troll;
    vrp[i] = tpitch;
    vry[i] = tyaw;

    w[i] = particles_[i].prob;
  }

  pose.pose.pose.position.x = weighted_mean(vpx, w);
  pose.pose.pose.position.y = weighted_mean(vpy, w);
  pose.pose.pose.position.z = weighted_mean(vpz, w);

  tf2::WithCovarianceStamped<tf2::Transform> ret;
  ret.setOrigin({weighted_mean(vpx, w), weighted_mean(vpx, w), weighted_mean(vpz, w)});

  tf2::Quaternion q;
  double mvrr = angle_weighted_mean(vrr, w);
  double mvrp = angle_weighted_mean(vrp, w);
  double mvry = angle_weighted_mean(vry, w);

  q.setRPY(mvrr, mvrp, mvry);
  ret.setRotation(q);

  pose.pose.pose.orientation.x = q.x();
  pose.pose.pose.orientation.y = q.y();
  pose.pose.pose.orientation.z = q.z();
  pose.pose.pose.orientation.w = q.w();
}

double
ParticlesDistribution::normalize_angle(double angle)
{
  while (angle > M_PI) {angle = angle - 2.0 * M_PI;}
  while (angle < -M_PI) {angle = angle + 2.0 * M_PI;}
  return angle;
}

void
ParticlesDistribution::update_covariance(geometry_msgs::msg::PoseWithCovarianceStamped & pose)
{
  std::vector<double> vpx(particles_.size(), 0.0);
  std::vector<double> vpy(particles_.size(), 0.0);
  std::vector<double> vpz(particles_.size(), 0.0);
  std::vector<double> vrr(particles_.size(), 0.0);
  std::vector<double> vrp(particles_.size(), 0.0);
  std::vector<double> vry(particles_.size(), 0.0);

  for (int i = 0; i < particles_.size(); i++) {
    const auto & pose = particles_[i].pose.getOrigin();
    vpx[i] = pose.x();
    vpy[i] = pose.y();
    vpz[i] = pose.z();

    double troll, tpitch, tyaw;
    tf2::Matrix3x3(particles_[i].pose.getRotation()).getRPY(troll, tpitch, tyaw);
    vrr[i] = troll;
    vrp[i] = tpitch;
    vry[i] = tyaw;
  }

  std::vector<std::vector<double>> vs = {vpx, vpy, vpz, vrr, vrp, vry};

  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      bool is_i_angle = i >= 3;
      bool is_j_angle = j >= 3;
      pose.pose.covariance[i * 6 + j] = covariance(vs[i], vs[j], is_i_angle, is_j_angle);
    }
  }
}

double weighted_mean(const std::vector<double> & v, const std::vector<double> & w)
{
  if (v.empty() || v.size() != w.size()) {
    return 0.0;
  }

  double wsum = 0.0;
  for (int i = 0; i < v.size(); i++) {
    wsum = wsum + v[i] * w[i];
  }
  return wsum;
}

double angle_weighted_mean(const std::vector<double> & v, const std::vector<double> & w)
{
  if (v.empty() || v.size() != w.size()) {
    return 0.0;
  }

  double x = 0.0;
  double y = 0.0;
  for (int i = 0; i < v.size(); i++) {
    x += w[i] * cos(v[i]);
    y += w[i] * sin(v[i]);
  }

  return atan2(y, x);
}

double angle_mean(const std::vector<double> & v)
{
  if (v.empty()) {
    return 0.0;
  }

  double x = 0.0;
  double y = 0.0;
  for (const auto & val : v) {
    x += cos(val);
    y += sin(val);
  }

  return atan2(y, x);
}

double mean(const std::vector<double> & v)
{
  if (v.empty()) {
    return 0.0;
  }
  return std::accumulate(v.begin(), v.end(), 0.0) / v.size();
}

double covariance(
  const std::vector<double> & v1, const std::vector<double> & v2,
  bool v1_is_angle, bool v2_is_angle)
{
  assert(v1.size() == v2.size());

  if (v1.size() < 2) {
    return 0.0;
  }

  double mv1, mv2;
  if (v1_is_angle) {
    mv1 = angle_mean(v1);
  } else {
    mv1 = mean(v1);
  }

  if (v2_is_angle) {
    mv2 = angle_mean(v2);
  } else {
    mv2 = mean(v2);
  }

  double sum = 0.0;

  for (int i = 0; i < v1.size(); i++) {
    sum += (v1[i] - mv1) * (v2[i] - mv2);
  }

  return sum / static_cast<double>(v1.size() - 1.0);
}


CallbackReturnT
ParticlesDistribution::on_activate(const rclcpp_lifecycle::State & state)
{
  pub_particles_->on_activate();

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ParticlesDistribution::on_deactivate(const rclcpp_lifecycle::State & state)
{
  pub_particles_->on_deactivate();
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ParticlesDistribution::on_cleanup(const rclcpp_lifecycle::State & state)
{
  return CallbackReturnT::SUCCESS;
}

void
ParticlesDistribution::init(const tf2::Transform & pose_init)
{
  std::normal_distribution<double> noise_x(0, init_error_x_);
  std::normal_distribution<double> noise_y(0, init_error_y_);
  std::normal_distribution<double> noise_t(0, init_error_yaw_);

  particles_.clear();
  particles_.resize((max_particles_ + min_particles_) / 2);

  for (auto & particle : particles_) {
    particle.prob = 1.0 / static_cast<double>(particles_.size());
    particle.pose = pose_init;

    tf2::Vector3 pose = particle.pose.getOrigin();
    pose.setX(pose.getX() + noise_x(generator_));
    pose.setY(pose.getY() + noise_y(generator_));
    pose.setZ(0.0);

    particle.pose.setOrigin(pose);

    double roll, pitch, yaw;
    tf2::Matrix3x3(particle.pose.getRotation()).getRPY(roll, pitch, yaw);

    double newyaw = yaw + noise_t(generator_);

    tf2::Quaternion q;
    q.setRPY(roll, pitch, newyaw);

    particle.pose.setRotation(q);
  }

  normalize();
  update_covariance(pose_);
  update_pose(pose_);
}

void
ParticlesDistribution::predict(const tf2::Transform & movement)
{
  for (auto & particle : particles_) {
    particle.pose = particle.pose * movement * add_noise(movement);
  }
  update_pose(pose_);
}

tf2::Transform
ParticlesDistribution::add_noise(const tf2::Transform & dm)
{
  tf2::Transform returned_noise;

  std::normal_distribution<double> translation_noise(0.0, translation_noise_);
  std::normal_distribution<double> rotation_noise(0.0, rotation_noise_);

  double noise_tra = translation_noise(generator_);
  double noise_rot = rotation_noise(generator_);

  double x = dm.getOrigin().x() * noise_tra;
  double y = dm.getOrigin().y() * noise_tra;
  double z = 0.0;

  returned_noise.setOrigin(tf2::Vector3(x, y, z));

  double roll, pitch, yaw;
  tf2::Matrix3x3(dm.getRotation()).getRPY(roll, pitch, yaw);

  double newyaw = yaw * noise_rot;

  tf2::Quaternion q;
  q.setRPY(roll, pitch, newyaw);
  returned_noise.setRotation(q);

  return returned_noise;
}

void
ParticlesDistribution::publish_particles(int base_idx, const std_msgs::msg::ColorRGBA & color) const
{
  if (pub_particles_->get_subscription_count() == 0) {
    return;
  }

  visualization_msgs::msg::MarkerArray msg;

  int counter = 0;
  for (auto & particle : particles_) {
    visualization_msgs::msg::Marker pose_msg;

    pose_msg.header.frame_id = "map";
    pose_msg.header.stamp = parent_node_->now();
    pose_msg.id = base_idx * 200 + counter++;
    pose_msg.type = visualization_msgs::msg::Marker::ARROW;
    pose_msg.type = visualization_msgs::msg::Marker::ADD;
    pose_msg.lifetime = rclcpp::Duration(1s);

    const auto translation = particle.pose.getOrigin();
    const auto rotation = particle.pose.getRotation();

    pose_msg.pose.position.x = translation.x();
    pose_msg.pose.position.y = translation.y();
    pose_msg.pose.position.z = translation.z();

    pose_msg.pose.orientation.x = rotation.x();
    pose_msg.pose.orientation.y = rotation.y();
    pose_msg.pose.orientation.z = rotation.z();
    pose_msg.pose.orientation.w = rotation.w();

    pose_msg.scale.x = 0.1;
    pose_msg.scale.y = 0.01;
    pose_msg.scale.z = 0.01;

    pose_msg.color = color;

    msg.markers.push_back(pose_msg);
  }

  pub_particles_->publish(msg);
}

void
ParticlesDistribution::correct_once(
  const sensor_msgs::msg::LaserScan & scan, const nav2_costmap_2d::Costmap2D & costmap)
{
  std::string error;
  if (tf_buffer_.canTransform(
      scan.header.frame_id, "base_footprint", tf2_ros::fromMsg(scan.header.stamp), &error))
  {
    auto bf2laser_msg = tf_buffer_.lookupTransform(
      "base_footprint", scan.header.frame_id, tf2_ros::fromMsg(scan.header.stamp));
    tf2::fromMsg(bf2laser_msg, bf2laser_);
  } else {
    RCLCPP_WARN(
      parent_node_->get_logger(), "Timeout while waiting TF %s -> base_footprint [%s]",
      scan.header.frame_id.c_str(), error.c_str());
    return;
  }

  const double o = distance_perception_error_;

  static const float inv_sqrt_2pi = 0.3989422804014327;
  const double normal_comp_1 = inv_sqrt_2pi / o;

  for (auto & p : particles_) {
    p.hits = 0.0;
  }

  for (int j = 0; j < scan.ranges.size(); j++) {
    if (std::isnan(scan.ranges[j]) || std::isinf(scan.ranges[j])) {continue;}

    tf2::Transform laser2point = get_tranform_to_read(scan, j);

    for (int i = 0; i < particles_.size(); i++) {
      auto & p = particles_[i];

      double calculated_distance = get_error_distance_to_obstacle(
        p.pose, bf2laser_, laser2point, scan, costmap, o);

      if (!std::isinf(calculated_distance)) {
        const double a = calculated_distance / o;
        const double normal_comp_2 = std::exp(-0.5 * a * a);

        double prob = std::clamp(normal_comp_1 * normal_comp_2, 0.0, 1.0);
        p.prob = std::max(p.prob + prob, 0.000001);

        p.hits += prob;
      }
    }
  }

  normalize();

  // Calculate quality
  quality_ = 0.0;
  for (auto & p : particles_) {
    p.hits = p.hits / static_cast<float>(scan.ranges.size());
    quality_ = std::max(quality_, p.hits);
  }
}

tf2::Transform
ParticlesDistribution::get_tranform_to_read(const sensor_msgs::msg::LaserScan & scan, int index)
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

unsigned char
ParticlesDistribution::get_cost(
  const tf2::Transform & transform, const nav2_costmap_2d::Costmap2D & costmap)
{
  unsigned int mx, my;
  if (costmap.worldToMap(transform.getOrigin().x(), transform.getOrigin().y(), mx, my)) {
    return costmap.getCost(mx, my);
  } else {
    return nav2_costmap_2d::NO_INFORMATION;
  }
}

double
ParticlesDistribution::get_error_distance_to_obstacle(
  const tf2::Transform & map2bf, const tf2::Transform & bf2laser,
  const tf2::Transform & laser2point, const sensor_msgs::msg::LaserScan & scan,
  const nav2_costmap_2d::Costmap2D & costmap, double o)
{
  if (std::isinf(laser2point.getOrigin().x()) || std::isnan(laser2point.getOrigin().x())) {
    return std::numeric_limits<double>::infinity();
  }

  tf2::Transform map2laser = map2bf * bf2laser;
  tf2::Transform map2point = map2laser * laser2point;
  tf2::Transform map2point_aux = map2point;
  tf2::Transform uvector;
  tf2::Vector3 unit = laser2point.getOrigin() / laser2point.getOrigin().length();

  if (get_cost(map2point, costmap) == nav2_costmap_2d::LETHAL_OBSTACLE) {return 0.0;}

  float dist = costmap.getResolution();
  while (dist < (3.0 * o)) {
    uvector.setOrigin(unit * dist);
    // For positive
    map2point = map2point_aux * uvector;
    auto cost = get_cost(map2point, costmap);

    if (cost == nav2_costmap_2d::LETHAL_OBSTACLE) {return dist;}

    // For negative
    uvector.setOrigin(uvector.getOrigin() * -1.0);
    map2point = map2point_aux * uvector;
    cost = get_cost(map2point, costmap);

    if (cost == nav2_costmap_2d::LETHAL_OBSTACLE) {return dist;}
    dist = dist + costmap.getResolution();
  }

  return std::numeric_limits<double>::infinity();
}

void
ParticlesDistribution::reseed()
{
  // Sort particles by prob
  std::sort(
    particles_.begin(), particles_.end(),
    [](const Particle & a, const Particle & b) -> bool
    {
      return a.prob > b.prob;
    });

  double percentage_losers = reseed_percentage_losers_;
  double percentage_winners = reseed_percentage_winners_;

  auto number_particles = particles_.size();
  if (get_quality() < low_q_hypo_thereshold_) {
    number_particles = std::clamp(
      static_cast<int>(number_particles + particles_step_), min_particles_, max_particles_);
    int new_particles = number_particles - particles_.size();
    for (int i = 0; i < new_particles; i++) {
      Particle new_p = particles_.front();
      particles_.push_back(new_p);
    }
  } else if (get_quality() > good_hypo_thereshold_) {
    number_particles = std::clamp(
      static_cast<int>(number_particles - particles_step_), min_particles_, max_particles_);
    for (int i = 0; i < particles_.size() - number_particles; i++) {
      particles_.pop_back();
    }
  }

  int number_losers = number_particles * percentage_losers;
  int number_no_losers = number_particles - number_losers;
  int number_winners = number_particles * percentage_winners;

  std::vector<Particle> new_particles(particles_.begin(), particles_.begin() + number_no_losers);

  std::normal_distribution<double> selector(0, number_winners);
  std::normal_distribution<double> noise_x(0, init_error_x_ * init_error_x_);
  std::normal_distribution<double> noise_y(0, init_error_y_ * init_error_y_);
  std::normal_distribution<double> noise_t(0, init_error_yaw_ * init_error_yaw_);

  for (int i = 0; i < number_losers; i++) {
    int index = std::clamp(static_cast<int>(selector(generator_)), 0, number_winners);

    Particle p;
    p.prob = new_particles.back().prob;

    auto w_pose = particles_[i].pose.getOrigin();

    double nx = noise_x(generator_);
    double ny = noise_y(generator_);

    p.pose.setOrigin({w_pose.x() + nx, w_pose.y() + ny, w_pose.z()});

    double roll, pitch, yaw;

    tf2::Matrix3x3 m(particles_[i].pose.getRotation());
    m.getRPY(roll, pitch, yaw);

    double newyaw = yaw + noise_t(generator_);
    while (newyaw > M_PI) {newyaw -= 2.0 * M_PI;}
    while (newyaw < -M_PI) {newyaw += 2.0 * M_PI;}

    tf2::Quaternion q;
    q.setRPY(roll, pitch, newyaw);

    p.pose.setRotation(q);

    new_particles.push_back(p);
  }

  particles_ = new_particles;
  normalize();
  update_covariance(pose_);
}

void
ParticlesDistribution::normalize()
{
  double sum = 0.0;
  std::for_each(
    particles_.begin(), particles_.end(), [&sum](const Particle & p) {
      sum += p.prob;
    });

  if (sum != 0.0) {
    std::for_each(
      particles_.begin(), particles_.end(), [&](Particle & p) {
        p.prob = p.prob / sum;
      });
  }
}

void
ParticlesDistribution::merge(ParticlesDistribution & other)
{
  size_t size = particles_.size();
  particles_.insert(particles_.end(), other.particles_.begin(), other.particles_.end());

  std::sort(
    particles_.begin(), particles_.end(),
    [](const Particle & a, const Particle & b) -> bool
    {
      return a.prob > b.prob;
    });
  particles_.erase(particles_.begin() + size, particles_.end());
}

std_msgs::msg::ColorRGBA
getColor(Color color_id, double alpha)
{
  std_msgs::msg::ColorRGBA color;

  switch (color_id) {
    case RED:
      color.r = 0.8;
      color.g = 0.1;
      color.b = 0.1;
      color.a = alpha;
      break;
    case GREEN:
      color.r = 0.1;
      color.g = 0.8;
      color.b = 0.1;
      color.a = alpha;
      break;
    case BLUE:
      color.r = 0.1;
      color.g = 0.1;
      color.b = 0.8;
      color.a = alpha;
      break;
    case WHITE:
      color.r = 1.0;
      color.g = 1.0;
      color.b = 1.0;
      color.a = alpha;
      break;
    case GREY:
      color.r = 0.9;
      color.g = 0.9;
      color.b = 0.9;
      color.a = alpha;
      break;
    case DARK_GREY:
      color.r = 0.6;
      color.g = 0.6;
      color.b = 0.6;
      color.a = alpha;
      break;
    case BLACK:
      color.r = 0.0;
      color.g = 0.0;
      color.b = 0.0;
      color.a = alpha;
      break;
    case YELLOW:
      color.r = 1.0;
      color.g = 1.0;
      color.b = 0.0;
      color.a = alpha;
      break;
    case ORANGE:
      color.r = 1.0;
      color.g = 0.5;
      color.b = 0.0;
      color.a = alpha;
      break;
    case BROWN:
      color.r = 0.597;
      color.g = 0.296;
      color.b = 0.0;
      color.a = alpha;
      break;
    case PINK:
      color.r = 1.0;
      color.g = 0.4;
      color.b = 1;
      color.a = alpha;
      break;
    case LIME_GREEN:
      color.r = 0.6;
      color.g = 1.0;
      color.b = 0.2;
      color.a = alpha;
      break;
    case PURPLE:
      color.r = 0.597;
      color.g = 0.0;
      color.b = 0.597;
      color.a = alpha;
      break;
    case CYAN:
      color.r = 0.0;
      color.g = 1.0;
      color.b = 1.0;
      color.a = alpha;
      break;
    case MAGENTA:
      color.r = 1.0;
      color.g = 0.0;
      color.b = 1.0;
      color.a = alpha;
      break;
  }
  return color;
}

}  // namespace mh_amcl
