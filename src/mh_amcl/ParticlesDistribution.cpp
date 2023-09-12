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

#include "grid_map_msgs/msg/grid_map.hpp"
#include "grid_map_ros/grid_map_ros.hpp"

#include "mh_amcl/ParticlesDistribution.hpp"

#include "mh_amcl/LaserCorrecter.hpp"
#include "mh_amcl/PointCloudCorrecter.hpp"

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
  if (!parent_node->has_parameter("init_pos_z")) {
    parent_node->declare_parameter("init_pos_z", 0.0);
  }
  if (!parent_node->has_parameter("init_pos_pitch")) {
    parent_node->declare_parameter("init_pos_pitch", 0.0);
  }
  if (!parent_node->has_parameter("init_pos_roll")) {
    parent_node->declare_parameter("init_pos_roll", 0.0);
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
  if (!parent_node->has_parameter("init_error_pitch")) {
    parent_node->declare_parameter("init_error_pitch", 0.01);
  }
  if (!parent_node->has_parameter("init_error_roll")) {
    parent_node->declare_parameter("init_error_roll", 0.01);
  }
  if (!parent_node->has_parameter("translation_noise")) {
    parent_node->declare_parameter("translation_noise", 0.01);
  }
  if (!parent_node->has_parameter("rotation_noise")) {
    parent_node->declare_parameter("rotation_noise", 0.01);
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
  parent_node_->get_parameter("init_pos_z", init_pos_z_);
  parent_node_->get_parameter("init_pos_yaw", init_pos_yaw_);
  parent_node_->get_parameter("init_pos_pitch", init_pos_pitch_);
  parent_node_->get_parameter("init_pos_roll", init_pos_roll_);
  parent_node_->get_parameter("init_error_x", init_error_x_);
  parent_node_->get_parameter("init_error_y", init_error_y_);
  parent_node_->get_parameter("init_error_yaw", init_error_yaw_);
  parent_node_->get_parameter("init_error_pitch", init_error_pitch_);
  parent_node_->get_parameter("init_error_roll", init_error_roll_);
  parent_node_->get_parameter("translation_noise", translation_noise_);
  parent_node_->get_parameter("rotation_noise", rotation_noise_);
  parent_node_->get_parameter("reseed_percentage_losers", reseed_percentage_losers_);
  parent_node_->get_parameter("reseed_percentage_winners", reseed_percentage_winners_);
  parent_node_->get_parameter("low_q_hypo_thereshold", low_q_hypo_thereshold_);
  parent_node_->get_parameter("good_hypo_thereshold", good_hypo_thereshold_);
  parent_node_->get_parameter("particles_step", particles_step_);

  tf2::Transform init_pose;
  init_pose.setOrigin(tf2::Vector3(init_pos_x_, init_pos_y_, init_pos_z_));

  tf2::Quaternion q;
  q.setRPY(init_pos_roll_, init_pos_pitch_, init_pos_yaw_);
  init_pose.setRotation(q);

  init(init_pose);
  quality_ = 0.25;

  return CallbackReturnT::SUCCESS;
}

void
ParticlesDistribution::update_pose(tf2::WithCovarianceStamped<tf2::Transform> & pose)
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
    const auto & poseaux = particles_[i].pose.getOrigin();
    vpx[i] = poseaux.x();
    vpy[i] = poseaux.y();
    vpz[i] = poseaux.z();

    double troll, tpitch, tyaw;
    tf2::Matrix3x3(particles_[i].pose.getRotation()).getRPY(troll, tpitch, tyaw);
    vrr[i] = troll;
    vrp[i] = tpitch;
    vry[i] = tyaw;

    w[i] = particles_[i].prob;
  }

  pose.setOrigin({weighted_mean(vpx, w), weighted_mean(vpy, w), weighted_mean(vpz, w)});

  tf2::Quaternion q;
  double mvrr = angle_weighted_mean(vrr, w);
  double mvrp = angle_weighted_mean(vrp, w);
  double mvry = angle_weighted_mean(vry, w);

  q.setRPY(mvrr, mvrp, mvry);
  pose.setRotation(q);
}

double
ParticlesDistribution::normalize_angle(double angle)
{
  while (angle > M_PI) {angle = angle - 2.0 * M_PI;}
  while (angle < -M_PI) {angle = angle + 2.0 * M_PI;}
  return angle;
}

void
ParticlesDistribution::update_covariance(tf2::WithCovarianceStamped<tf2::Transform> & pose)
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
      pose.cov_mat_[i][j] = covariance(vs[i], vs[j], is_i_angle, is_j_angle);
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
  std::normal_distribution<double> noise_z(0, init_error_z_);
  std::normal_distribution<double> noise_yw(0, init_error_yaw_);
  std::normal_distribution<double> noise_p(0, init_error_pitch_);
  std::normal_distribution<double> noise_r(0, init_error_roll_);

  particles_.clear();
  particles_.resize((max_particles_ + min_particles_) / 2);

  for (auto & particle : particles_) {
    particle.prob = 1.0 / static_cast<double>(particles_.size());
    particle.pose = pose_init;

    tf2::Vector3 pose = particle.pose.getOrigin();
    pose.setX(pose.getX() + noise_x(generator_));
    pose.setY(pose.getY() + noise_y(generator_));
    pose.setZ(pose.getZ() + noise_z(generator_));

    particle.pose.setOrigin(pose);

    double roll, pitch, yaw;
    tf2::Matrix3x3(particle.pose.getRotation()).getRPY(roll, pitch, yaw);

    double newyaw = yaw + noise_yw(generator_);
    double newpitch = pitch + noise_p(generator_);
    double newroll = roll + noise_r(generator_);

    tf2::Quaternion q;
    q.setRPY(newroll, newpitch, newyaw);

    particle.pose.setRotation(q);
  }

  normalize();
  update_covariance(pose_);
  update_pose(pose_);
}

void
ParticlesDistribution::predict(const tf2::Transform & movement, std::shared_ptr<grid_map::GridMap> gridmap)
{
  auto & gridpmap_pos = gridmap->getPosition();

  pose_.setData(static_cast<tf2::Transform>(pose_) * movement);

  if (gridmap != nullptr) {
    auto & pos = pose_.getOrigin();

    try {
      grid_map::Position particle_pos(pos.x(), pos.y());
      particle_pos = particle_pos + gridpmap_pos;
      float elevation = gridmap->atPosition("elevation", particle_pos);
      pose_.setOrigin({pos.x(), pos.y(), static_cast<double>(elevation)});
    } catch (std::out_of_range e) {
      std::cerr << "Error accessing gridmap pos at " << pos.x() << ", " << pos.y() << ")" << std::endl;
    }
  }


  for (auto & particle : particles_) {
    particle.pose = particle.pose * movement * add_noise(movement);

    if (gridmap != nullptr) {
      auto & pos = particle.pose.getOrigin();

      try {
        grid_map::Position particle_pos(pos.x(), pos.y());
        particle_pos = particle_pos + gridpmap_pos;
        float elevation = gridmap->atPosition("elevation", particle_pos);
        particle.pose.setOrigin({pos.x(), pos.y(), static_cast<double>(elevation)});
      } catch (std::out_of_range e) {
        std::cerr << "Error accessing gridmap pos at " << pos.x() << ", " << pos.y() << ")" << std::endl;
      }
    }
  }  
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
  double z = dm.getOrigin().z() * noise_tra;

  returned_noise.setOrigin(tf2::Vector3(x, y, z));

  double roll, pitch, yaw;
  tf2::Matrix3x3(dm.getRotation()).getRPY(roll, pitch, yaw);

  double newyaw = yaw * noise_rot;
  double newpitch = pitch * noise_rot;
  double newroll = roll * noise_rot;

  tf2::Quaternion q;
  q.setRPY(newroll, newpitch, newyaw);
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
ParticlesDistribution::correct_once(const std::list<CorrecterBase*> & correcters, rclcpp::Time & update_time)
{
  bool new_data = false;
  for (const auto correcter : correcters) {
    if (correcter->type_ == "laser") {
      auto * correcter_casted = dynamic_cast<LaserCorrecter*>(correcter);
      new_data = new_data || (correcter_casted->last_perception_ != nullptr);
    }
    if (correcter->type_ == "pointcloud") {
      auto * correcter_casted = dynamic_cast<PointCloudCorrecter*>(correcter);
      new_data = new_data || (correcter_casted->last_perception_ != nullptr);
    }
  } 
  
  if (!new_data) {
    return;
  }

  for (const auto correcter : correcters) {
    if (correcter->type_ == "laser") {
      auto * correcter_casted = dynamic_cast<LaserCorrecter*>(correcter);
      correcter_casted->correct(particles_, update_time);
    }
    if (correcter->type_ == "pointcloud") {
      auto * correcter_casted = dynamic_cast<PointCloudCorrecter*>(correcter);
      correcter_casted->correct(particles_, update_time);
    }
  }

  normalize();

  // Calculate quality
  quality_ = 0.0;
  for (auto & p : particles_) {
    if (p.possible_hits > 0.0) {
      quality_ = std::max(quality_, p.hits / p.possible_hits);
    }
  }

  if (quality_ > 0.0) {
    for (auto & p : particles_) {
      p.hits = 0.0;
      p.possible_hits = 0.0;
    }
  }
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

  int number_losers = std::max(1, static_cast<int>(number_particles * percentage_losers));
  int number_no_losers = std::max(1, static_cast<int>(number_particles - number_losers));
  int number_winners = std::max(1, static_cast<int>(number_particles * percentage_winners));


  std::vector<Particle> new_particles(particles_.begin(), particles_.begin() + number_no_losers);

  std::normal_distribution<double> selector(0, number_winners);
  std::normal_distribution<double> noise_x(0, init_error_x_ * init_error_x_);
  std::normal_distribution<double> noise_y(0, init_error_y_ * init_error_y_);
  std::normal_distribution<double> noise_yw(0, init_error_yaw_ * init_error_yaw_);
  std::normal_distribution<double> noise_p(0, init_error_pitch_ * init_error_pitch_);
  std::normal_distribution<double> noise_r(0, init_error_roll_ * init_error_roll_);

  for (int i = 0; i < number_losers; i++) {
    int index = std::clamp(static_cast<int>(selector(generator_)), 0, number_winners);

    Particle p;
    p.prob = new_particles.back().prob / 10.0;

    auto w_pose = particles_[i].pose.getOrigin();

    double nx = noise_x(generator_);
    double ny = noise_y(generator_);

    p.pose.setOrigin({w_pose.x() + nx, w_pose.y() + ny, w_pose.z()});

    double roll, pitch, yaw;

    tf2::Matrix3x3 m(particles_[i].pose.getRotation());
    m.getRPY(roll, pitch, yaw);

    double newyaw = yaw + noise_yw(generator_);
    while (newyaw > M_PI) {newyaw -= 2.0 * M_PI;}
    while (newyaw < -M_PI) {newyaw += 2.0 * M_PI;}
    double newpitch = pitch + noise_p(generator_);
    while (newpitch > M_PI) {newpitch -= 2.0 * M_PI;}
    while (newpitch < -M_PI) {newpitch += 2.0 * M_PI;}
    double newroll = roll + noise_r(generator_);
    while (newroll > M_PI) {newroll -= 2.0 * M_PI;}
    while (newroll < -M_PI) {newroll += 2.0 * M_PI;}

    tf2::Quaternion q;
    q.setRPY(newroll, newpitch, newyaw);

    p.pose.setRotation(q);

    new_particles.push_back(p);
  }

  particles_ = new_particles;
  normalize();

  update_pose(pose_);
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
