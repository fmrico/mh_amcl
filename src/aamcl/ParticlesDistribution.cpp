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

#include <random>
#include <cmath>

#include "visualization_msgs/MarkerArray.h"
#include "sensor_msgs/LaserScan.h"

#include "costmap_2d/costmap_2d.h"
#include "costmap_2d/cost_values.h"

#include "aamcl/ParticlesDistribution.h"

#include "ros/ros.h"

namespace aamcl
{

ParticlesDistribution::ParticlesDistribution()
: nh_(),
  buffer_(),
  listener_(buffer_),
  rd_(),
  generator_(rd_())
{
  pub_particles_ = nh_.advertise<visualization_msgs::MarkerArray>("poses", 1000);
}

void 
ParticlesDistribution::init()
{
  tf2::Transform init_pose;
  init_pose.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
  init_pose.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));

  init(init_pose);
}

void
ParticlesDistribution::init(const tf2::Transform & pose_init)
{
  std::normal_distribution<double> noise_x(0, 0.1);
  std::normal_distribution<double> noise_y(0, 0.1);
  std::normal_distribution<double> noise_t(0, 0.05);

  particles_.resize(NUM_PART);

  for (auto & particle : particles_)
  {
    particle.prob = 1.0 / NUM_PART;
    particle.pose = pose_init;

    tf2::Vector3 pose = particle.pose.getOrigin();
    pose.setX(pose.getX() + noise_x(generator_));
    pose.setY(pose.getY() + noise_y(generator_));

    particle.pose.setOrigin(pose);

    double roll, pitch, yaw;
    tf2::Matrix3x3(particle.pose.getRotation()).getRPY(roll, pitch, yaw);

    double newyaw = yaw + noise_t(generator_);

    tf2::Quaternion q;
    q.setRPY(roll, pitch, newyaw);
    
    particle.pose.setRotation(q);
  }
}

void
ParticlesDistribution::predict(const tf2::Transform & movement)
{
  for (auto & particle : particles_)
  {
    // const auto x1 = particle.pose.getOrigin().x();
    // const auto y1 = particle.pose.getOrigin().y();
    // std::cerr << "[1] " << x1 << ", " << y1 << std::endl;

    particle.pose =  particle.pose * movement * add_noise(movement);

    // const auto x2 = particle.pose.getOrigin().x();
    // const auto y2 = particle.pose.getOrigin().y();
    // std::cerr << "[2] " << x2 << ", " << y2 << std::endl;
  }
}

tf2::Transform
ParticlesDistribution::add_noise(const tf2::Transform & dm)
{
  tf2::Transform returned_noise;

  std::normal_distribution<double> translation_noise_(0.0, 0.01);
  std::normal_distribution<double> rotation_noise_(0.0, 0.01);

  double noise_tra = translation_noise_(generator_);
  double noise_rot = rotation_noise_(generator_);

  double x = dm.getOrigin().x() * noise_tra;
  double y = dm.getOrigin().y() * noise_tra;
  double z = 0.0;

  // std::cerr << "[noise X] " << x << " = " << dm.getOrigin().x() << " * " << noise_tra << std::endl;
  // std::cerr << "[noise Y] " << y << " = " << dm.getOrigin().y() << " * " << noise_tra << std::endl;

  returned_noise.setOrigin(tf2::Vector3(x, y, z));

  double roll, pitch, yaw;
  tf2::Matrix3x3(dm.getRotation()).getRPY(roll, pitch, yaw);

  double newyaw = yaw + noise_rot;

  tf2::Quaternion q;
  q.setRPY(roll, pitch, newyaw);
  returned_noise.setRotation(q);

  return returned_noise;
}

void
ParticlesDistribution::publish_particles(const std_msgs::ColorRGBA & color) const
{
  if (pub_particles_.getNumSubscribers() == 0)
  {
    return;
  }

  visualization_msgs::MarkerArray msg;

  int counter = 0;
  for (auto & particle : particles_)
  {
    visualization_msgs::Marker pose_msg;

    pose_msg.header.frame_id = "map";
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.id = counter++;
    pose_msg.type = visualization_msgs::Marker::ARROW;
    pose_msg.type = visualization_msgs::Marker::ADD;    

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

  pub_particles_.publish(msg);
}

void
ParticlesDistribution::correct_once(const sensor_msgs::LaserScan & scan, const costmap_2d::Costmap2D & costmap)
{
  std::string error;
  if (buffer_.canTransform(
    scan.header.frame_id, "base_footprint", scan.header.stamp), ros::Duration(0.1), &error)
  {
    auto bf2laser_msg = buffer_.lookupTransform(
      "base_footprint", scan.header.frame_id, scan.header.stamp);
    tf2::fromMsg(bf2laser_msg, bf2laser_);
  } else {
    ROS_WARN("Timeout while waiting TF %s -> base_footprint [%s]",
      scan.header.frame_id.c_str(), error.c_str());
    return;
  }

  for (int j = 0; j < scan.ranges.size(); j++)
  {
    if (std::isnan(scan.ranges[j])) continue;
    
    tf2::Transform laser2point;
    bool is_inf = std::isinf(scan.ranges[j]);
    if (!is_inf) laser2point = get_tranform_to_read(scan, j);
    tf2::Transform laser2point_uvectortf = get_tranform_to_read_vector(scan, j);

    for (int i = 0; i < NUM_PART; i++)
    {
      auto & p = particles_[i];

      double calculated_distance = get_distance_to_obstacle(
        p.pose, bf2laser_, laser2point_uvectortf, scan, costmap);
      double measured_distance = (bf2laser_ * laser2point).getOrigin().length();

      
    }
  }


}

tf2::Transform 
ParticlesDistribution::get_tranform_to_read(const sensor_msgs::LaserScan & scan, int index)
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

tf2::Transform 
ParticlesDistribution::get_tranform_to_read_vector(const sensor_msgs::LaserScan & scan, int index, double module)
{
  double dist = module;
  double angle = scan.angle_min + static_cast<double>(index) * scan.angle_increment;

  tf2::Transform ret;
  
  double x = dist * cos(angle);
  double y = dist * sin(angle);

  ret.setOrigin({x, y, 0.0});
  ret.setRotation({0.0, 0.0, 0.0, 1.0});

  return ret;
}

double
ParticlesDistribution::get_distance_to_obstacle(
  const tf2::Transform & map2bf, const tf2::Transform & bf2laser,  const tf2::Transform & uvector,
  const sensor_msgs::LaserScan & scan, const costmap_2d::Costmap2D & costmap)
{
  // std::cerr << "==================================================" << std::endl;
  bool obstacle_found = false;
  double dist = scan.range_min;

  tf2::Transform laser2point = uvector;
  tf2::Transform map2laser = map2bf * bf2laser;

  while (dist < scan.range_max && !obstacle_found)
  {
    laser2point.setOrigin(uvector.getOrigin() * dist);

    tf2::Transform map2point = map2laser * laser2point;

    unsigned int mx, my;
    if (costmap.worldToMap(map2point.getOrigin().x(), map2point.getOrigin().y(), mx, my)) {
      auto cost = costmap.getCost(mx, my);
      obstacle_found = costmap.getCost(mx, my) == costmap_2d::LETHAL_OBSTACLE;

      // std::cerr << "(" << map2point.getOrigin().x() << ", " << map2point.getOrigin().y() << ")[" << 
      //   static_cast<int>(costmap.getCost(mx, my)) << "] ";

    } else {
      // std::cerr << "INFINITE!! " << std::endl;
      return std::numeric_limits<float>::infinity();
    }
    if (!obstacle_found) dist = dist + costmap.getResolution();
  }

  if (obstacle_found)
  {
    // std::cerr << "DIST =  " << dist << std::endl;
    return dist;
  }
  else
  {
    // std::cerr << "INFINITE " << std::endl;
    return std::numeric_limits<float>::infinity();
  }
}

void
ParticlesDistribution::reseed()
{
  // Sort particles by prob
  std::sort(particles_.begin(), particles_.end(),
   [](const Particle & a, const Particle & b) -> bool
  { 
    return a.prob > b.prob; 
  });

  double percentage_losers = 0.3;
  double percentage_winners = 0.1;

  int number_losers = particles_.size() * percentage_losers;
  int number_no_losers = particles_.size() - number_losers;
  int number_winners = particles_.size() * percentage_winners;

  std::vector<Particle> new_particles(particles_.begin(), particles_.begin() + number_no_losers);
  
  std::normal_distribution<double> selector(0, number_winners);
  std::normal_distribution<double> noise_x(0, 0.02);
  std::normal_distribution<double> noise_y(0, 0.02);
  std::normal_distribution<double> noise_t(0, 0.01);

  for (int i = 0; i < number_losers; i++)
  {
    int index = std::clamp(static_cast<int>(selector(generator_)), 0, number_winners);

    Particle p;
    p.prob = new_particles.back().prob;

    auto w_pose = particles_[i].pose.getOrigin();

    double nx = noise_x(generator_);
    double ny = noise_y(generator_);

    p.pose.setOrigin({w_pose.x() + nx, w_pose.y() + ny, w_pose.z()});
    
    double roll, pitch, yaw;

    tf2::Matrix3x3(particles_[i].pose.getRotation()).getRPY(roll, pitch, yaw);

    double newyaw = yaw + noise_t(generator_);

    tf2::Quaternion q;
    q.setRPY(roll, pitch, newyaw);
    
    p.pose.setRotation(q);

    new_particles.push_back(p);
  }

  particles_ = new_particles;
  
  /*
  // Normalize the distribution ton sum 1.0
  double sum = 0.0;
  std::for_each(particles_.begin(), particles_.end(),
    [&sum] (const Particle & p)
    {
      sum += p.prob;
    });
  
  double norm_factor = 1.0;
  if (sum > 0.0) {
    norm_factor = 1.0 / sum;
  }

  std::for_each(particles_.begin(), particles_.end(),
    [norm_factor] (Particle & p)
    {
      p.prob = p.prob * norm_factor;
    });
  */
}

std_msgs::ColorRGBA 
getColor(Color color_id, double alpha)
{
  std_msgs::ColorRGBA color;

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

}  // namespace aamcl
