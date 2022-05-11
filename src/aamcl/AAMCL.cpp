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
#include <random>
#include <cmath>

#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "visualization_msgs/MarkerArray.h"
#include "tf2/convert.h"
#include "geometry_msgs/Twist.h"
#include "costmap_2d/static_layer.h"

#include "costmap_2d/costmap_2d_publisher.h"
#include "costmap_2d/cost_values.h"

#include "aamcl/AAMCL.h"
#include "ros/ros.h"


namespace aamcl
{

AAMCL::AAMCL()
: nh_(),
  buffer_(),
  listener_(buffer_),
  costmap_(),
  translation_noise_(0.0, 0.1),
  rotation_noise_(0.0, 0.1)

    // costmap_("costmap",buffer_);
{
  pub_particles_ = nh_.advertise<visualization_msgs::MarkerArray>("poses", 1000);
  laser_marker_pub = nh_.advertise<visualization_msgs::MarkerArray>("laser_marker", 1000);
  sub_lsr_ = nh_.subscribe("scan_filtered", 100, &AAMCL::laser_callback, this);
  sub_map_ = nh_.subscribe("map", 100, &AAMCL::map_callback, this);

  sub_init_pose_ = nh_.subscribe("initialpose", 100, &AAMCL::initpose_callback, this);

  init();
}

void AAMCL::init()
{
  particles_.resize(NUM_PART);

  for (auto & particle : particles_)
  {
    particle.prob = 1.0 / NUM_PART;
    particle.pose.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
    particle.pose.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));
  }
}

void
AAMCL::step()
{
  std::cerr << "=====================================================" << std::endl;
  particles_init = true;
  predict();
  correct();
  reseed();

  for (int i = 0; i < NUM_PART; i++) {
    double roll, pitch, yaw;
    tf2::Matrix3x3(particles_[i].pose.getRotation()).getRPY(roll, pitch, yaw);

    std::cerr << "(" << particles_[i].pose.getOrigin().x() << ", " << particles_[i].pose.getOrigin().y() << 
      ", " << yaw << ") ["<< particles_[i].prob << "]" << std::endl;
  }

  if (pub_particles_.getNumSubscribers() > 0)
  {
    publish_particles();
  }
}

void
AAMCL::publish_particles()
{
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

    pose_msg.color.r = (1.0 - particle.prob);
    pose_msg.color.g = particle.prob;
    pose_msg.color.b = 0.0;
    pose_msg.color.a = 1.0;

    msg.markers.push_back(pose_msg);
  }

  pub_particles_.publish(msg);
}

void
AAMCL::predict()
{
  geometry_msgs::TransformStamped odom2bf_msg;
  std::string error;
  if (buffer_.canTransform("odom", "base_footprint", ros::Time(0), ros::Duration(0.1), &error)) //  ¿No sería bf2odom?
  {
      odom2bf_msg = buffer_.lookupTransform("odom", "base_footprint", ros::Time(0));

      tf2::Stamped<tf2::Transform> odom2bf;
      tf2::fromMsg(odom2bf_msg, odom2bf);

      if (valid_prev_odom2bf_) {
        tf2::Transform bfprev2bf = odom2prevbf_.inverse() * odom2bf;

        for (auto & particle : particles_) {
          particle.pose =  particle.pose * bfprev2bf * add_noise(bfprev2bf);
        }
      }

      valid_prev_odom2bf_ = true;
      odom2prevbf_ = odom2bf;
  }
}

tf2::Transform
AAMCL::add_noise(const tf2::Transform & dm)
{
  tf2::Transform returned_noise;

  double noise_tra = translation_noise_(generator_);
  double noise_rot = rotation_noise_(generator_);


  returned_noise.setOrigin(dm.getOrigin()* noise_tra);

  double roll, pitch, yaw;
  tf2::Matrix3x3(dm.getRotation()).getRPY(roll, pitch, yaw);

  double newyaw = yaw * noise_rot;

  tf2::Quaternion q;
  q.setRPY(roll, pitch, newyaw);
  returned_noise.setRotation(q);


  // std::cerr << "[" << noise_tra << "," << noise_rot << "] (" << 
  //   dm.getOrigin().x()  << "," << dm.getOrigin().y()  << "," <<yaw << ")  -> (" << 
  //   returned_noise.getOrigin().x()  << "," << returned_noise.getOrigin().y()  << "," << newyaw << ")" << std::endl; 

  return returned_noise;
}

void
AAMCL::map_callback(const nav_msgs::OccupancyGrid::ConstPtr & msg)
{
  unsigned int size_x = msg->info.width, size_y = msg->info.height;
 
  ROS_DEBUG("Received a %d X %d map at %f m/pix", size_x, size_y, msg->info.resolution);
 
  costmap_.resizeMap(size_x, size_y, msg->info.resolution, msg->info.origin.position.x,
                               msg->info.origin.position.y);
   
 
   unsigned int index = 0;
 
   // initialize the costmap with static data
   for (unsigned int i = 0; i < size_y; ++i)
   {
     for (unsigned int j = 0; j < size_x; ++j)
     {
       unsigned char value = msg->data.at(index);
       unsigned int x , y;
       costmap_.indexToCells(index,x,y);
       costmap_.setCost(x, y, interpretValue(value));
        //  std::cerr << "Index: " << index << " X: " << x << " Y: " << y << " Cost: " << interpretValue(value) << std::endl;
       ++index;
     }
   }
}

unsigned int 
AAMCL::interpretValue(unsigned char value)
{
  // check if the static value is above the unknown or lethal thresholds
  if (value == -1)
    return costmap_2d::NO_INFORMATION; // Depending if track unknown space or not
  else if (value >= 70)
    return costmap_2d::LETHAL_OBSTACLE;
  else if (value <= 10)
    return costmap_2d::FREE_SPACE;

  double scale = (double) value / 254;
  return scale * costmap_2d::LETHAL_OBSTACLE;
}

void
AAMCL::laser_callback(const sensor_msgs::LaserScanConstPtr & lsr_msg)
{
  last_laser_ = *lsr_msg;
}

std::vector<tf2::Vector3>
AAMCL::create_elements(const sensor_msgs::LaserScan & lsr_msg)
{
  std::vector<tf2::Vector3> laser_elements;

  laser_elements.resize(lsr_msg.ranges.size());
  for (int i = 0; i< lsr_msg.ranges.size(); i++)
  {
    float thetha = lsr_msg.angle_min + i * lsr_msg.angle_increment;

    laser_elements.push_back({
        lsr_msg.ranges.at(i) * cos(thetha),
        lsr_msg.ranges.at(i) * sin(thetha),
        0.0});
  }

  return laser_elements;
}

void
AAMCL::publish_marker(const std::list<tf2::Vector3> & readings)
{
  visualization_msgs::MarkerArray marker_array;
  int counter = 0;

  for (const auto & point : readings) {
    visualization_msgs::Marker marker;

    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = "map";

    marker.ns = "default";
    marker.id = counter++;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = point.x();
    marker.pose.position.y = point.y();
    marker.pose.position.z = point.z();
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    marker_array.markers.push_back(marker);
  }

  laser_marker_pub.publish(marker_array);
}
void
AAMCL::correct()
{
  if (last_laser_.ranges.empty()) return;

  tf2::Stamped<tf2::Transform> bf2laser;
  std::string error;
  if (buffer_.canTransform(
    last_laser_.header.frame_id, "base_footprint", last_laser_.header.stamp), ros::Duration(0.1), &error)
  {
    auto bf2laser_msg = buffer_.lookupTransform(
      "base_footprint", last_laser_.header.frame_id, last_laser_.header.stamp);
    tf2::fromMsg(bf2laser_msg, bf2laser);
    
  } else {
    ROS_WARN("Timeout while waiting TF %s -> base_footprint [%s]",
      last_laser_.header.frame_id.c_str(), error.c_str());
    return;
  }
  
  double noise_dist = 0.001;
  int max_tests = 30;
  int test_counter = 0;
  while (!last_laser_.ranges.empty() && test_counter++ < max_tests)
  {
    tf2::Transform laser2point = extract_random_read_with_noise(last_laser_, noise_dist);

    for (auto & p : particles_)
    {
      auto map2point = p.pose * bf2laser * laser2point;
      
      unsigned int mx, my;
      if (costmap_.worldToMap(map2point.getOrigin().x(), map2point.getOrigin().y(), mx, my)) {
        auto cost = costmap_.getCost(mx, my);

        if (cost == costmap_2d::LETHAL_OBSTACLE) {
          p.prob = std::clamp(p.prob + (1.0 / last_laser_.ranges.size()), 0.0, 1.0);
        } else {
          p.prob = std::clamp(p.prob - (1.0 / last_laser_.ranges.size()), 0.0, 1.0);
        }
      }
    }
  }

}

void
AAMCL::reseed()
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
  
  std::uniform_int_distribution<int> selector(0, number_winners);
  std::normal_distribution<double> noise_x(0, 0.01);
  std::normal_distribution<double> noise_y(0, 0.01);

  for (int i = 0; i < number_losers; i++)
  {
    int index = selector(generator_);

    Particle p;
    p.prob = new_particles.back().prob;

    auto w_pose = particles_[i].pose.getOrigin();

    double nx = noise_x(generator_);
    double ny = noise_y(generator_);

    p.pose.setOrigin({w_pose.x() + nx, w_pose.y() + ny, w_pose.z()});
    p.pose.setRotation(particles_[i].pose.getRotation());

    new_particles.push_back(p);
  }

  particles_ = new_particles;
  
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
}

tf2::Transform 
AAMCL::extract_random_read_with_noise(sensor_msgs::LaserScan & scan, double noise)
{
  std::uniform_int_distribution<int> selector(0, scan.ranges.size() - 1);
  std::normal_distribution<double> dist_noise(0.0, noise);
  
  int index = selector(generator_);

  double dist = scan.ranges[index];
  double dist_with_noise = dist + dist_noise(generator_);

  double angle = scan.angle_min + index * scan.angle_increment;

  tf2::Transform ret;
  double x = dist_with_noise * cos(angle);
  double y = dist_with_noise * sin(angle);

  ret.setOrigin({x, y, 0.0});
  ret.setRotation({0.0, 0.0, 0.0, 1.0});

  // Remove used scan reading
  scan.ranges.erase(scan.ranges.begin() + index);

  return ret;
}


void 
AAMCL::initpose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &pose_msg)
{
  if (pose_msg->header.frame_id == "map") {
    for (auto & particle : particles_)
    {
      particle.pose.setOrigin(tf2::Vector3(
        pose_msg->pose.pose.position.x,
        pose_msg->pose.pose.position.y,
        pose_msg->pose.pose.position.z));
      
      particle.pose.setRotation({
        pose_msg->pose.pose.orientation.x,
        pose_msg->pose.pose.orientation.y,
        pose_msg->pose.pose.orientation.z,
        pose_msg->pose.pose.orientation.w});
    } 
  } else {
    ROS_WARN("Not possible to init particles in frame %s", pose_msg->header.frame_id.c_str());
  }
}

}  // namespace aamcl
