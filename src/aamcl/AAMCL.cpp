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

#include "aamcl/AAMCL.h"
#include "ros/ros.h"
#include "random"
#include "cmath"

namespace aamcl
{

AAMCL::AAMCL()
: nh_()
{
  pub_particles_ = nh_.advertise<geometry_msgs::PoseArray>("poses", 1000);
  init();
}

void AAMCL::init()
{
  particles_.resize(NUM_PART);

  for (auto & particle : particles_)
  {
    particle.prob = 1.0 / NUM_PART;
    particle.pose.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
    particle.pose.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 0.0));
  }
}

void
AAMCL::step()
{
  ROS_INFO("Initializing...");
  init();
  particles_init = true;
  ROS_INFO_STREAM("Particles initialized: pos particle 1 x" << particles_.at(0).pose.getOrigin().getX());
  predict();
  ROS_INFO_STREAM("Done predicting , pos particle 1: " << particles_.at(0).pose.getOrigin().getX());
  correct();
  if (pub_particles_.getNumSubscribers() > 0)
  {
    publish_particles();
    ROS_INFO("Subscribers available");
  }
}

void
AAMCL::publish_particles()
{
  geometry_msgs::PoseArray msg;

  for (auto & particle : particles_)
  {
    geometry_msgs::Pose pose_msg;

    const auto translation = particle.pose.getOrigin();
    const auto rotation = particle.pose.getRotation();

    pose_msg.position.x = translation.x();
    pose_msg.position.y = translation.y();
    pose_msg.position.z = translation.z();

    pose_msg.position.x = translation.x();
    pose_msg.position.y = translation.y();
    pose_msg.position.z = translation.z();

    particle.prob = 1.0 / NUM_PART;
    particle.pose.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
    particle.pose.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 0.0));
    msg.poses.push_back(pose_msg);
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";
  }
  pub_particles_.publish(msg);
}
void
AAMCL::predict()
{
  sub_map_ = nh_.subscribe("map", 100, &AAMCL::mapcallback, this);  // ¿Por qué se pone el this?
  for (auto & particle : particles_)
  {
    float a_or = -M_PI , b_or = M_PI , b_pos_x , b_pos_y, a_pos_x, a_pos_y;
    if (!particles_init)
    {
     a_pos_x, a_pos_y = 0;
    }
    else
    {
     // predict particles using laser
     sub_lsr_ = nh_.subscribe("scan_filtered", 100, &AAMCL::lsrcallback, this);
     a_pos_x = x_min_;
     a_pos_y = y_min_;
    }

    b_pos_x = x_max_, b_pos_y = y_max_;
  // Generating a random orientation az
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine generator(seed);
  std::uniform_real_distribution <float> dist_or(a_or, b_or);
  float ax, ay = 0.0, az = dist_or(generator);

  // Converting to quaternion
  tf2::Quaternion q;
  q.setRPY(ax, ay, az);

  // Assigning to a particle
  particle.pose.setRotation(q);

  // Generating a random distance x position
  seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine generator2(seed);  // ¿Por qué no puedo utilizar el mismo generator?
  std::uniform_real_distribution <float> dist_pos_x(a_pos_x, b_pos_x);
  double x = dist_pos_x(generator2);

  // Generating a random distance y position
  std::default_random_engine generator3(seed);
  std::uniform_real_distribution <float> dist_pos_y(a_pos_y, b_pos_y);
  double y = dist_pos_y(generator3);

  // Assigning Origin to a particle
  particle.pose.setOrigin(tf2::Vector3(x, y, 0.0));

  // Calculating probability
  float probx = static_cast<float>(1)/ (b_pos_x - a_pos_x);
  float proby = static_cast<float>(1)/ (b_pos_y - a_pos_y);

  // Assigning probability to a particle
  particle.prob = probx * proby;
  }
  return;
}

void
AAMCL::mapcallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
  y_max_ = msg->info.height;
  x_max_ = msg->info.width;
  return;
}
void
AAMCL::lsrcallback(const sensor_msgs::LaserScanConstPtr &lsr_msg)
{
  for (int i = 0; i< lsr_msg->ranges.size(); i++)
  {
    float thetha = lsr_msg->angle_min + i*lsr_msg->angle_increment;
    float element_x = lsr_msg->ranges.at(i) * cos(thetha);
    float element_y = lsr_msg->ranges.at(i) * sin(thetha);

  if (i == 1)
  {
    x_min_, x_max_ = element_x;
    y_min_, y_max_ = element_y;
  }

  else
  {
     if ( element_x < x_min_)
     {
       x_min_ = element_x;
     }
     if ( element_x > x_max_)
     {
       x_max_ = element_x;
     }
     if ( element_y < y_min_ )
     {
       y_min_ = element_y;
     }
     if (element_y > y_max_)
     {
       y_max_ = element_y;
     }
  }  // else
  }  // for
return;
}

void
AAMCL::correct()
{
return;
}

}  // namespace aamcl
