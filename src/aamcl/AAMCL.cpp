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

#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/convert.h"
#include "geometry_msgs/Twist.h"
#include "costmap_2d/static_layer.h"

#include "costmap_2d/costmap_2d_publisher.h"
#include "costmap_2d/cost_values.h"

namespace aamcl
{

AAMCL::AAMCL()
: nh_(),
  buffer_(),
  listener_(buffer_),
  layered_costmap_()
    // costmap_("costmap",buffer_);
{
  pub_particles_ = nh_.advertise<geometry_msgs::PoseArray>("poses", 1000);
  sub_lsr_ = nh_.subscribe("scan_filtered", 100, &AAMCL::laser_callback, this);
   //  sub_map_ = nh_.subscribe("map", 100, &AAMCL::map_callback, this);
  sub_map_ = nh_.subscribe("initialpose", 100, &AAMCL::initpose_callback, this);

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
  particles_init = true;
  predict();
  correct();

  if (pub_particles_.getNumSubscribers() > 0)
  {
    publish_particles();
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

    pose_msg.orientation.x = rotation.x();
    pose_msg.orientation.y = rotation.y();
    pose_msg.orientation.z = rotation.z();
    pose_msg.orientation.w = rotation.w();

    msg.poses.push_back(pose_msg);
  }
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "map";


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
          particle.pose =  particle.pose * bfprev2bf;
        }
      }

      valid_prev_odom2bf_ = true;
      odom2prevbf_ = odom2bf;
  }
}

void
AAMCL::map_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{

}
void
AAMCL::laser_callback(const sensor_msgs::LaserScanConstPtr &lsr_msg)
{
  laser_elements_.resize(lsr_msg->ranges.size());
  for (int i = 0; i< lsr_msg->ranges.size(); i++)
  {
    float thetha = lsr_msg->angle_min + i*lsr_msg->angle_increment;
    Point p;
    p.x = lsr_msg->ranges.at(i) * cos(thetha);
    p.y = lsr_msg->ranges.at(i) * sin(thetha);
    laser_elements_.push_back(p);
  
  }
return;
}

void
AAMCL::correct()
{
  std::string error, source_frame = "scan_filtered", target_frame = "map";
  if(buffer_.canTransform(target_frame, source_frame, ros::Time(0), ros::Duration(0.1), &error))
  {
    geometry_msgs::TransformStamped laser2map_msg = buffer_.lookupTransform(target_frame, source_frame , ros::Time(0));
    tf2::Stamped<tf2::Transform> laser2map;
    tf2::fromMsg(laser2map_msg, laser2map);

    for (auto& part : particles_)
    {
      for (auto& point: laser_elements_ )
      {
        tf2::Vector3 map_point = laser2map * tf2::Vector3(point.x, point.y, 0);
        if (layered_costmap_.getCost(map_point.getX(), map_point.getY() == costmap_2d::FREE_SPACE))
        {
          part.prob +=0.1;
        }
        else if (layered_costmap_.getCost(map_point.getX(), map_point.getY() == costmap_2d::LETHAL_OBSTACLE))
        {
          part.prob -=0.1;
        }
      }
      
    }
  }
  else
  {
    std::cerr << "Error in tranformation from " << source_frame << " to " << target_frame << " : " << error;
  }
}

void 
AAMCL::initpose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &pose_msg)
{
  std::cerr << "1";
  if (pose_msg->header.frame_id == "map") {
  std::cerr << "2";
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
