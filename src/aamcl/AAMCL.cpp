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
  costmap_()
{
  
  sub_laser_ = nh_.subscribe("scan_filtered", 100, &AAMCL::laser_callback, this);
  sub_map_ = nh_.subscribe("map", 100, &AAMCL::map_callback, this);
  sub_init_pose_ = nh_.subscribe("initialpose", 100, &AAMCL::initpose_callback, this);

  predict_timer_ = nh_.createTimer(ros::Duration(0.05), &AAMCL::predict, this);
  correct_timer_ = nh_.createTimer(ros::Duration(0.05), &AAMCL::correct, this);
  reseed_timer_ = nh_.createTimer(ros::Duration(1), &AAMCL::reseed, this);
  publish_particles_timer_ = nh_.createTimer(ros::Duration(1), &AAMCL::publish_particles, this);

  init();
}

void AAMCL::init()
{
  particles_.init();
}

void
AAMCL::publish_particles(const ros::TimerEvent & event)
{
  auto start = ros::Time::now();
  (void)event;

  particles_.publish_particles();
  ROS_DEBUG_STREAM("Publish [" << (ros::Time::now() - start).toNSec() << " nsecs]");
}

void
AAMCL::predict(const ros::TimerEvent & event)
{
  auto start = ros::Time::now();
  (void)event;

  geometry_msgs::TransformStamped odom2bf_msg;
  std::string error;
  if (buffer_.canTransform("odom", "base_footprint", ros::Time(0), ros::Duration(0.1), &error)) //  ¿No sería bf2odom?
  {
      odom2bf_msg = buffer_.lookupTransform("odom", "base_footprint", ros::Time(0));

      tf2::Stamped<tf2::Transform> odom2bf;
      tf2::fromMsg(odom2bf_msg, odom2bf);

      if (valid_prev_odom2bf_) {
        tf2::Transform bfprev2bf = odom2prevbf_.inverse() * odom2bf;

        particles_.predict(bfprev2bf);
      }

      valid_prev_odom2bf_ = true;
      odom2prevbf_ = odom2bf;
  }

  ROS_DEBUG_STREAM("Predict [" << (ros::Time::now() - start).toNSec() << " nsecs]");
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
  counter_ = 0;
}

void
AAMCL::correct(const ros::TimerEvent & event)
{ 
  // std::cerr << counter_++ << " ";
  auto start = ros::Time::now();
  (void)event;

  if (last_laser_.ranges.empty()) return;

  particles_.correct_once(last_laser_, costmap_);
  ROS_DEBUG_STREAM("Correct [" << (ros::Time::now() - start).toNSec() << " nsecs]");
}

void
AAMCL::reseed(const ros::TimerEvent & event)
{
  auto start = ros::Time::now();
  (void)event;

  particles_.reseed();
  ROS_DEBUG_STREAM("Reseed [" << (ros::Time::now() - start).toNSec() << " nsecs]");
}

void 
AAMCL::initpose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &pose_msg)
{
  if (pose_msg->header.frame_id == "map") {

    tf2::Transform pose;
    pose.setOrigin({
      pose_msg->pose.pose.position.x,
      pose_msg->pose.pose.position.y,
      pose_msg->pose.pose.position.z});
      
    pose.setRotation({
      pose_msg->pose.pose.orientation.x,
      pose_msg->pose.pose.orientation.y,
      pose_msg->pose.pose.orientation.z,
      pose_msg->pose.pose.orientation.w});

    particles_.init(pose);
  } else {
    ROS_WARN("Not possible to init particles in frame %s", pose_msg->header.frame_id.c_str());
  }
}

}  // namespace aamcl
