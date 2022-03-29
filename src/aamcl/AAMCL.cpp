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

#include "aamcl/AAMCL.h"
#include "ros/ros.h"
#include "random"
#include "cmath"

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
namespace aamcl
{

AAMCL::AAMCL()
: nh_(),
  buffer_(),
  listener_(buffer_),
  costmap_()
    // costmap_("costmap",buffer_);
{
  pub_particles_ = nh_.advertise<geometry_msgs::PoseArray>("poses", 1000);
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

  std::cerr << "Correcting" << std::endl;
  auto laser_elements = create_elements(last_laser_);
  
  std::string error;
  std::string source_frame = last_laser_.header.frame_id;
  std::string target_frame = "base_footprint";

  if(buffer_.canTransform(target_frame, source_frame, last_laser_.header.stamp, ros::Duration(0.1), &error))
  {
    geometry_msgs::TransformStamped laser2bf_msg = buffer_.lookupTransform(target_frame, source_frame , last_laser_.header.stamp);
    tf2::Stamped<tf2::Transform> laser2bf;
    tf2::fromMsg(laser2bf_msg, laser2bf);

    for (auto& part : particles_)
    {

      std::list<tf2::Vector3> reading;

      for (auto& point: laser_elements)
      {
        reading.push_back(part.pose * (tf2::Transform(laser2bf) * point));

        //laser_marker_pub

      }


      for (auto & read: reading)
      {
        unsigned int mx, my;
        costmap_.worldToMap(read.getX(), read.getY(), mx, my);
         //  std::cerr << "Valor de X: " << read.getX() << " Valor de Y: " << read.getY() << std::endl;
         //  std::cerr << "Valor de mapx " << mx << " Valor de mapy " << my << std::endl;
         //  std::cerr << "Valor mínimo de x" << costmap_.getOriginX() << "Valor máximo de X " << costmap_.getSizeInMetersX() << std::endl;
         //  std::cerr << "Valor mínimo de y" << costmap_.getOriginY() << "Valor máximo de Y " << costmap_.getSizeInMetersY() << std::endl; 
        unsigned char cost = costmap_.getCost(mx, my);

         //  std::cerr << cost << " ";

        if (cost == costmap_2d::FREE_SPACE)
        {
           // std::cerr << "Probability incrementing , previous: " << part.prob;
           part.prob = std::clamp(part.prob + 0.1, 0.0, 1.0);
           // std::cerr << " updated: " << part.prob << std::endl;
        }
        else if (cost == costmap_2d::LETHAL_OBSTACLE)
        {
          // std::cerr << "Probability incrementing , previous: " << part.prob;          
          part.prob = std::clamp(part.prob - 0.1, 0.0, 1.0);
          // std::cerr << " updated: " << part.prob << std::endl;
        }
      }

      std::cerr << std::endl;

      publish_marker(reading);

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
