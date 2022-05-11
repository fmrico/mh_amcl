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

#ifndef AAMCL_SAMPLELASERPERCEPTIONMODEL_H
#define AAMCL_SAMPLELASERPERCEPTIONMODEL_H

#include <vector>
#include <random>

#include "geometry_msgs/PoseArray.h"

#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"


#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Transform.h"
#include "costmap_2d/static_layer.h"


#include "aamcl/AAMCL.h"
#include "ros/ros.h"

namespace aamcl
{

class SampleLaserPerceptionModel
{
public:
  SampleLaserPerceptionModel();


protected:
  void publish_particles();
  void predict();
  void correct();
  void reseed();

  tf2::Transform add_noise(const tf2::Transform & dm);

  tf2::Transform extract_random_read_with_noise(sensor_msgs::LaserScan & scan, double noise);

private:
  ros::NodeHandle nh_;
  ros::Publisher pub_particles_;
  ros::Publisher laser_marker_pub;
  ros::Subscriber sub_map_;
  ros::Subscriber sub_lsr_;
  ros::Subscriber sub_init_pose_;

  static const int NUM_PART = 50;

  std::vector<Particle> particles_;
  bool particles_init;
  float y_max_, x_max_, y_min_, x_min_;

  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  tf2::Stamped<tf2::Transform> odom2prevbf_;
  bool valid_prev_odom2bf_ {false};
  costmap_2d::Costmap2D costmap_;

  sensor_msgs::LaserScan last_laser_;

  std::vector<tf2::Vector3> create_elements(const sensor_msgs::LaserScan & lsr_msg);
  void publish_marker(const std::list<tf2::Vector3> & readings);
  unsigned int interpretValue(unsigned char value);
  void map_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg);
  void laser_callback(const sensor_msgs::LaserScanConstPtr &lsr_msg);
  void initpose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &pose_msg);

  std::default_random_engine generator_;
  std::normal_distribution<double> translation_noise_;
  std::normal_distribution<double> rotation_noise_;

};

}  // namespace aamcl

#endif  // AAMCL_SAMPLELASERPERCEPTIONMODEL_H
