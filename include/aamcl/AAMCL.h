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

#ifndef AAMCL_AAMCL_H
#define AAMCL_AAMCL_H

#include <vector>

#include "geometry_msgs/PoseArray.h"

#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"


#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Transform.h"

#include "costmap_2d/static_layer.h"

#include "aamcl/ParticlesDistribution.h"
#include "ros/ros.h"

namespace aamcl
{

class AAMCL
{
public:
  AAMCL();

  void init();
  void step();

protected:
  void predict(const ros::TimerEvent & event);
  void correct(const ros::TimerEvent & event);
  void reseed(const ros::TimerEvent & event);
  void publish_particles(const ros::TimerEvent & event);

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_map_;
  ros::Subscriber sub_laser_;
  ros::Subscriber sub_init_pose_;

  ros::Timer predict_timer_;
  ros::Timer correct_timer_;
  ros::Timer reseed_timer_;
  ros::Timer publish_particles_timer_;


  ParticlesDistribution particles_;

  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;

  tf2::Stamped<tf2::Transform> odom2prevbf_;
  bool valid_prev_odom2bf_ {false};

  costmap_2d::Costmap2D costmap_;
  sensor_msgs::LaserScan last_laser_;

  unsigned int interpretValue(unsigned char value);
  void map_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg);

  void laser_callback(const sensor_msgs::LaserScanConstPtr &lsr_msg);
  void initpose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &pose_msg);
};

}  // namespace aamcl

#endif  // AAMCL_AAMCL_H
