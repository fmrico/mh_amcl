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

#ifndef AAMCL_PARTICLESDISTRIBUTION_H
#define AAMCL_PARTICLESDISTRIBUTION_H

#include <vector>
#include <random>

#include "tf2/LinearMath/Transform.h"
#include "sensor_msgs/LaserScan.h"
#include "tf2_ros/transform_listener.h"

#include "geometry_msgs/TransformStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "std_msgs/ColorRGBA.h"

#include "costmap_2d/costmap_2d.h"

#include "ros/ros.h"

namespace aamcl
{

typedef struct
{
  tf2::Transform pose;
  double prob;
} Particle;

typedef enum TColor {
  RED, GREEN, BLUE, WHITE, GREY, DARK_GREY, BLACK, YELLOW, ORANGE, BROWN, PINK, 
  LIME_GREEN, PURPLE, CYAN, MAGENTA, NUM_COLORS} Color;

std_msgs::ColorRGBA 
getColor(Color color_id, double alpha = 1.0);

class ParticlesDistribution
{
public:
  ParticlesDistribution();

  void init();
  void init(const tf2::Transform & pose_init);
  void predict(const tf2::Transform & movement);
  void correct_once(const sensor_msgs::LaserScan & scan, const costmap_2d::Costmap2D & costmap);
  void reseed();

  void publish_particles(const std_msgs::ColorRGBA & color) const;

protected:  
  ros::NodeHandle nh_;
  ros::Publisher pub_particles_;

  tf2::Transform add_noise(const tf2::Transform & dm);
  tf2::Transform get_tranform_to_read(const sensor_msgs::LaserScan & scan, int index);
  double get_error_distance_to_obstacle(
    const tf2::Transform & map2bf, const tf2::Transform & bf2laser,  const tf2::Transform & laser2point,
    const sensor_msgs::LaserScan & scan, const costmap_2d::Costmap2D & costmap, double o);
  unsigned char get_cost( const tf2::Transform & transform, const costmap_2d::Costmap2D & costmap);
  void normalize();

  std::random_device rd_;
  std::mt19937 generator_;

  int NUM_PART;// = 200; Cambiado a parámetro
  double noise_x_param, noise_y_param, noise_t_param, translation_noise_param, rotation_noise_param,
  random_read_noise_param, percentage_losers, percentage_winners;
  std::vector<Particle> particles_;

  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;

  tf2::Stamped<tf2::Transform> bf2laser_;
  bool bf2laser_init_ {false};

};

}  // namespace aamcl

#endif  // AAMCL_PARTICLESDISTRIBUTION_H