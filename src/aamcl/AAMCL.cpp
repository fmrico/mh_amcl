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
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseArray.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"
#include "ros/ros.h"
#include <vector>
#include <math.h>
#include <time.h>
#include "tf2/LinearMath/Quaternion.h"
#include <costmap_2d/costmap_2d_ros.h>
#include <random>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_broadcaster.h>
#include "gazebo_msgs/ModelStates.h"

//sensors
#include <sensor_msgs/LaserScan.h>
*/
namespace aamcl
{

 typedef struct {
    geometry_msgs::Pose pose;
    float prob;
 } Particle;

 class AAMCL
  {


  public:
   AAMCL::AAMCL(): n_();vec_part_(NUM_PART)
    {   
    }
  }

}  // namespace aamcl
