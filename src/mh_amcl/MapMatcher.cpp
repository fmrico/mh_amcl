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

#include "ceres/context.h"
#include "ceres/internal/disable_warnings.h"
#include "ceres/internal/port.h"
#include "ceres/types.h"

#include "pcl/kdtree/kdtree_flann.h"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

#include "mh_amcl/MapMatcher.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mh_amcl
{


MapMatcher::MapMatcher(const nav_msgs::msg::OccupancyGrid & map)
{
  costmaps_.resize(NUM_LEVEL_SCALE_COSTMAP);
  costmaps_[0] = std::make_shared<nav2_costmap_2d::Costmap2D>(map);

  for (int i = 1; i < NUM_LEVEL_SCALE_COSTMAP; i++) {
    costmaps_[i] = half_scale(costmaps_[i - 1]);
  }
}

std::shared_ptr<nav2_costmap_2d::Costmap2D>
MapMatcher::half_scale(std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap_in)
{
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2D>(
    costmap_in->getSizeInCellsX() / 2, costmap_in->getSizeInCellsY() / 2,
    costmap_in->getResolution() * 2.0, costmap_in->getOriginX(),
    costmap_in->getOriginY(), costmap_in->getDefaultValue());

  for (unsigned int i = 0; i < costmap->getSizeInCellsX(); i++) {
    for (unsigned int j = 0; j < costmap->getSizeInCellsY(); j++) {
      unsigned int ri = i * 2;
      unsigned int rj = j * 2;

      auto cost1 = costmap_in->getCost(ri, rj);
      auto cost2 = costmap_in->getCost(ri + 1, rj);
      auto cost3 = costmap_in->getCost(ri, rj + 1);
      auto cost4 = costmap_in->getCost(ri + 1, rj + 1);
      
      if (cost1 == nav2_costmap_2d::LETHAL_OBSTACLE ||
        cost2  == nav2_costmap_2d::LETHAL_OBSTACLE ||
        cost3  == nav2_costmap_2d::LETHAL_OBSTACLE ||
        cost4  == nav2_costmap_2d::LETHAL_OBSTACLE)
      {
        costmap->setCost(i, j, nav2_costmap_2d::LETHAL_OBSTACLE);
      } else if (cost1 == nav2_costmap_2d::FREE_SPACE ||
        cost2  == nav2_costmap_2d::FREE_SPACE ||
        cost3  == nav2_costmap_2d::FREE_SPACE ||
        cost4  == nav2_costmap_2d::FREE_SPACE)
      {
        costmap->setCost(i, j, nav2_costmap_2d::FREE_SPACE);
      } else if (cost1 == nav2_costmap_2d::NO_INFORMATION &&
        cost2  == nav2_costmap_2d::NO_INFORMATION &&
        cost3  == nav2_costmap_2d::NO_INFORMATION &&
        cost4  == nav2_costmap_2d::NO_INFORMATION)
      {
        costmap->setCost(i, j, nav2_costmap_2d::NO_INFORMATION);
      } else {
        costmap->setCost(i, j, cost1);
      }
    }
  }

  return costmap;
}

std::list<TransformWeighted>
MapMatcher::get_matchs(const sensor_msgs::msg::LaserScan & scan)
{
  std::vector<tf2::Vector3> laser_poins = laser2points(scan);

  int start_level = NUM_LEVEL_SCALE_COSTMAP - 2;
  int min_level = 2;

  double min_x, max_x, min_y, max_y;
  costmaps_[start_level]->mapToWorld(0, 0, min_x, min_y);
  costmaps_[start_level]->mapToWorld(costmaps_[start_level]->getSizeInCellsX(), costmaps_[start_level]->getSizeInCellsY(), max_x, max_y);

  std::list<TransformWeighted> candidates[NUM_LEVEL_SCALE_COSTMAP];
  candidates[start_level] = get_matchs(start_level, laser_poins, min_x, min_y, max_x, max_y);
  candidates[start_level].sort();
  
  for (int level = start_level; level >= min_level; level--) {

    // std::cerr << "Level " << level << "\tcandidates: " << candidates[level].size() << "\tResolution: " << costmaps_[level]->getResolution() << std::endl;
    for (const auto & candidate : candidates[level]) {
      double min_x, max_x, min_y, max_y;
      min_x = candidate.transform.getOrigin().x() - costmaps_[level]->getResolution() / 2.0;
      max_x = candidate.transform.getOrigin().x() + costmaps_[level]->getResolution() / 2.0;
      min_y = candidate.transform.getOrigin().y() - costmaps_[level]->getResolution() / 2.0;
      max_y = candidate.transform.getOrigin().y() + costmaps_[level]->getResolution() / 2.0;

      auto new_candidates = get_matchs(level - 1, laser_poins, min_x, min_y, max_x, max_y);

      // std::cerr << "\tAdding " << new_candidates.size() << " candidates in ([" <<  min_x << ", " << max_x << "] , ["  <<
      //   min_y << ", " << max_y << "])" << std::endl;
      candidates[level - 1].insert(candidates[level - 1].end(), new_candidates.begin(), new_candidates.end()); 
    }
  }
  
  candidates[min_level].sort();

  return candidates[min_level];
}

std::list<TransformWeighted>
MapMatcher::get_matchs(
  int scale, const std::vector<tf2::Vector3> & scan,
  float min_x, float min_y, float max_y, float max_x)
{
  std::list<TransformWeighted> ret;
  const auto & costmap = costmaps_[scale];

  int init_i, init_j, end_i, end_j;
  costmap->worldToMapEnforceBounds(min_x, min_y, init_i, init_j);
  costmap->worldToMapEnforceBounds(max_x, max_y, end_i, end_j);

  for (unsigned int i = init_i; i < end_i; i++) {
    for (unsigned int j = init_j; j < end_j; j++) {
      auto cost = costmap->getCost(i, j);
      if (cost == nav2_costmap_2d::FREE_SPACE) {
        double inc_theta = (M_PI / 4.0);
        for (double theta = 0; theta < 1.9 * M_PI; theta = theta + inc_theta) {
          double x, y;
          costmap->mapToWorld(i, j, x, y);
          TransformWeighted tw;

          tf2::Quaternion q;
          q.setRPY(0.0, 0.0, theta);
          tw.transform = tf2::Transform(q, {x, y, 0.0});

          tw.weight = match(scale, *costmap, scan, tw.transform);
          
          if (tw.weight > 0.5) {
            ret.push_back(tw);
          }
        }
      }
    }
  }
  return ret;
}

float
MapMatcher::match(int scale, const nav2_costmap_2d::Costmap2D & costmap,
  const std::vector<tf2::Vector3> & scan, tf2::Transform & transform)
{
  int hits = 0;
  int total = 0;

  for (int i = 0; i < scan.size(); i = i + scale) {
    tf2::Vector3 test_point = transform * scan[i];
    int gi, gj;
    
    costmap.worldToMapNoBounds(test_point.x(), test_point.y(), gi, gj);


    if (gi > 0 && gj > 0 && gi < costmap.getSizeInCellsX() && gj < costmap.getSizeInCellsY() &&
      costmap.getCost(gi, gj) == nav2_costmap_2d::LETHAL_OBSTACLE)
    {
      // if (transform.getOrigin().x() < -5.0 && transform.getOrigin().x() > -6.0  &&
      //   transform.getOrigin().y() < -4.5 && transform.getOrigin().y() > -5.0) {
      //   std::cerr << "Hit in (" << test_point.x() << ", " << test_point.y() << ")" << 
      //   "{" << gi << ", " << gj << "}" << "[" <<  costmap.getSizeInCellsX() << " - " << costmap.getSizeInCellsY() << "]" <<
      //   std::endl;
      // }

      hits++;
    }
    total++;
  }

  return static_cast<float>(hits) / static_cast<float>(total);
}

std::vector<tf2::Vector3>
MapMatcher::laser2points(const sensor_msgs::msg::LaserScan & scan)
{
  std::list<tf2::Vector3> points;  
  for (auto i = 0; i < scan.ranges.size(); i++) {
    if (std::isnan(scan.ranges[i]) || std::isinf(scan.ranges[i])) {continue;}
    
    tf2::Vector3 p;
    float dist = scan.ranges[i];
    float theta = scan.angle_min + i * scan.angle_increment;
    p.setX(dist * cos(theta));
    p.setY(dist * sin(theta));
    p.setZ(0.0);
    points.push_back(p);
  }
  return std::vector<tf2::Vector3>(points.begin(), points.end());
}

nav_msgs::msg::OccupancyGrid
toMsg(const nav2_costmap_2d::Costmap2D & costmap)
{
  nav_msgs::msg::OccupancyGrid grid;

  grid.info.resolution = costmap.getResolution();
  grid.info.width = costmap.getSizeInCellsX();
  grid.info.height = costmap.getSizeInCellsY();

  double wx, wy;
  costmap.mapToWorld(0, 0, wx, wy);
  grid.info.origin.position.x = wx -  costmap.getResolution() / 2;
  grid.info.origin.position.y = wy -  costmap.getResolution() / 2;
  grid.info.origin.position.z = 0.0;
  grid.info.origin.orientation.w = 1.0;

  grid.data.resize(grid.info.width * grid.info.height);

  std::vector<char> cost_translation_table(256);

  // special values:
  cost_translation_table[0] = 0;  // NO obstacle
  cost_translation_table[253] = 99;  // INSCRIBED obstacle
  cost_translation_table[254] = 100;  // LETHAL obstacle
  cost_translation_table[255] = -1;  // UNKNOWN

  // regular cost values scale the range 1 to 252 (inclusive) to fit
  // into 1 to 98 (inclusive).
  for (int i = 1; i < 253; i++) {
    cost_translation_table[i] = static_cast<char>(1 + (97 * (i - 1)) / 251);
  }

  unsigned char * data = costmap.getCharMap();
  for (unsigned int i = 0; i < grid.data.size(); i++) {
    grid.data[i] = cost_translation_table[data[i]];
  }

  return grid;
}

bool operator<(const TransformWeighted & tw1, const TransformWeighted & tw2)
{
  // To sort incremental
  return tw1.weight > tw2.weight;
}

}  // namespace mh_amcl
