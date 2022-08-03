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

Eigen::Vector2d point2eigen(PointType p)
{
    Eigen::Vector2d pp;
    pp(0) = p.x;
    pp(1) = p.y;
    return pp;
}

PointType eigen2point(Eigen::Vector2d pp)
{
    PointType p;
    p.x = pp(0);
    p.y = pp(1);
    return p;
}

MapMatcher::MapMatcher()
{
}

pcl::PointCloud<PointType>
MapMatcher::PCfromScan(const sensor_msgs::msg::LaserScan & scan)
{
  pcl::PointCloud<PointType> ret;

  ret.points.resize(scan.ranges.size());
  for (auto i = 0; i < scan.ranges.size(); i++) {
    float dist = scan.ranges[i];
    float theta = scan.angle_min + i * scan.angle_increment;
    ret.points[i].x = dist * cos(theta);
    ret.points[i].y = dist * sin(theta);
  }
  
  ret.width = ret.points.size();
  ret.height = 1;
  ret.is_dense = true;

  return ret;
}

pcl::PointCloud<PointType>
MapMatcher::PCfromGrid(const nav_msgs::msg::OccupancyGrid & grid)
{
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2D>(grid);
  pcl::PointCloud<PointType> ret;

  for (auto i = 0; i < costmap->getSizeInCellsX(); i++) {
    for (auto j = 0; j < costmap->getSizeInCellsY(); j++) {
      if (costmap->getCost(i, j) == nav2_costmap_2d::LETHAL_OBSTACLE) {
        double x, y;
        costmap->mapToWorld(i, j, x, y);
        
        PointType p;
        p.x = x;
        p.y = y;

        ret.points.push_back(p);
      }
    }
  }

  ret.width = ret.points.size();
  ret.height = 1;
  ret.is_dense = true;

  return ret;
}

state2d
MapMatcher::pc_match(const pcl::PointCloud<PointType> & pc1, const pcl::PointCloud<PointType> & pc2, state2d state)
{
  state2d ret = state;

  double pose[3] = {state.t(0), state.t(1), state.theta};
  if (pc1.points.size() && pc2.points.size()) {
    ceres::Problem problem;
    
    //solve delta with ceres constraints
    pcl::KdTreeFLANN<PointType> kdtree;
    kdtree.setInputCloud(pc1.makeShared());
    int K = 2; // K nearest neighbor search
    std::vector<int> index(K);
    std::vector<float> distance(K);
    
    //1. project scan_prev to scan
    Eigen::Matrix2d R;
    R(0, 0) = cos(ret.theta); R(0, 1) = -sin(ret.theta);
    R(1, 0) = sin(ret.theta); R(1, 1) = cos(ret.theta);
    
    Eigen::Vector2d dt = ret.t;
    //find nearest neighur
    for (int i = 0; i < pc2.points.size(); i++) {
      PointType search_point = pc2.points[i];
      //project search_point to current frame
      PointType search_point_predict = eigen2point(R * point2eigen(search_point) + dt);
      if (kdtree.nearestKSearch(search_point_predict, K, index, distance) == K) {
        //add constraints
        Eigen::Vector2d p = point2eigen(search_point);
        Eigen::Vector2d p1 = point2eigen(pc1.points[index[0]]);
        Eigen::Vector2d p2 = point2eigen(pc1.points[index[1]]);
        ceres::CostFunction *cost_function = lidar_edge_error::Create(p, p1, p2);
        problem.AddResidualBlock(cost_function, nullptr, pose);
      }
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 50;
    options.num_threads = 1;
    options.use_nonmonotonic_steps = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";

    printf("result: %lf, %lf, %lf\n", pose[0], pose[1], pose[2]);

    ret.theta = pose[0];
    ret.t(0) = pose[1];
    ret.t(1) = pose[2];
  }
  return ret;
}

}  // namespace mh_amcl
