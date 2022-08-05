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


#include <limits>

#include "gtest/gtest.h"

#include "mh_amcl/MH_AMCL.hpp"

#include "nav2_costmap_2d/cost_values.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "nav2_costmap_2d/costmap_2d.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"


class MH_AMCL_NodeTest : public mh_amcl::MH_AMCL_Node
{
public:
  MH_AMCL_NodeTest() : MH_AMCL_Node() {}

  double pdf_test(const geometry_msgs::msg::Pose & pose,
    geometry_msgs::msg::PoseWithCovariance & distrib)
  {
    return pdf(pose, distrib);
  }
};


TEST(test1, test_pdf)
{
  MH_AMCL_NodeTest mh_amcl;

  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.05001;
  pose.position.y = -0.00011;
  pose.position.z = 0.004;

  geometry_msgs::msg::PoseWithCovariance distrib;
  for (int i = 0; i < 6; i++) {
    distrib.covariance[i * 6 + i] = 0.5 * 0.5;
  }
  // distrib.covariance[0 * 6 + 0] = 0.1 * 0.1;
  // distrib.covariance[1 * 6 + 1] = 0.1 * 0.1;
  // distrib.covariance[5 * 6 + 5] = 0.1 * 0.1;

  std::cerr << mh_amcl.pdf_test(pose, distrib) << std::endl;
}

int main(int argc, char * argv[])
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
  return 0;
}
