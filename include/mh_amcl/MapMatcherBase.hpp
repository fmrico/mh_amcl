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


#ifndef MH_AMCL__MAPMATCHERBASE_HPP_
#define MH_AMCL__MAPMATCHERBASE_HPP_

#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/transform_datatypes.h>

#include "mh_amcl/Types.hpp"

#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mh_amcl
{

typedef struct
{
  float weight;
  tf2::Transform transform;
} TransformWeighted;

bool operator<(const TransformWeighted & tw1, const TransformWeighted & tw2);

using std::placeholders::_1;

class MapMatcherBase {
public:
  virtual ~MapMatcherBase() {};

  virtual std::list<TransformWeighted> get_matchs() = 0;

  std::string type_;
};

}  // namespace mh_amcl

#endif  // MH_AMCL__MAPMATCHERBASE_HPP_
