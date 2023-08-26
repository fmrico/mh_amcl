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


#ifndef MH_AMCL__TYPES_HPP_
#define MH_AMCL__TYPES_HPP_

#include <tf2/LinearMath/Transform.h>

namespace mh_amcl
{

typedef struct
{
  tf2::Transform pose;
  double prob;
  float hits;
  float possible_hits;
} Particle;

typedef enum TColor
{
  RED, GREEN, BLUE, WHITE, GREY, DARK_GREY, BLACK, YELLOW, ORANGE, BROWN, PINK,
  LIME_GREEN, PURPLE, CYAN, MAGENTA, NUM_COLORS
} Color;

}  // namespace mh_amcl

#endif  // MH_AMCL__TYPES_HPP_
