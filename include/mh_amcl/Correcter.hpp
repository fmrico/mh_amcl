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


#ifndef MH_AMCL__CORRECTER_HPP_
#define MH_AMCL__CORRECTER_HPP_

#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/transform_datatypes.h>

#include "mh_amcl/Types.hpp"

#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mh_amcl
{

using std::placeholders::_1;

class CorrecterBase {
public:
  virtual ~CorrecterBase() {};
  std::string type_;
};

template<class T, class M>
class Correcter : public CorrecterBase
{
public:
  Correcter(const std::string & name, nav2_util::LifecycleNode::SharedPtr node, typename std::shared_ptr<M> & map)
  : name_(name),
    node_(node),
    map_(map),
    tf_buffer_(),
    tf_listener_(tf_buffer_)
  {
    std::string topic;
    if (!node->has_parameter(name + ".topic")) {
      node->declare_parameter(name + ".topic", "/perception");
    }
    node->get_parameter(name + ".topic", topic);

    percept_sub_ = node->create_subscription<T>(
      topic, rclcpp::QoS(100).best_effort(), std::bind(&Correcter::perception_callback, this, _1));
  }

  void set_last_perception(T & last_perception) {
    if (last_perception_ == nullptr) {
      last_perception_ = std::make_unique<T>();
    }
    *last_perception_ = last_perception;
  }

  virtual void correct(std::vector<Particle> & particles, rclcpp::Time & update_time) = 0;

public:
  typename T::UniquePtr last_perception_;

protected:
  const std::string name_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;

  typename std::shared_ptr<M> & map_;
  typename rclcpp::Subscription<T>::SharedPtr percept_sub_;

  tf2::BufferCore tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  void perception_callback(typename T::UniquePtr msg) {
    last_perception_ = std::move(msg);
  }
};

}  // namespace mh_amcl

#endif  // MH_AMCL__CORRECTER_HPP_
