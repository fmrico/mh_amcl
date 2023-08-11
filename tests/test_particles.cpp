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

#include "mh_amcl/ParticlesDistribution.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "nav2_costmap_2d/costmap_2d.hpp"

#include "mh_amcl/LaserCorrecter.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"


class ParticlesDistributionTest : public mh_amcl::ParticlesDistribution
{
public:
  ParticlesDistributionTest()
  : ParticlesDistribution(rclcpp_lifecycle::LifecycleNode::make_shared("test_node")) {}

  int get_num_particles() {return particles_.size();}

  rclcpp_lifecycle::LifecycleNode::SharedPtr get_parent() {return parent_node_;}
  std::vector<mh_amcl::Particle> & get_particles_test() {return particles_;}

  void normalize_test() {normalize();}
};

class LaserCorrecterTest : public mh_amcl::LaserCorrecter
{
public:
  LaserCorrecterTest(
    nav2_util::LifecycleNode::SharedPtr node, const std::string & topic,
    std::shared_ptr<nav2_costmap_2d::Costmap2D> map)
  : LaserCorrecter(node, topic, map)
  {
  }

  tf2::Transform get_tranform_to_read_test(const sensor_msgs::msg::LaserScan & scan, int index)
  {
    return get_tranform_to_read(scan, index);
  }

  double get_error_distance_to_obstacle_test(
    const tf2::Transform & map2bf, const tf2::Transform & bf2laser,
    const tf2::Transform & laser2point, const sensor_msgs::msg::LaserScan & scan,
    const nav2_costmap_2d::Costmap2D & costmap, double o)
  {
    return get_error_distance_to_obstacle(map2bf, bf2laser, laser2point, scan, costmap, o);
  }

  unsigned char get_cost_test(
    const tf2::Transform & transform, const nav2_costmap_2d::Costmap2D & costmap)
  {
    return get_cost(transform, costmap);
  }
};


std::tuple<double, double> get_mean_stdev(std::vector<double> & values)
{
  if (values.empty()) {
    return {0.0, 0.0};
  }

  double mean = 0.0;
  double stdev = 0.0;

  for (const auto & value : values) {
    mean += value;
  }

  mean = mean / values.size();

  for (const auto & value : values) {
    stdev += fabs(value - mean);
  }

  stdev = stdev / values.size();

  return {mean, stdev};
}


TEST(test1, test_init)
{
  ParticlesDistributionTest particle_dist;
  particle_dist.get_parent()->set_parameter({"min_particles", 200});
  particle_dist.on_configure(
    rclcpp_lifecycle::State(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "Inactive"));
  ASSERT_EQ(particle_dist.get_num_particles(), 200);

  auto particles = particle_dist.get_particles_test();

  std::vector<double> pos_x(particles.size());
  std::vector<double> pos_y(particles.size());
  std::vector<double> pos_z(particles.size());
  std::vector<double> angle_x(particles.size());
  std::vector<double> angle_y(particles.size());
  std::vector<double> angle_z(particles.size());

  for (int i = 0; i < particles.size(); i++) {
    auto pos = particles[i].pose.getOrigin();

    pos_x[i] = pos.x();
    pos_y[i] = pos.y();
    pos_z[i] = pos.z();

    tf2::Matrix3x3 m(particles[i].pose.getRotation());
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    angle_x[i] = roll;
    angle_y[i] = pitch;
    angle_z[i] = yaw;

    ASSERT_NEAR(particles[i].prob, 1.0 / particles.size(), 0.0001);
  }

  ASSERT_NE(particles.size(), 0);

  auto [mean_x, stdev_x] = get_mean_stdev(pos_x);
  auto [mean_y, stdev_y] = get_mean_stdev(pos_y);
  auto [mean_z, stdev_z] = get_mean_stdev(pos_z);
  auto [mean_ax, stdev_ax] = get_mean_stdev(angle_x);
  auto [mean_ay, stdev_ay] = get_mean_stdev(angle_y);
  auto [mean_az, stdev_az] = get_mean_stdev(angle_z);

  ASSERT_NEAR(mean_x, 0.0, 0.015);
  ASSERT_NEAR(stdev_x, 0.1, 0.05);
  ASSERT_NEAR(mean_y, 0.0, 0.015);
  ASSERT_NEAR(stdev_y, 0.1, 0.05);
  ASSERT_NEAR(mean_z, 0.0, 0.0001);
  ASSERT_NEAR(mean_ax, 0.0, 0.015);
  ASSERT_NEAR(stdev_ax, 0.0, 0.0);
  ASSERT_NEAR(mean_ay, 0.0, 0.015);
  ASSERT_NEAR(stdev_ay, 0.0, 0.05);
  ASSERT_NEAR(mean_az, 0.0, 0.015);
  ASSERT_NEAR(stdev_az, 0.05, 0.02);

  auto pose = particle_dist.get_pose();
  ASSERT_NEAR(pose.pose.pose.position.x, 0.0, 0.015);
  ASSERT_NEAR(pose.pose.pose.position.y, 0.0, 0.015);
  ASSERT_NEAR(pose.pose.pose.position.z, 0.0, 0.015);
  ASSERT_NEAR(pose.pose.pose.orientation.x, 0.0, 0.015);
  ASSERT_NEAR(pose.pose.pose.orientation.y, 0.0, 0.015);
  ASSERT_NEAR(pose.pose.pose.orientation.z, 0.0, 0.015);
  ASSERT_NEAR(pose.pose.pose.orientation.w, 1.0, 0.015);
  ASSERT_NEAR(pose.pose.covariance[0], 0.01, 0.003);
  ASSERT_NEAR(pose.pose.covariance[7], 0.01, 0.003);
  ASSERT_NEAR(pose.pose.covariance[14], 0.00, 0.001);
  ASSERT_NEAR(pose.pose.covariance[21], 0.00, 0.001);
  ASSERT_NEAR(pose.pose.covariance[28], 0.00, 0.001);
  ASSERT_NEAR(pose.pose.covariance[35], 0.05 * 0.05, 0.003);
}

TEST(test1, test_init_2)
{
  // Init particles to (x=0.0, y=0.0, t=90.0)
  ParticlesDistributionTest particle_dist;
  particle_dist.get_parent()->set_parameter({"min_particles", 200});

  particle_dist.on_configure(
    rclcpp_lifecycle::State(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "Inactive"));
  tf2::Transform init_rot;
  init_rot.setOrigin({0.0, 0.0, 0.0});
  init_rot.setRotation({0.0, 0.0, 0.707, 0.707});
  particle_dist.init(init_rot);

  ASSERT_EQ(particle_dist.get_num_particles(), 200);

  auto particles = particle_dist.get_particles_test();

  std::vector<double> pos_x(particles.size());
  std::vector<double> pos_y(particles.size());
  std::vector<double> pos_z(particles.size());
  std::vector<double> angle_x(particles.size());
  std::vector<double> angle_y(particles.size());
  std::vector<double> angle_z(particles.size());

  for (int i = 0; i < particles.size(); i++) {
    auto pos = particles[i].pose.getOrigin();

    pos_x[i] = pos.x();
    pos_y[i] = pos.y();
    pos_z[i] = pos.z();

    tf2::Matrix3x3 m(particles[i].pose.getRotation());
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    angle_x[i] = roll;
    angle_y[i] = pitch;
    angle_z[i] = yaw;

    ASSERT_NEAR(particles[i].prob, 1.0 / particles.size(), 0.0001);
  }

  ASSERT_NE(particles.size(), 0);

  auto [mean_x, stdev_x] = get_mean_stdev(pos_x);
  auto [mean_y, stdev_y] = get_mean_stdev(pos_y);
  auto [mean_z, stdev_z] = get_mean_stdev(pos_z);
  auto [mean_ax, stdev_ax] = get_mean_stdev(angle_x);
  auto [mean_ay, stdev_ay] = get_mean_stdev(angle_y);
  auto [mean_az, stdev_az] = get_mean_stdev(angle_z);

  ASSERT_NEAR(mean_x, 0.0, 0.015);
  ASSERT_NEAR(stdev_x, 0.1, 0.05);
  ASSERT_NEAR(mean_y, 0.0, 0.015);
  ASSERT_NEAR(stdev_y, 0.1, 0.05);
  ASSERT_NEAR(mean_z, 0.0, 0.0001);
  ASSERT_NEAR(mean_ax, 0.0, 0.015);
  ASSERT_NEAR(stdev_ax, 0.0, 0.05);
  ASSERT_NEAR(mean_ay, 0.0, 0.015);
  ASSERT_NEAR(stdev_ay, 0.0, 0.05);
  ASSERT_NEAR(mean_az, M_PI_2, 0.15);
  ASSERT_NEAR(stdev_az, 0.05, 0.02);
}

TEST(test1, test_reseed)
{
  ParticlesDistributionTest particle_dist;
  particle_dist.get_parent()->set_parameter({"min_particles", 200});

  particle_dist.on_configure(
    rclcpp_lifecycle::State(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "Inactive"));

  ASSERT_EQ(particle_dist.get_num_particles(), 200);

  for (int i = 0; i < 1000; i++) {
    particle_dist.reseed();
  }

  auto particles = particle_dist.get_particles_test();

  std::vector<double> pos_x(particles.size());
  std::vector<double> pos_y(particles.size());
  std::vector<double> pos_z(particles.size());

  for (int i = 0; i < particles.size(); i++) {
    auto pos = particles[i].pose.getOrigin();

    pos_x[i] = pos.x();
    pos_y[i] = pos.y();
    pos_z[i] = pos.z();
  }

  ASSERT_NE(particles.size(), 0);

  auto [mean_x, stdev_x] = get_mean_stdev(pos_x);
  auto [mean_y, stdev_y] = get_mean_stdev(pos_y);
  auto [mean_z, stdev_z] = get_mean_stdev(pos_z);

  ASSERT_NEAR(mean_x, 0.0, 0.04);
  ASSERT_NEAR(stdev_x, 0.1, 0.15);
  ASSERT_NEAR(mean_y, 0.0, 0.06);
  ASSERT_NEAR(stdev_y, 0.1, 0.15);
  ASSERT_NEAR(mean_z, 0.0, 0.0001);
}

TEST(test1, test_predict)
{
  ParticlesDistributionTest particle_dist;
  particle_dist.get_parent()->set_parameter({"min_particles", 200});

  particle_dist.on_configure(
    rclcpp_lifecycle::State(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "Inactive"));
  ASSERT_EQ(particle_dist.get_num_particles(), 200);

  tf2::Transform trans;
  trans.setOrigin(tf2::Vector3(1.0, 0.0, 0.0));
  trans.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));

  particle_dist.predict(trans);

  auto particles = particle_dist.get_particles_test();

  std::vector<double> pos_x(particles.size());
  std::vector<double> pos_y(particles.size());
  std::vector<double> pos_z(particles.size());

  for (int i = 0; i < particles.size(); i++) {
    auto pos = particles[i].pose.getOrigin();

    pos_x[i] = pos.x();
    pos_y[i] = pos.y();
    pos_z[i] = pos.z();
  }

  ASSERT_NE(particles.size(), 0);

  auto [mean_x, stdev_x] = get_mean_stdev(pos_x);
  auto [mean_y, stdev_y] = get_mean_stdev(pos_y);
  auto [mean_z, stdev_z] = get_mean_stdev(pos_z);

  ASSERT_NEAR(mean_x, 1.0, 0.05);
  ASSERT_NEAR(stdev_x, 0.1, 0.05);
  ASSERT_NEAR(mean_y, 0.0, 0.025);
  ASSERT_NEAR(stdev_y, 0.1, 0.1);
  ASSERT_NEAR(mean_z, 0.0, 0.0001);
}

TEST(test1, test_laser_get_tranform_to_read)
{
  ParticlesDistributionTest particle_dist;
  particle_dist.get_parent()->set_parameter({"min_particles", 200});
  particle_dist.on_configure(
    rclcpp_lifecycle::State(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "Inactive"));
  LaserCorrecterTest laser_correcter(
    nav2_util::LifecycleNode::make_shared("test"), "/scan", nullptr);

  sensor_msgs::msg::LaserScan scan;
  scan.header.frame_id = "laser";
  scan.header.stamp = particle_dist.get_parent()->now();
  scan.angle_min = -M_PI;
  scan.angle_max = M_PI;
  scan.angle_increment = M_PI_2;
  scan.ranges = {1.0, 2.0, 3.0, 4.0, 5.0};

  tf2::Transform point_0 = laser_correcter.get_tranform_to_read_test(scan, 0);
  tf2::Transform point_1 = laser_correcter.get_tranform_to_read_test(scan, 1);
  tf2::Transform point_2 = laser_correcter.get_tranform_to_read_test(scan, 2);
  tf2::Transform point_3 = laser_correcter.get_tranform_to_read_test(scan, 3);
  tf2::Transform point_4 = laser_correcter.get_tranform_to_read_test(scan, 4);

  ASSERT_NEAR(point_0.getOrigin().x(), -1.0, 0.0001);
  ASSERT_NEAR(point_0.getOrigin().y(), 0.0, 0.0001);
  ASSERT_NEAR(point_0.getOrigin().z(), 0.0, 0.0001);

  ASSERT_NEAR(point_1.getOrigin().x(), 0.0, 0.0001);
  ASSERT_NEAR(point_1.getOrigin().y(), -2.0, 0.0001);
  ASSERT_NEAR(point_1.getOrigin().z(), 0.0, 0.0001);

  ASSERT_NEAR(point_2.getOrigin().x(), 3.0, 0.0001);
  ASSERT_NEAR(point_2.getOrigin().y(), 0.0, 0.0001);
  ASSERT_NEAR(point_2.getOrigin().z(), 0.0, 0.0001);

  ASSERT_NEAR(point_3.getOrigin().x(), 0.0, 0.0001);
  ASSERT_NEAR(point_3.getOrigin().y(), 4.0, 0.0001);
  ASSERT_NEAR(point_3.getOrigin().z(), 0.0, 0.0001);

  ASSERT_NEAR(point_4.getOrigin().x(), -5.0, 0.0001);
  ASSERT_NEAR(point_4.getOrigin().y(), 0.0, 0.0001);
  ASSERT_NEAR(point_4.getOrigin().z(), 0.0, 0.0001);
}


TEST(test1, test_laser_get_cost)
{
  // Costmap with an obstacle in front and left
  unsigned int size_x = 400;
  unsigned int size_y = 400;
  double resolution = 0.01;

  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2D>();
  costmap->resizeMap(size_x, size_y, resolution, -2.0, -2.0);

  unsigned int index = 0;
  // initialize the costmap with static data
  for (unsigned int i = 0; i < size_y; ++i) {
    for (unsigned int j = 0; j < size_x; ++j) {
      unsigned int x, y;
      costmap->indexToCells(index, x, y);
      costmap->setCost(x, y, nav2_costmap_2d::FREE_SPACE);
      ++index;
    }
  }

  for (double x = -1.0; x < 1.0; x = x + (resolution / 2.0)) {
    unsigned int mx, my;
    costmap->worldToMap(x, 1.0, mx, my);
    costmap->setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
    costmap->worldToMap(x, -0.5, mx, my);
    costmap->setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
  }

  for (double y = -0.5; y < 1.0; y = y + (resolution / 2.0)) {
    unsigned int mx, my;
    costmap->worldToMap(0.75, y, mx, my);
    costmap->setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
  }

  // Init particles to (x=0.0, y=0.0, t=90.0)
  ParticlesDistributionTest particle_dist;
  particle_dist.get_parent()->set_parameter({"min_particles", 200});
  particle_dist.on_configure(
    rclcpp_lifecycle::State(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "Inactive"));

  LaserCorrecterTest laser_correcter(
    nav2_util::LifecycleNode::make_shared("test"), "/scan", nullptr);

  tf2::Transform init_rot;
  init_rot.setOrigin({0.0, 0.0, 0.0});
  init_rot.setRotation({0.0, 0.0, 0.707, 0.707});
  particle_dist.init(init_rot);

  unsigned int mx, my;

  costmap->worldToMap(0.0, 0.0, mx, my);
  ASSERT_EQ(costmap->getCost(mx, my), nav2_costmap_2d::FREE_SPACE);
  costmap->worldToMap(0.75, 0.0, mx, my);
  ASSERT_EQ(costmap->getCost(mx, my), nav2_costmap_2d::LETHAL_OBSTACLE);
  costmap->worldToMap(0.0, -0.5, mx, my);
  ASSERT_EQ(costmap->getCost(mx, my), nav2_costmap_2d::LETHAL_OBSTACLE);
  costmap->worldToMap(0.0, 1.0, mx, my);
  ASSERT_EQ(costmap->getCost(mx, my), nav2_costmap_2d::LETHAL_OBSTACLE);

  tf2::Transform tf_test;
  tf_test.setOrigin({0.0, 0.0, 0.0});
  ASSERT_EQ(laser_correcter.get_cost_test(tf_test, *costmap), nav2_costmap_2d::FREE_SPACE);
  tf_test.setOrigin({0.75, 0.0, 0.0});
  ASSERT_EQ(laser_correcter.get_cost_test(tf_test, *costmap), nav2_costmap_2d::LETHAL_OBSTACLE);
  tf_test.setOrigin({0.0, -0.5, 0.0});
  ASSERT_EQ(laser_correcter.get_cost_test(tf_test, *costmap), nav2_costmap_2d::LETHAL_OBSTACLE);
  tf_test.setOrigin({0.0, 1.0, 0.0});
  ASSERT_EQ(laser_correcter.get_cost_test(tf_test, *costmap), nav2_costmap_2d::LETHAL_OBSTACLE);
}

TEST(test1, test_laser_get_error_distance_to_obstacle)
{
  auto test_node = rclcpp::Node::make_shared("test_node");
  // Costmap with an obstacle in front and left
  unsigned int size_x = 400;
  unsigned int size_y = 400;
  double resolution = 0.01;

  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2D>();
  costmap->resizeMap(size_x, size_y, resolution, -2.0, -2.0);

  unsigned int index = 0;
  // initialize the costmap with static data
  for (unsigned int i = 0; i < size_y; ++i) {
    for (unsigned int j = 0; j < size_x; ++j) {
      unsigned int x, y;
      costmap->indexToCells(index, x, y);
      costmap->setCost(x, y, nav2_costmap_2d::FREE_SPACE);
      ++index;
    }
  }

  for (double x = -1.0; x < 1.0; x = x + (resolution / 2.0)) {
    unsigned int mx, my;
    costmap->worldToMap(x, 1.0, mx, my);
    costmap->setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
    costmap->worldToMap(x, -0.5, mx, my);
    costmap->setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
  }

  for (double y = -0.5; y < 1.0; y = y + (resolution / 2.0)) {
    unsigned int mx, my;
    costmap->worldToMap(0.75, y, mx, my);
    costmap->setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
  }


  // Transform base_footprint -> laser
  tf2_ros::StaticTransformBroadcaster tf_pub(test_node);
  geometry_msgs::msg::TransformStamped bf2laser;
  bf2laser.header.stamp = test_node->now();
  bf2laser.header.frame_id = "base_footprint";
  bf2laser.child_frame_id = "laser";
  bf2laser.transform.rotation.w = 1.0;
  tf_pub.sendTransform({bf2laser});

  // Init particles to (x=0.0, y=0.0, t=90.0)
  ParticlesDistributionTest particle_dist;
  particle_dist.get_parent()->set_parameter({"min_particles", 200});
  particle_dist.on_configure(
    rclcpp_lifecycle::State(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "Inactive"));

  LaserCorrecterTest laser_correcter(
    nav2_util::LifecycleNode::make_shared("test"), "/scan", nullptr);

  tf2::Transform init_rot;
  init_rot.setOrigin({0.0, 0.0, 0.0});
  init_rot.setRotation({0.0, 0.0, 0.707, 0.707});
  particle_dist.init(init_rot);

  // Spin for 1 sec to receive TFs
  rclcpp::Rate rate(20);
  auto start = test_node->now();
  while ((test_node->now() - start).seconds() < 1.0) {
    rclcpp::spin_some(test_node);
    rate.sleep();
  }

  // Scan with an obstacle in (1, 0)
  sensor_msgs::msg::LaserScan scan;
  scan.header.frame_id = "laser";
  scan.header.stamp = test_node->now();
  scan.range_min = 0.05;
  scan.range_max = 20.0;
  scan.angle_min = -M_PI;
  scan.angle_max = M_PI;
  scan.angle_increment = M_PI_2;
  scan.ranges = {
    0.54,
    0.76,
    1.0,
    std::numeric_limits<float>::infinity(),
    0.57};

  tf2::Transform map2bf;
  map2bf.setOrigin({0.0, 0.0, 0.0});
  map2bf.setRotation({0.0, 0.0, 0.707, 0.707});
  tf2::Transform bf2lasert;
  bf2lasert.setOrigin({0.0, 0.0, 0.0});
  bf2lasert.setRotation({0.0, 0.0, 0.0, 1.0});

  // Real tests
  {
    tf2::Transform laser2point;
    laser2point = laser_correcter.get_tranform_to_read_test(scan, 0);

    ASSERT_NEAR(laser2point.getOrigin().x(), -0.54, 0.0001);
    ASSERT_NEAR(laser2point.getOrigin().y(), 0.00, 0.0001);

    double distance = laser_correcter.get_error_distance_to_obstacle_test(
      map2bf, bf2lasert, laser2point, scan, *costmap, 0.02);

    ASSERT_FALSE(std::isinf(distance));
    ASSERT_NEAR(distance, 0.05, 0.0001);
  }

  {
    tf2::Transform laser2point;
    laser2point = laser_correcter.get_tranform_to_read_test(scan, 1);

    ASSERT_NEAR(laser2point.getOrigin().x(), 0.0, 0.0001);
    ASSERT_NEAR(laser2point.getOrigin().y(), -0.76, 0.0001);

    double distance = laser_correcter.get_error_distance_to_obstacle_test(
      map2bf, bf2lasert, laser2point, scan, *costmap, 0.02);

    ASSERT_FALSE(std::isinf(distance));
    ASSERT_NEAR(distance, 0.0, 0.0001);
  }

  {
    tf2::Transform laser2point;
    laser2point = laser_correcter.get_tranform_to_read_test(scan, 2);

    ASSERT_NEAR(laser2point.getOrigin().x(), 1.0, 0.0001);
    ASSERT_NEAR(laser2point.getOrigin().y(), 0.0, 0.0001);

    double distance = laser_correcter.get_error_distance_to_obstacle_test(
      map2bf, bf2lasert, laser2point, scan, *costmap, 0.02);

    ASSERT_FALSE(std::isinf(distance));
    ASSERT_NEAR(distance, 0.0, 0.0001);
  }

  {
    tf2::Transform laser2point;
    laser2point = laser_correcter.get_tranform_to_read_test(scan, 3);

    double distance = laser_correcter.get_error_distance_to_obstacle_test(
      map2bf, bf2lasert, laser2point, scan, *costmap, 0.02);

    ASSERT_TRUE(std::isinf(distance));
  }

  {
    tf2::Transform laser2point;
    laser2point = laser_correcter.get_tranform_to_read_test(scan, 4);

    double distance = laser_correcter.get_error_distance_to_obstacle_test(
      map2bf, bf2lasert, laser2point, scan, *costmap, 0.02);

    ASSERT_TRUE(std::isinf(distance));
  }
}

TEST(test1, normalize)
{
  ParticlesDistributionTest particle_dist;
  particle_dist.get_parent()->set_parameter({"min_particles", 200});
  particle_dist.on_configure(
    rclcpp_lifecycle::State(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "Inactive"));

  auto & particles = particle_dist.get_particles_test();

  std::for_each(
    particles.begin(), particles.end(), [&](const mh_amcl::Particle & p) {
      ASSERT_NEAR(p.prob, 1.0 / particles.size(), 0.0001);
    });

  std::for_each(
    particles.begin(), particles.end(), [&](mh_amcl::Particle & p) {
      p.prob = p.prob * 2.0;
    });

  std::for_each(
    particles.begin(), particles.end(), [&](const mh_amcl::Particle & p) {
      ASSERT_NEAR(p.prob, 2.0 / particles.size(), 0.0001);
    });

  particle_dist.normalize_test();

  std::for_each(
    particles.begin(), particles.end(), [&](const mh_amcl::Particle & p) {
      ASSERT_NEAR(p.prob, 1.0 / particles.size(), 0.0001);
    });
}

TEST(test1, test_laser_correct)
{
  // Costmap with an obstacle in (1, 0)
  unsigned int size_x = 400;
  unsigned int size_y = 400;
  double resolution = 0.01;

  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2D>();
  costmap->resizeMap(size_x, size_y, resolution, -2.0, -2.0);

  unsigned int index = 0;
  // initialize the costmap with static data
  for (unsigned int i = 0; i < size_y; ++i) {
    for (unsigned int j = 0; j < size_x; ++j) {
      unsigned int x, y;
      costmap->indexToCells(index, x, y);
      costmap->setCost(x, y, nav2_costmap_2d::FREE_SPACE);
      ++index;
    }
  }

  for (double x = -1.0; x < 1.0; x = x + resolution) {
    unsigned int mx, my;
    costmap->worldToMap(x, 1.0, mx, my);
    costmap->setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
  }

  for (double y = -1.0; y < 1.0; y = y + resolution) {
    unsigned int mx, my;
    costmap->worldToMap(1.0, y, mx, my);
    costmap->setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
  }

  auto test_node = rclcpp_lifecycle::LifecycleNode::make_shared("test_node");
  // Transform base_footprint -> laser
  tf2_ros::StaticTransformBroadcaster tf_pub(test_node);
  geometry_msgs::msg::TransformStamped bf2laser;
  bf2laser.header.stamp = test_node->now();
  bf2laser.header.frame_id = "base_footprint";
  bf2laser.child_frame_id = "laser";
  bf2laser.transform.rotation.w = 1.0;
  tf_pub.sendTransform({bf2laser});

  // Init particles to (x=0.0, y=0.0, t=0.0)
  ParticlesDistributionTest particle_dist;
  particle_dist.get_parent()->set_parameter({"min_particles", 200});
  particle_dist.on_configure(
    rclcpp_lifecycle::State(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "Inactive"));

  // Spin for 1 sec to receive TFs
  rclcpp::Rate rate(20);
  auto start = test_node->now();
  while ((test_node->now() - start).seconds() < 1.0) {
    rclcpp::spin_some(test_node->get_node_base_interface());
    rate.sleep();
  }

  // Scan with an obstacle in (1, 0)
  sensor_msgs::msg::LaserScan scan;
  scan.header.frame_id = "laser";
  scan.header.stamp = test_node->now();
  scan.range_min = 0.05;
  scan.range_max = 20.0;
  scan.angle_min = -M_PI;
  scan.angle_max = M_PI;
  scan.angle_increment = M_PI_2;
  scan.ranges = {
    std::numeric_limits<float>::infinity(),
    std::numeric_limits<float>::infinity(),
    1.0,
    1.0,
    std::numeric_limits<float>::infinity()};
  // Here we should set perception noise to 0.0 to avoid random errors

  auto node = nav2_util::LifecycleNode::make_shared("aux_node");
  mh_amcl::LaserCorrecter correcter(node, "/scan", costmap);
  correcter.set_last_perception(scan);

  rclcpp::Time last_time;
  particle_dist.correct_once({&correcter}, last_time);

  ASSERT_EQ(last_time, scan.header.stamp);

  ASSERT_GT(particle_dist.get_quality(), 0.3);

  auto & particles = particle_dist.get_particles_test();

  // Sort particles by prob
  std::sort(
    particles.begin(), particles.end(),
    [](const mh_amcl::Particle & a, const mh_amcl::Particle & b) -> bool
    {
      return a.prob > b.prob;
    });

  for (int i = 0; i < 5; i++) {
    const mh_amcl::Particle & part = particles[i];

    const double x = part.pose.getOrigin().x();
    const double y = part.pose.getOrigin().y();

    double dist = sqrt(x * x + y * y);
    ASSERT_LE(dist, 0.15);
  }

  particle_dist.normalize_test();

  double sum_probs = 0.0;
  for (const auto p : particle_dist.get_particles_test()) {
    sum_probs += p.prob;
  }

  ASSERT_NEAR(sum_probs, 1.0, 0.000001);
}

TEST(test1, test_statistics)
{
  std::vector<double> v1 = {5.0, 6.0, 7.0, 5.0, 6.0, 7.0};
  std::vector<double> v2 = {8.0, 8.0, 8.0, 9.0, 9.0, 94.0};

  ASSERT_NEAR(mh_amcl::mean(v1), 6.0, 0.001);
  ASSERT_NEAR(mh_amcl::mean(v2), 22.667, 0.001);
  ASSERT_NEAR(mh_amcl::covariance(v1, v2), 17, 0.001);
}

int main(int argc, char * argv[])
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
  return 0;
}
