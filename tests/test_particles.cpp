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
#include "mh_amcl/Utils.hpp"

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
    const std::string & name, nav2_util::LifecycleNode::SharedPtr node,
    std::shared_ptr<octomap::OcTree> map)
  : LaserCorrecter(name, node, map)
  {
  }

  tf2::Transform get_tranform_to_read_test(const sensor_msgs::msg::LaserScan & scan, int index)
  {
    return get_tranform_to_read(scan, index);
  }

  tf2::Vector3 get_perception_unit_vector_test(const sensor_msgs::msg::LaserScan & scan, int index)
  {
    return get_perception_unit_vector(scan, index);
  }

  double get_distance_to_obstacle_test(
    const tf2::Transform & map2bf, const tf2::Transform & bf2laser,
    const tf2::Vector3 unit_vector, const sensor_msgs::msg::LaserScan & scan,
    const octomap::OcTree & octomap)
  {
    return get_distance_to_obstacle(map2bf, bf2laser, unit_vector, scan, octomap);
  }

  double get_occupancy_test(
    const tf2::Transform & transform, const octomap::OcTree & octomap)
  {
    return get_occupancy(transform, octomap);
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
  ASSERT_NEAR(stdev_ax, 0.0, 0.05);
  ASSERT_NEAR(mean_ay, 0.0, 0.015);
  ASSERT_NEAR(stdev_ay, 0.0, 0.05);
  ASSERT_NEAR(mean_az, 0.0, 0.015);
  ASSERT_NEAR(stdev_az, 0.05, 0.05);

  auto pose = particle_dist.get_pose();
  ASSERT_NEAR(pose.getOrigin().x(), 0.0, 0.015);
  ASSERT_NEAR(pose.getOrigin().y(), 0.0, 0.015);
  ASSERT_NEAR(pose.getOrigin().z(), 0.0, 0.015);
  ASSERT_NEAR(pose.getRotation().x(), 0.0, 0.015);
  ASSERT_NEAR(pose.getRotation().y(), 0.0, 0.015);
  ASSERT_NEAR(pose.getRotation().z(), 0.0, 0.015);
  ASSERT_NEAR(pose.getRotation().w(), 1.0, 0.015);
  ASSERT_NEAR(pose.cov_mat_[0][0], 0.01, 0.003);
  ASSERT_NEAR(pose.cov_mat_[1][1], 0.01, 0.003);
  ASSERT_NEAR(pose.cov_mat_[2][2], 0.00, 0.001);
  ASSERT_NEAR(pose.cov_mat_[3][3], 0.00, 0.001);
  ASSERT_NEAR(pose.cov_mat_[4][4], 0.00, 0.001);
  ASSERT_NEAR(pose.cov_mat_[5][5], 0.05 * 0.05, 0.003);
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

  ASSERT_NEAR(mean_x, 0.0, 0.1);
  ASSERT_NEAR(stdev_x, 0.1, 0.15);
  ASSERT_NEAR(mean_y, 0.0, 0.1);
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

  particle_dist.predict(trans, nullptr);

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
  ASSERT_NEAR(mean_z, 0.0, 0.025);
}

TEST(test1, test_laser_get_tranform_to_read)
{
  ParticlesDistributionTest particle_dist;
  particle_dist.get_parent()->set_parameter({"min_particles", 200});
  particle_dist.on_configure(
    rclcpp_lifecycle::State(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "Inactive"));

  auto node =  nav2_util::LifecycleNode::make_shared("test");
  node->set_parameter({"test.topic", std::string("/scan")});
  LaserCorrecterTest laser_correcter("test",node, nullptr);

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

TEST(test1, test_laser_get_perception_unit_vector)
{
  ParticlesDistributionTest particle_dist;
  particle_dist.get_parent()->set_parameter({"min_particles", 200});
  particle_dist.on_configure(
    rclcpp_lifecycle::State(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "Inactive"));

  auto node =  nav2_util::LifecycleNode::make_shared("test");
  node->set_parameter({"test.topic", std::string("/scan")});
  LaserCorrecterTest laser_correcter("test",node, nullptr);

  sensor_msgs::msg::LaserScan scan;
  scan.header.frame_id = "laser";
  scan.header.stamp = particle_dist.get_parent()->now();
  scan.angle_min = -M_PI;
  scan.angle_max = M_PI;
  scan.angle_increment = M_PI_2;
  scan.ranges = {1.0, 2.0, 3.0, 4.0, 5.0};

  tf2::Vector3 point_0 = laser_correcter.get_perception_unit_vector_test(scan, 0);
  tf2::Vector3 point_1 = laser_correcter.get_perception_unit_vector_test(scan, 1);
  tf2::Vector3 point_2 = laser_correcter.get_perception_unit_vector_test(scan, 2);
  tf2::Vector3 point_3 = laser_correcter.get_perception_unit_vector_test(scan, 3);
  tf2::Vector3 point_4 = laser_correcter.get_perception_unit_vector_test(scan, 4);

  ASSERT_NEAR(point_0.x(), -1.0, 0.0001);
  ASSERT_NEAR(point_0.y(), 0.0, 0.0001);
  ASSERT_NEAR(point_0.z(), 0.0, 0.0001);

  ASSERT_NEAR(point_1.x(), 0.0, 0.0001);
  ASSERT_NEAR(point_1.y(), -1.0, 0.0001);
  ASSERT_NEAR(point_1.z(), 0.0, 0.0001);

  ASSERT_NEAR(point_2.x(), 1.0, 0.0001);
  ASSERT_NEAR(point_2.y(), 0.0, 0.0001);
  ASSERT_NEAR(point_2.z(), 0.0, 0.0001);

  ASSERT_NEAR(point_3.x(), 0.0, 0.0001);
  ASSERT_NEAR(point_3.y(), 1.0, 0.0001);
  ASSERT_NEAR(point_3.z(), 0.0, 0.0001);

  ASSERT_NEAR(point_4.x(), -1.0, 0.0001);
  ASSERT_NEAR(point_4.y(), 0.0, 0.0001);
  ASSERT_NEAR(point_4.z(), 0.0, 0.0001);
}

TEST(test1, test_laser_get_occupancy)
{
  // Costmap with an obstacle in front and left
  unsigned int size_x = 400;
  unsigned int size_y = 400;
  double resolution = 0.01;

  auto octomap = std::make_shared<octomap::OcTree>(resolution);
  octomap->setProbHit(0.7);
  octomap->setProbMiss(0.4);
  octomap->setClampingThresMax(0.97);
  octomap->setClampingThresMin(0.12);

  for (double x = -1.0; x < 1.0; x = x + (resolution / 2.0)) {
    for (double z = 0.0; z < 2.0; z = z + (resolution / 2.0)) {
      ASSERT_NE(octomap->setNodeValue(x, 1.0, z, octomap::logodds(1.0f)), nullptr);
      ASSERT_NE(octomap->setNodeValue(x, -0.5, z, octomap::logodds(1.0f)), nullptr);
    }
  } 

  for (float y = -0.5; y < 1.0; y = y + (resolution / 2.0)) {
    for (float z = 0.0; z < 2.0; z = z + (resolution / 2.0)) {
      ASSERT_NE(octomap->setNodeValue(0.75f, y, z, octomap::logodds(1.0f)), nullptr);
    }
  }

  // Init particles to (x=0.0, y=0.0, t=90.0)
  ParticlesDistributionTest particle_dist;
  particle_dist.get_parent()->set_parameter({"min_particles", 200});
  particle_dist.on_configure(
    rclcpp_lifecycle::State(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "Inactive"));

  auto node =  nav2_util::LifecycleNode::make_shared("test");
  node->set_parameter({"test.topic", std::string("/scan")});
  LaserCorrecterTest laser_correcter("test",node, nullptr);

  tf2::Transform init_rot;
  init_rot.setOrigin({0.0, 0.0, 0.0});
  init_rot.setRotation({0.0, 0.0, 0.707, 0.707});
  particle_dist.init(init_rot);

  ASSERT_EQ(octomap->search(0.0, 0.0, 0.0), nullptr);
  ASSERT_NE(octomap->search(0.75, 0.0, 0.0), nullptr);
  ASSERT_NE(octomap->search(0.0, -0.5, 0.0), nullptr);
  ASSERT_NE(octomap->search(0.0, 1.0, 0.0), nullptr);
  ASSERT_EQ(octomap->search(0.75, 0.0, 0.0)->getOccupancy(), octomap->getClampingThresMax());
  ASSERT_EQ(octomap->search(0.0, -0.5, 0.0)->getOccupancy(), octomap->getClampingThresMax());
  ASSERT_EQ(octomap->search(0.0, 1.0, 0.0)->getOccupancy(), octomap->getClampingThresMax());

  tf2::Transform tf_test;
  tf_test.setOrigin({0.0, 0.0, 0.0});
  ASSERT_EQ(laser_correcter.get_occupancy_test(tf_test, *octomap), octomap->getClampingThresMin());
  tf_test.setOrigin({0.75, 0.0, 0.0});
  ASSERT_EQ(laser_correcter.get_occupancy_test(tf_test, *octomap), octomap->getClampingThresMax());
  tf_test.setOrigin({0.0, -0.5, 0.0});
  ASSERT_EQ(laser_correcter.get_occupancy_test(tf_test, *octomap), octomap->getClampingThresMax());
  tf_test.setOrigin({0.0, 1.0, 0.0});
  ASSERT_EQ(laser_correcter.get_occupancy_test(tf_test, *octomap), octomap->getClampingThresMax());
}


TEST(test1, test_laser_get_distance_to_obstacle)
{
  auto test_node = rclcpp::Node::make_shared("test_node");
  // Costmap with an obstacle in front and left
  unsigned int size_x = 400;
  unsigned int size_y = 400;
  double resolution = 0.01;

  auto octomap = std::make_shared<octomap::OcTree>(resolution);
  octomap->setProbHit(0.7);
  octomap->setProbMiss(0.4);
  octomap->setClampingThresMax(0.97);
  octomap->setClampingThresMin(0.12);

  for (double x = -5.0; x < 5.0; x = x + (resolution / 2.0)) {
    for (double z = -0.5; z < 2.0; z = z + (resolution / 2.0)) {
      octomap->setNodeValue(x, 5.0, z, octomap::logodds(1.0f));
      octomap->setNodeValue(x, -2.0, z, octomap::logodds(1.0f));
    }
  }

  for (double y = -5.0; y < 5.0; y = y + (resolution / 2.0)) {
    for (double z = -0.5; z < 2.0; z = z + (resolution / 2.0)) {
      octomap->setNodeValue(3.0, y, z, octomap::logodds(1.0f));
    }
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

  auto node =  nav2_util::LifecycleNode::make_shared("test");
  node->set_parameter({"test.topic", std::string("/scan")});
  LaserCorrecterTest laser_correcter("test",node, nullptr);

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
    -3.05,
    2.06,
    4.043,
    std::numeric_limits<float>::infinity(),
    2.97};

  tf2::Transform map2bf;
  map2bf.setOrigin({1.0, 1.0, 0.0});
  map2bf.setRotation({0.0, 0.0, 0.707, 0.707});
  tf2::Transform bf2lasert;
  bf2lasert.setOrigin({0.0, 0.0, 0.0});
  bf2lasert.setRotation({0.0, 0.0, 0.0, 1.0});

  // Real tests
  {
    tf2::Vector3 laser2point_u;
    laser2point_u = laser_correcter.get_perception_unit_vector_test(scan, 0);

    ASSERT_NEAR(laser2point_u.x(), -1.0, 0.0001);
    ASSERT_NEAR(laser2point_u.y(), 0.00, 0.0001);

    double distance = laser_correcter.get_distance_to_obstacle_test(
      map2bf, bf2lasert, laser2point_u, scan, *octomap);

    ASSERT_FALSE(std::isinf(distance));
    ASSERT_NEAR(distance, 3.0, resolution);
  }


  {
    tf2::Vector3 laser2point_u;
    laser2point_u = laser_correcter.get_perception_unit_vector_test(scan, 1);

    ASSERT_NEAR(laser2point_u.x(), 0.0, 0.0001);
    ASSERT_NEAR(laser2point_u.y(), -1.00, 0.0001);

    double distance = laser_correcter.get_distance_to_obstacle_test(
      map2bf, bf2lasert, laser2point_u, scan, *octomap);

    ASSERT_FALSE(std::isinf(distance));
    ASSERT_NEAR(distance, 2.0, resolution);
  }

  
  {
    tf2::Vector3 laser2point_u;
    laser2point_u = laser_correcter.get_perception_unit_vector_test(scan, 2);

    ASSERT_NEAR(laser2point_u.x(), 1.0, 0.0001);
    ASSERT_NEAR(laser2point_u.y(), 0.0, 0.0001);

    double distance = laser_correcter.get_distance_to_obstacle_test(
      map2bf, bf2lasert, laser2point_u, scan, *octomap);

    ASSERT_FALSE(std::isinf(distance));
    ASSERT_NEAR(distance, 4.0, resolution);
  }

  {
    tf2::Vector3 laser2point_u;
    laser2point_u = laser_correcter.get_perception_unit_vector_test(scan, 3);

    ASSERT_NEAR(laser2point_u.x(), 0.0, 0.0001);
    ASSERT_NEAR(laser2point_u.y(), 1.0, 0.0001);

    double distance = laser_correcter.get_distance_to_obstacle_test(
      map2bf, bf2lasert, laser2point_u, scan, *octomap);
  }

  {
    tf2::Vector3 laser2point_u;
    laser2point_u = laser_correcter.get_perception_unit_vector_test(scan, 4);

    ASSERT_NEAR(laser2point_u.x(), -1.0, 0.0001);
    ASSERT_NEAR(laser2point_u.y(), 0.0, 0.0001);

    double distance = laser_correcter.get_distance_to_obstacle_test(
      map2bf, bf2lasert, laser2point_u, scan, *octomap);

    ASSERT_FALSE(std::isinf(distance));
    ASSERT_NEAR(distance, 3.0, resolution);

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

/*
TEST(test1, test_laser_correct_single_voxel)
{
  double resolution = 0.01;

  auto octomap = std::make_shared<octomap::OcTree>(resolution);
  octomap->setProbHit(0.7);
  octomap->setProbMiss(0.4);
  octomap->setClampingThresMax(0.97);
  octomap->setClampingThresMin(0.12);
 
  octomap->setNodeValue(0.0, 0.0, 0.0, true);
  octomap->setNodeValue(5.0, 0.0, 0.0, octomap::logodds(0.3f));
  octomap->setNodeValue(10.0, 0.0, 0.0, octomap::logodds(0.5f));

  std::cerr << " --->" << octomap::probability(octomap->search(0.0, 0.0, 0.0)->getValue()) << std::endl;
  for (int i = 0; i < 10; i++) {

    auto * node = octomap->updateNode(0.0, 0.0, 0.0, octomap::logodds(0.05f));
    std::cerr << i << " --->" << octomap::probability(node->getValue()) << std::endl;
  }
  for (int i = 0; i < 10; i++) {

    auto * node = octomap->updateNode(0.0, 0.0, 0.0, octomap::logodds(0.8f));
    std::cerr << i << " --->" << octomap::probability(node->getValue()) << std::endl;
  }
  for (int i = 0; i < 10; i++) {

    auto * node = octomap->updateNode(0.0, 0.0, 0.0, octomap::logodds(0.05f));
    std::cerr << i << " --->" << octomap::probability(node->getValue()) << std::endl;
  }
  {
    octomap::point3d end;
    bool is_obstacle = octomap->castRay({-1.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, end, true, 20.0);
    std::cerr << "Hit in " << end << std::endl;
  }

  {
    std::vector<octomap::point3d> ray;
    octomap->computeRay({-1.0f, 0.0f, 0.0f}, {20.0f, 0.0f, 0.0f}, ray);
    for (const auto & p : ray) {
      std::cerr << p << " -> ";
    }

    std::cerr << std::endl;
  }
}*/

TEST(test1, test_laser_correct)
{
  // Costmap with an obstacle in (1, 0)
  unsigned int size_x = 400;
  unsigned int size_y = 400;
  double resolution = 0.01;

  auto octomap = std::make_shared<octomap::OcTree>(resolution);
  octomap->setProbHit(0.7);
  octomap->setProbMiss(0.4);
  octomap->setClampingThresMax(0.97);
  octomap->setClampingThresMin(0.12);

  for (double x = -5.0; x < 5.0; x = x + (resolution / 2.0)) {
    for (double z = -0.5; z < 2.0; z = z + (resolution / 2.0)) {
      octomap->setNodeValue(x, 5.0, z, octomap::logodds(1.0f));
      octomap->setNodeValue(x, -2.0, z, octomap::logodds(1.0f));
    }
  }

  for (double y = -5.0; y < 5.0; y = y + (resolution / 2.0)) {
    for (double z = -0.5; z < 2.0; z = z + (resolution / 2.0)) {
      octomap->setNodeValue(3.0, y, z, octomap::logodds(1.0f));
    }
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
    std::numeric_limits<float>::infinity(),  // Hit
    std::numeric_limits<float>::infinity(),  // Miss
    1.0,  // Miss
    5.0,  // Hit
    std::numeric_limits<float>::infinity()}; // Hit
  // Here we should set perception noise to 0.0 to avoid random errors

  auto node = nav2_util::LifecycleNode::make_shared("aux_node");
  node->set_parameter({"aux_node.topic", std::string("/scan")});
  mh_amcl::LaserCorrecter * correcter = new mh_amcl::LaserCorrecter("test", node, octomap);

  correcter->type_ = "laser";
  correcter->set_last_perception(scan);

  rclcpp::Time last_time;
  particle_dist.correct_once({correcter}, last_time);

  ASSERT_EQ(last_time, rclcpp::Time(scan.header.stamp, last_time.get_clock_type()));

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
