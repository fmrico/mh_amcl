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

#include "aamcl/ParticlesDistribution.h"
#include "costmap_2d/cost_values.h"
#include "tf2_ros/static_transform_broadcaster.h"

#include <gtest/gtest.h>

class ParticlesDistributionTest: public aamcl::ParticlesDistribution
{
public:
  ParticlesDistributionTest() : ParticlesDistribution() {}

  int get_num_particles() {return particles_.size();}

  std::vector<aamcl::Particle> & get_particles() {return particles_;}
  tf2::Transform get_tranform_to_read_test(const sensor_msgs::LaserScan & scan, int index)
  {
    return get_tranform_to_read(scan, index);
  }

  double get_error_distance_to_obstacle_test(
    const tf2::Transform & map2bf, const tf2::Transform & bf2laser,  const tf2::Transform & laser2point,
    const sensor_msgs::LaserScan & scan, const costmap_2d::Costmap2D & costmap, double o)
  {
    return get_error_distance_to_obstacle(map2bf, bf2laser, laser2point, scan, costmap, o);
  }

  void normalize_test() {normalize();}
  unsigned char get_cost_test( const tf2::Transform & transform, const costmap_2d::Costmap2D & costmap)
  {
    return get_cost(transform, costmap);
  }
};

std::tuple<double, double> get_mean_stdev(std::vector<double> & values)
{
  if (values.empty())
  {
    return {0.0, 0.0};
  }

  double mean = 0.0;
  double stdev = 0.0;

  for (const auto & value : values)
  {
    mean += value;
  }

  mean = mean / values.size();

  for (const auto & value : values)
  {
    stdev += fabs(value - mean);
  }

  stdev = stdev / values.size();

  return {mean, stdev};
}


TEST(test1, test_init)
{
  ParticlesDistributionTest particle_dist;
  particle_dist.init();
  ASSERT_EQ(particle_dist.get_num_particles(), 200);

  auto particles = particle_dist.get_particles();

  std::vector<double> pos_x(particles.size());
  std::vector<double> pos_y(particles.size());
  std::vector<double> pos_z(particles.size());
  std::vector<double> angle_x(particles.size());
  std::vector<double> angle_y(particles.size());
  std::vector<double> angle_z(particles.size());
  
  for (int i = 0; i < particles.size(); i++)
  {
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
}

TEST(test1, test_init_2)
{
  // Init particles to (x=0.0, y=0.0, t=90.0)
  ParticlesDistributionTest particle_dist;
  tf2::Transform init_rot;
  init_rot.setOrigin({0.0, 0.0, 0.0});
  init_rot.setRotation({0.0, 0.0, 0.707, 0.707});
  particle_dist.init(init_rot);

  ASSERT_EQ(particle_dist.get_num_particles(), 200);

  auto particles = particle_dist.get_particles();

  std::vector<double> pos_x(particles.size());
  std::vector<double> pos_y(particles.size());
  std::vector<double> pos_z(particles.size());
  std::vector<double> angle_x(particles.size());
  std::vector<double> angle_y(particles.size());
  std::vector<double> angle_z(particles.size());

  for (int i = 0; i < particles.size(); i++)
  {
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
  ASSERT_NEAR(mean_az,  M_PI_2, 0.15);
  ASSERT_NEAR(stdev_az, 0.05, 0.02);
}

TEST(test1, test_reseed)
{
  ParticlesDistributionTest particle_dist;
  particle_dist.init();
  ASSERT_EQ(particle_dist.get_num_particles(), 200);

  for (int i = 0; i < 1000; i++) {
    particle_dist.reseed();
  }

  auto particles = particle_dist.get_particles();

  std::vector<double> pos_x(particles.size());
  std::vector<double> pos_y(particles.size());
  std::vector<double> pos_z(particles.size());

  for (int i = 0; i < particles.size(); i++)
  {
    auto pos = particles[i].pose.getOrigin();

    pos_x[i] = pos.x();
    pos_y[i] = pos.y();
    pos_z[i] = pos.z();
  }

  ASSERT_NE(particles.size(), 0);

  auto [mean_x, stdev_x] = get_mean_stdev(pos_x);
  auto [mean_y, stdev_y] = get_mean_stdev(pos_y);
  auto [mean_z, stdev_z] = get_mean_stdev(pos_z);

  ASSERT_NEAR(mean_x, 0.0, 0.03);
  ASSERT_NEAR(stdev_x, 0.1, 0.15);
  ASSERT_NEAR(mean_y, 0.0, 0.03);
  ASSERT_NEAR(stdev_y, 0.1, 0.15);
  ASSERT_NEAR(mean_z, 0.0, 0.0001);
}

TEST(test1, test_predict)
{
  ParticlesDistributionTest particle_dist;
  particle_dist.init();
  ASSERT_EQ(particle_dist.get_num_particles(), 200);

  tf2::Transform trans;
  trans.setOrigin(tf2::Vector3(1.0, 0.0, 0.0));
  trans.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));

  particle_dist.predict(trans);

  auto particles = particle_dist.get_particles();

  std::vector<double> pos_x(particles.size());
  std::vector<double> pos_y(particles.size());
  std::vector<double> pos_z(particles.size());

  for (int i = 0; i < particles.size(); i++)
  {
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
  ASSERT_NEAR(mean_y, 0.0, 0.02);
  ASSERT_NEAR(stdev_y, 0.1, 0.1);
  ASSERT_NEAR(mean_z, 0.0, 0.0001);
}

TEST(test1, test_get_tranform_to_read)
{
  ParticlesDistributionTest particle_dist;

  sensor_msgs::LaserScan scan;
  scan.header.frame_id = "laser";
  scan.header.stamp = ros::Time::now();
  scan.angle_min = -M_PI;
  scan.angle_max = M_PI;
  scan.angle_increment = M_PI_2;
  scan.ranges = {1.0, 2.0, 3.0, 4.0, 5.0};

  tf2::Transform point_0 = particle_dist.get_tranform_to_read_test(scan, 0);
  tf2::Transform point_1 = particle_dist.get_tranform_to_read_test(scan, 1);
  tf2::Transform point_2 = particle_dist.get_tranform_to_read_test(scan, 2);
  tf2::Transform point_3 = particle_dist.get_tranform_to_read_test(scan, 3);
  tf2::Transform point_4 = particle_dist.get_tranform_to_read_test(scan, 4);

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


TEST(test1, test_get_cost)
{
  // Costmap with an obstacle in front and left
  unsigned int size_x = 400;
  unsigned int size_y = 400;
  double resolution = 0.01;

  costmap_2d::Costmap2D costmap;
  costmap.resizeMap(size_x, size_y, resolution, -2.0, -2.0);

  unsigned int index = 0;
  // initialize the costmap with static data
  for (unsigned int i = 0; i < size_y; ++i)
  {
    for (unsigned int j = 0; j < size_x; ++j)
    {
      unsigned int x , y;
      costmap.indexToCells(index, x, y);
      costmap.setCost(x, y, costmap_2d::FREE_SPACE);
      ++index;
    }
  }  

  for (double x = -1.0; x < 1.0; x = x + (resolution / 2.0))
  {
    unsigned int mx , my;
    costmap.worldToMap(x, 1.0, mx, my);
    costmap.setCost(mx, my, costmap_2d::LETHAL_OBSTACLE);
    costmap.worldToMap(x, -0.5, mx, my);
    costmap.setCost(mx, my, costmap_2d::LETHAL_OBSTACLE);
  }

  for (double y = -0.5; y < 1.0; y = y + (resolution / 2.0))
  {
    unsigned int mx , my;
    costmap.worldToMap(0.75, y, mx, my);
    costmap.setCost(mx, my, costmap_2d::LETHAL_OBSTACLE);
  }

  // Init particles to (x=0.0, y=0.0, t=90.0)
  ParticlesDistributionTest particle_dist;
  tf2::Transform init_rot;
  init_rot.setOrigin({0.0, 0.0, 0.0});
  init_rot.setRotation({0.0, 0.0, 0.707, 0.707});
  particle_dist.init(init_rot);

  unsigned int mx , my;

  costmap.worldToMap(0.0, 0.0, mx, my);
  ASSERT_EQ(costmap.getCost(mx, my), costmap_2d::FREE_SPACE);  
  costmap.worldToMap(0.75, 0.0, mx, my);
  ASSERT_EQ(costmap.getCost(mx, my), costmap_2d::LETHAL_OBSTACLE);
  costmap.worldToMap(0.0, -0.5, mx, my);
  ASSERT_EQ(costmap.getCost(mx, my), costmap_2d::LETHAL_OBSTACLE);
  costmap.worldToMap(0.0, 1.0, mx, my);
  ASSERT_EQ(costmap.getCost(mx, my), costmap_2d::LETHAL_OBSTACLE);

  tf2::Transform tf_test;
  tf_test.setOrigin({0.0, 0.0, 0.0});
  ASSERT_EQ(particle_dist.get_cost_test(tf_test, costmap), costmap_2d::FREE_SPACE);
  tf_test.setOrigin({0.75, 0.0, 0.0});
  ASSERT_EQ(particle_dist.get_cost_test(tf_test, costmap), costmap_2d::LETHAL_OBSTACLE);
  tf_test.setOrigin({0.0, -0.5, 0.0});
  ASSERT_EQ(particle_dist.get_cost_test(tf_test, costmap), costmap_2d::LETHAL_OBSTACLE);
  tf_test.setOrigin({0.0, 1.0, 0.0});
  ASSERT_EQ(particle_dist.get_cost_test(tf_test, costmap), costmap_2d::LETHAL_OBSTACLE);
}

TEST(test1, test_get_error_distance_to_obstacle)
{
  // Costmap with an obstacle in front and left
  unsigned int size_x = 400;
  unsigned int size_y = 400;
  double resolution = 0.01;

  costmap_2d::Costmap2D costmap;
  costmap.resizeMap(size_x, size_y, resolution, -2.0, -2.0);

  unsigned int index = 0;
  // initialize the costmap with static data
  for (unsigned int i = 0; i < size_y; ++i)
  {
    for (unsigned int j = 0; j < size_x; ++j)
    {
      unsigned int x , y;
      costmap.indexToCells(index, x, y);
      costmap.setCost(x, y, costmap_2d::FREE_SPACE);
      ++index;
    }
  }  

  for (double x = -1.0; x < 1.0; x = x + (resolution / 2.0))
  {
    unsigned int mx , my;
    costmap.worldToMap(x, 1.0, mx, my);
    costmap.setCost(mx, my, costmap_2d::LETHAL_OBSTACLE);
    costmap.worldToMap(x, -0.5, mx, my);
    costmap.setCost(mx, my, costmap_2d::LETHAL_OBSTACLE);
  }

  for (double y = -0.5; y < 1.0; y = y + (resolution / 2.0))
  {
    unsigned int mx , my;
    costmap.worldToMap(0.75, y, mx, my);
    costmap.setCost(mx, my, costmap_2d::LETHAL_OBSTACLE);
  }


  // Transform base_footprint -> laser
  tf2_ros::StaticTransformBroadcaster tf_pub;
  geometry_msgs::TransformStamped bf2laser;
  bf2laser.header.stamp = ros::Time::now();
  bf2laser.header.frame_id = "base_footprint";
  bf2laser.child_frame_id = "laser";
  bf2laser.transform.rotation.w = 1.0;
  tf_pub.sendTransform({bf2laser});

  // Init particles to (x=0.0, y=0.0, t=90.0)
  ParticlesDistributionTest particle_dist;
  tf2::Transform init_rot;
  init_rot.setOrigin({0.0, 0.0, 0.0});
  init_rot.setRotation({0.0, 0.0, 0.707, 0.707});
  particle_dist.init(init_rot);

  // Spin for 1 sec to receive TFs
  ros::Rate rate(20);
  auto start = ros::Time::now();
  while ((ros::Time::now() - start).toSec() < 1.0)
  {
    ros::spinOnce();
    rate.sleep();
  }

  // Scan with an obstacle in (1, 0)
  sensor_msgs::LaserScan scan;
  scan.header.frame_id = "laser";
  scan.header.stamp = ros::Time::now();
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
    laser2point = particle_dist.get_tranform_to_read_test(scan, 0);

    ASSERT_NEAR(laser2point.getOrigin().x(), -0.54, 0.0001);
    ASSERT_NEAR(laser2point.getOrigin().y(), 0.00, 0.0001);

    double distance = particle_dist.get_error_distance_to_obstacle_test(map2bf, bf2lasert,
      laser2point, scan, costmap, 0.02);

    ASSERT_FALSE(std::isinf(distance));
    ASSERT_NEAR(distance, 0.05, 0.0001);
  }
  
  {
    tf2::Transform laser2point;
    laser2point = particle_dist.get_tranform_to_read_test(scan, 1);

    ASSERT_NEAR(laser2point.getOrigin().x(), 0.0, 0.0001);
    ASSERT_NEAR(laser2point.getOrigin().y(), -0.76, 0.0001);

    double distance = particle_dist.get_error_distance_to_obstacle_test(map2bf, bf2lasert,
      laser2point, scan, costmap, 0.02);

    ASSERT_FALSE(std::isinf(distance));
    ASSERT_NEAR(distance, 0.0, 0.0001);
  }

  {
    tf2::Transform laser2point;
    laser2point = particle_dist.get_tranform_to_read_test(scan, 2);

    ASSERT_NEAR(laser2point.getOrigin().x(), 1.0, 0.0001);
    ASSERT_NEAR(laser2point.getOrigin().y(), 0.0, 0.0001);

    double distance = particle_dist.get_error_distance_to_obstacle_test(map2bf, bf2lasert,
      laser2point, scan, costmap, 0.02);

    ASSERT_FALSE(std::isinf(distance));
    ASSERT_NEAR(distance, 0.0, 0.0001);
  }

  {
    tf2::Transform laser2point;
    laser2point = particle_dist.get_tranform_to_read_test(scan, 3);

    double distance = particle_dist.get_error_distance_to_obstacle_test(map2bf, bf2lasert,
      laser2point, scan, costmap, 0.02);

    ASSERT_TRUE(std::isinf(distance));
  }

  {
    tf2::Transform laser2point;
    laser2point = particle_dist.get_tranform_to_read_test(scan, 4);

    double distance = particle_dist.get_error_distance_to_obstacle_test(map2bf, bf2lasert,
      laser2point, scan, costmap, 0.02);

    ASSERT_TRUE(std::isinf(distance));
  }
}

TEST(test1, normalize)
{
  ParticlesDistributionTest particle_dist;
  particle_dist.init();

  auto & particles = particle_dist.get_particles();

  std::for_each(particles.begin(), particles.end(), [&](const aamcl::Particle &p) {
    ASSERT_NEAR(p.prob, 1.0 / particles.size(), 0.0001);
  });

  std::for_each(particles.begin(), particles.end(), [&](aamcl::Particle &p) {
    p.prob = p.prob * 2.0;
  });

  std::for_each(particles.begin(), particles.end(), [&](const aamcl::Particle &p) {
    ASSERT_NEAR(p.prob, 2.0 / particles.size(), 0.0001);
  });

  particle_dist.normalize_test();

  std::for_each(particles.begin(), particles.end(), [&](const aamcl::Particle &p) {
    ASSERT_NEAR(p.prob, 1.0 / particles.size(), 0.0001);
  });
}


TEST(test1, test_correct)
{
  // Costmap with an obstacle in (1, 0)
  unsigned int size_x = 400;
  unsigned int size_y = 400;
  double resolution = 0.01;

  costmap_2d::Costmap2D costmap;
  costmap.resizeMap(size_x, size_y, resolution, -2.0, -2.0);

  unsigned int index = 0;
  // initialize the costmap with static data
  for (unsigned int i = 0; i < size_y; ++i)
  {
    for (unsigned int j = 0; j < size_x; ++j)
    {
      unsigned int x , y;
      costmap.indexToCells(index, x, y);
      costmap.setCost(x, y, costmap_2d::FREE_SPACE);
      ++index;
    }
  }  

  for (double x = -1.0; x < 1.0; x = x + resolution)
  {
    unsigned int mx , my;
    costmap.worldToMap(x, 1.0, mx, my);
    costmap.setCost(mx, my, costmap_2d::LETHAL_OBSTACLE);
  }

  for (double y = -1.0; y < 1.0; y = y + resolution)
  {
    unsigned int mx , my;
    costmap.worldToMap(1.0, y, mx, my);
    costmap.setCost(mx, my, costmap_2d::LETHAL_OBSTACLE);
  }
  // Transform base_footprint -> laser
  tf2_ros::StaticTransformBroadcaster tf_pub;
  geometry_msgs::TransformStamped bf2laser;
  bf2laser.header.stamp = ros::Time::now();
  bf2laser.header.frame_id = "base_footprint";
  bf2laser.child_frame_id = "laser";
  bf2laser.transform.rotation.w = 1.0;
  tf_pub.sendTransform({bf2laser});

  // Init particles to (x=0.0, y=0.0, t=0.0)
  ParticlesDistributionTest particle_dist;
  particle_dist.init();

  // Spin for 1 sec to receive TFs
  ros::Rate rate(20);
  auto start = ros::Time::now();
  while ((ros::Time::now() - start).toSec() < 1.0)
  {
    ros::spinOnce();
    rate.sleep();
  }

  // Scan with an obstacle in (1, 0)
  sensor_msgs::LaserScan scan;
  scan.header.frame_id = "laser";
  scan.header.stamp = ros::Time::now();
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

  particle_dist.correct_once(scan, costmap);

  auto & particles = particle_dist.get_particles();
  // Sort particles by prob
  std::sort(particles.begin(), particles.end(),
   [](const aamcl::Particle & a, const aamcl::Particle & b) -> bool
  { 
    return a.prob > b.prob; 
  });

  for (int i = 0; i < 5; i++)
  {
    const aamcl::Particle & part = particles[i];

    const double x = part.pose.getOrigin().x();
    const double y = part.pose.getOrigin().y();
    
    double dist = sqrt(x * x + y * y);
    ASSERT_LE(dist, 0.05);
  }

}

int main(int argc, char *argv[])
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tester");
  // ros::NodeHandle nh;
  return RUN_ALL_TESTS();
  return 0;
}
