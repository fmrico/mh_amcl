[![main](https://github.com/fmrico/mh_amcl/actions/workflows/main.yaml/badge.svg)](https://github.com/fmrico/mh_amcl/actions/workflows/main.yaml)

# Multi-Hypothesis AMCL (MH-AMCL)

[![](https://img.youtube.com/vi/LnmQ11Ew01g/0.jpg)](https://www.youtube.com/watch?v=LnmQ11Ew01g&feature=youtu.be "Click to play on You Tube")
[![](https://img.youtube.com/vi/RmY82ApjCMQ/0.jpg)](https://www.youtube.com/watch?v=RmY82ApjCMQ&feature=youtu.be "Click to play on You Tube")

MH-AMCL is a fully functional localization algorithm implementation with [Nav2](https://navigation.ros.org/). The main feature is that it maintains several hypotheses about the robot's position. The algorithm periodically generates new hypotheses on positions where the robot could be, based on the latest laser and map reading. This allows:
* Total unknown the position of the robot.
* Recover from erroneous estimates and hijacks.

## Build

This package requires the Rolling distribution, as it is in sync with Nav2, whose `main` branch uses Rolling.

Just clone this repo in the workspace and build as usual:

```
colcon build --symlink-install
```

## Run

We have included in this package launchers and other files that are usually in the `nav2_bringup` package in order to have a demo of its operation:

* To run in the simulation with a Turtlebot 3: `ros2 launch mh_amcl tb3_simulation_launch.py`
[![](https://img.youtube.com/vi/j0iQpAx-pbc/0.jpg)](https://www.youtube.com/watch?v=j0iQpAx-pbc&feature=youtu.be "Click to play on You Tube")


* To run in the simulation with a real robot (Tiago):
  * `ros2 launch mh_amcl tiago_launch.py`
  * If you don't have the robot, you can launch a demo ros2 bag with real data below: `ros2 bag play test/rosbag2_2022_09_01-11_42_10`
  
## Details

### Suscribed Topics:
* `scan` (`sensor_msgs/msg/LaserScan`): Laser readings.
* `map` (`nav_msgs/msg/OccupancyGrid`): The environmen map.
* `initialpose` (`geometry_msgs/msg/PoseWithCovarianceStamped`): Used for reset the robot's position from Rviz2.

### Published Topics:
* `amcl_pose` (`geometry_msgs::msg::PoseWithCovarianceStamped`): The robot's pose with the covariance associated from the best hypothesis.
* `particle_cloud` (`nav2_msgs::msg::ParticleCloud`): The particles from the best hypothesis.
* `poses` (`visualization_msgs::msg::MarkerArray`): All the particles from all the hypotheses, each one with a different color.

### Parameters:
* `use_sim_time` (bool, False): Use the robot's clock or the one coming from the `/clock` topic.
* `max_particles` (int, 200): The maximum number of particles for each hypothesis.
* `min_particles` (int, 200): The minimum number of particles for each hypothesis.
* `particles_step` (int, 30): Particles' variation increases `particles_step` when the estimation is bad and decreases when it is good.
* `init_pos_x` (double): The initial X position of the robot, if known.
* `init_pos_y` (double): The initial Y position of the robot, if known.
* `init_pos_yaw` (double): The initial Yaw position of the robot, if known.
* `init_error_x` (double): The initial X uncertainty of the robot.
* `init_error_y` (double): The initial Y uncertainty of the robot.
* `init_error_yaw` (double): The initial Yaw uncertainty of the robot.
* `translation_noise` (double, 10%): The error percentage coming from the translation component.
* `rotation_noise` (double, 10%): The error percentage from the rotational component.
* `rotation_noise` (double, 10%): The error percentage from the rotational component.
* `distance_perception_error` (double, 0.01): The error in meters of the sensor when reading distances.
* `reseed_percentage_losers` (double, 90%): The percentage of particles to be replaced when reseeding.
* `reseed_percentage_winners` (double, 3%): The percentage of particles that generate new particles when reseeding.
* `multihypothesis` (bool, true): Use multiples hypothesis, or only one - the created initially.
* `max_hypotheses` (int, 5): Maximum number of concurrent hypotheses.
* `min_candidate_weight` (float, 0.5): Minimum quality of a candidate to be considered for a new hypothesis.
* `min_candidate_distance` (double, 1.0): Minimum distance to an existing hypothesis to be considered for a new hypothesis.
* `min_candidate_angle` (double, PI/2): Minimum angle to an existing hypothesis to be considered for a new hypothesis.
* `low_q_hypo_thereshold` (float, 0.25): Under this threshold, a hypothesis is considered low quality and should be removed if there is a better candidate.
* `very_low_q_hypo_thereshold` (float, 0.10): A hypothesis is considered very low quality and should be removed under this threshold.
* `hypo_merge_distance` (double, 0.3): Distance under consideration to merge two hypotesese (angle and distance shpuld meet).
* `hypo_merge_angle` (double, 0.5): Angle to consider merging two hypotheses (angle and distance should meet).
* `good_hypo_thereshold` double, 0.5): Threshold to consider a hypothesis to be selected as the newly selected hypothesis for the algorithm's output.
* `min_hypo_diff_winner` double, 0.3): An hypothesis should have a quality `min_hypo_diff_winner` better than the currently selected hypothesis to be the newly selected hypothesis. Low values could lead to continuing changes between the two hypotheses. High values could make it impossible to consider other hypotheses.

## Citing

```
@INPROCEEDINGS{10160957,
  author={García, Alberto and Martín, Francisco and Guerrero, José Miguel and Rodríguez, Francisco J. and Matellán, Vicente},
  booktitle={2023 IEEE International Conference on Robotics and Automation (ICRA)}, 
  title={Portable Multi-Hypothesis Monte Carlo Localization for Mobile Robots}, 
  year={2023},
  volume={},
  number={},
  pages={1933-1939},
  doi={10.1109/ICRA48891.2023.10160957}}

```
