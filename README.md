# mav_trajectory_generation_ros2
This is a ROS 2 version of the original [mav_trajectory_generation](https://github.com/ethz-asl/mav_trajectory_generation/tree/master) repository.  This pkg is used to generate feasible multi-rotor trajectories. Not all the packages are ported to ROS 2, only the core package [mav_trajectory_generation](https://github.com/ethz-asl/mav_trajectory_generation/tree/master/mav_trajectory_generation)

# Dependencies
* NLopt : `sudo apt-get install libnlopt-dev`
* glog: `sudo apt-get install libgoogle-glog-dev`
* Eigen3
* `geometry_msgs`
* `yaml_cpp_vendor`

You can see the dependecies in the [package.xml](package.xml) file.

# Nodes

## go_to_waypoint_node
### Subscribers
* `odom` topic. Message type `nav_msgs::msg::Odometry`. This is the current odometry of the mav.
* `waypoint`. Message type `mav_trajectory_generation_ros2::msg::Waypoint`. The desired waypoint. This is a custom message [Waypoint.msg](msg/Waypoint.msg)

### Publishers
* `path_segments`. Message type `mav_trajectory_generation_ros2::msg::PolynomialTrajectory4D`. Custom message [PolynomialTrajectory4D.msg](msg/PolynomialTrajectory4D.msg). Publishes the generate trajectory segments which is consumed by a trajectory sampler.

* `waypoint_navigator_polynomial_markers`. Message type `visualization_msgs::msg::MarkerArray`. Publishes markers to visualize the path segments in RViz2.