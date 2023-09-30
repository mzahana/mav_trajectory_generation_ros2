#include <mav_trajectory_generation_ros2/polynomial_optimization_linear.h>
#include <mav_trajectory_generation_ros2/trajectory.h>
#include <Eigen/Eigen>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <mav_trajectory_generation_ros2/eigen_mav_msgs.h>
#include <geometry_msgs/msg/point.hpp>
#include <mav_trajectory_generation_ros2/msg/polynomial_trajectory4_d.hpp>
#include <mav_trajectory_generation_ros2/ros_conversions.h>

using namespace std::chrono_literals;
using std::placeholders::_1;

class GoToWaypointNode : public rclcpp::Node
{
public:
  GoToWaypointNode();
  ~GoToWaypointNode();

private:

  static const double kCommandTimerFrequency;
  // Distance before a waypoint is considered reached [m].
  static const double kWaypointAchievementDistance;
  // Minimum distance between intermediate waypoints [m].
  static const double kIntermediatePoseTolerance;
  // Number of dimensions in the problem.
  static const int kDimensions;
  // Order of derivative to optimize polynomial trajectory for.
  static const int kDerivativeToOptimize;
  // Number of coefficients of polynomial trajectory.
  static const int kPolynomialCoefficients;

  // Heading alignment method.
  std::string heading_mode_;
  
  std::string frame_id_;

  // Addition of intermediate command poses.
  bool intermediate_poses_;
  // Maximum distance between poses [m].
  double intermediate_pose_separation_;

  bool got_odometry_;
  mav_msgs::EigenOdometry odometry_;

  // Path execution state (for pose publishing).
  size_t current_leg_;

  // A list of waypoints to visit.
  // [x,y,z,heading]
  std::vector<mav_msgs::EigenTrajectoryPoint> coarse_waypoints_;

  // Path vertices and segments.
  mav_trajectory_generation::Trajectory polynomial_trajectory_;
  mav_trajectory_generation::Vertex::Vector polynomial_vertices_;
  mav_trajectory_generation::Trajectory yaw_trajectory_;
  mav_trajectory_generation::Vertex::Vector yaw_vertices_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;

  // Interpolates intermediate points between waypoints in a sequence.
  void addIntermediateWaypoints();

  // Adds current MAV position as a waypoint to the path.
  void addCurrentOdometryWaypoint();

  // Creates and optimizes a smooth polynomial trajectory from a waypoint list.
  void createTrajectory();

  // Starts sending execution commands to the controller.
  void publishCommands();

  // Deletes old polynomial trajectory markers.
  void deletePolynomialMarkers();

  void odometryCallback(const nav_msgs::msg::Odometry& odometry_message);
  void waypointCallback(const geometry_msgs::msg::Point& point_msg);

  rclcpp::Publisher<mav_trajectory_generation_ros2::msg::PolynomialTrajectory4D>::SharedPtr  path_segments_pub_;

};

////////////////////////////////// Class definition ///////////////////////
GoToWaypointNode::GoToWaypointNode() : Node("go_to_waypoint_node"),
got_odometry_(false)
{

  // Get ros2 params

  odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", rclcpp::SensorDataQoS(), std::bind(&GoToWaypointNode::odometryCallback, this, _1));

  path_segments_pub_ = this->create_publisher<mav_trajectory_generation_ros2::msg::PolynomialTrajectory4D>("path_segments", 10);

}

GoToWaypointNode::~GoToWaypointNode() {/* Destructor*/}

void GoToWaypointNode::waypointCallback(const geometry_msgs::msg::Point& point_msg)
{
  coarse_waypoints_.clear();
  current_leg_ = 0;
  // timer_counter_ = 0;
  // command_timer_.stop();

  addCurrentOdometryWaypoint();

  // Add the new waypoint.
  mav_msgs::EigenTrajectoryPoint vwp;
  vwp.position_W.x() = point_msg.x;
  vwp.position_W.y() = point_msg.y;
  vwp.position_W.z() = point_msg.z;
  if (heading_mode_ == "zero") {
    vwp.setFromYaw(0.0);
  } else if (sqrt(pow(point_msg.y - odometry_.position_W.y(), 2) +
                  pow(point_msg.x - odometry_.position_W.x(), 2)) < 0.05) {
    vwp.orientation_W_B = odometry_.orientation_W_B;
  } else {
    vwp.setFromYaw(atan2(point_msg.y - odometry_.position_W.y(),
                        point_msg.x - odometry_.position_W.x()));
  }
  coarse_waypoints_.push_back(vwp);

  // Limit the maximum distance between waypoints.
  if (intermediate_poses_) {
    addIntermediateWaypoints();
  }

  publishCommands();
  LOG(INFO) << "Going to a new waypoint...";
  
}

void GoToWaypointNode::publishCommands()
{

  createTrajectory();
  // Publish the trajectory directly to the trajectory sampler.
  mav_trajectory_generation_ros2::msg::PolynomialTrajectory4D msg;
  mav_trajectory_generation::Trajectory traj_with_yaw;
  polynomial_trajectory_.getTrajectoryWithAppendedDimension(yaw_trajectory_,
                                                            &traj_with_yaw);
  mav_trajectory_generation::trajectoryToPolynomialTrajectoryMsg(
      traj_with_yaw, &msg);
  path_segments_pub_->publish(msg);

}

void GoToWaypointNode::createTrajectory()
{
  polynomial_vertices_.clear();
  polynomial_trajectory_.clear();
  yaw_vertices_.clear();
  yaw_trajectory_.clear();
  deletePolynomialMarkers();

  // Create a list of vertices.
  for (size_t i = 0; i < coarse_waypoints_.size(); i++) {
    mav_trajectory_generation::Vertex vertex(kDimensions);
    mav_trajectory_generation::Vertex yaw(1);

    // Position.
    if (i == 0 || i == coarse_waypoints_.size() - 1) {
      vertex.makeStartOrEnd(coarse_waypoints_[i].position_W,
                            mav_trajectory_generation::derivative_order::SNAP);
    } else {
      vertex.addConstraint(
          mav_trajectory_generation::derivative_order::POSITION,
          coarse_waypoints_[i].position_W);
    }
    // Yaw.
    if (i != 0) {
      // Check whether to rotate clockwise or counter-clockwise in yaw.
      double yaw_mod = fmod(
          coarse_waypoints_[i].getYaw() - coarse_waypoints_[i - 1].getYaw(),
          2 * M_PI);
      if (yaw_mod < -M_PI) {
        yaw_mod += 2 * M_PI;
      } else if (yaw_mod > M_PI) {
        yaw_mod -= 2 * M_PI;
      }
      coarse_waypoints_[i].setFromYaw(coarse_waypoints_[i - 1].getYaw() +
                                      yaw_mod);
    }
    yaw.addConstraint(mav_trajectory_generation::derivative_order::ORIENTATION,
                      coarse_waypoints_[i].getYaw());

    polynomial_vertices_.push_back(vertex);
    yaw_vertices_.push_back(yaw);
  }

  // Optimize the polynomial trajectory.
  // Position.
  std::vector<double> segment_times;
  segment_times =
      estimateSegmentTimes(polynomial_vertices_, reference_speed_,
                           reference_acceleration_);

  mav_trajectory_generation::PolynomialOptimization<kPolynomialCoefficients>
      opt(kDimensions);
  opt.setupFromVertices(polynomial_vertices_, segment_times,
                        kDerivativeToOptimize);
  opt.solveLinear();
  opt.getTrajectory(&polynomial_trajectory_);
  // Yaw.
  mav_trajectory_generation::PolynomialOptimization<kPolynomialCoefficients>
      yaw_opt(1);
  yaw_opt.setupFromVertices(yaw_vertices_, segment_times,
                            kDerivativeToOptimize);
  yaw_opt.solveLinear();
  yaw_opt.getTrajectory(&yaw_trajectory_);
}


void GoToWaypointNode::deletePolynomialMarkers()
{
  //@todo implement
}