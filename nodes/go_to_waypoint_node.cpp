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
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <mav_trajectory_generation_ros2/msg/waypoint.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory_point.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>
#include <mav_trajectory_generation_ros2/trajectory_sampling.h>

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

  // Maximum speed (m/s).
  double reference_speed_;
  // Maximum acceleration (m/s^2).
  double reference_acceleration_;

  bool got_odometry_;
  mav_msgs::EigenOdometry odometry_;

  // Polynomial trajectory markers.
  visualization_msgs::msg::MarkerArray markers_;

  // Path execution state (for pose publishing).
  size_t current_leg_;

  // A list of waypoints to visit.
  // [x,y,z,heading]
  std::vector<mav_msgs::EigenTrajectoryPoint> coarse_waypoints_;
  std::vector<mav_msgs::EigenTrajectoryPoint> reduced_coarse_waypoints_;

  // Path vertices and segments.
  mav_trajectory_generation::Trajectory polynomial_trajectory_;
  mav_trajectory_generation::Vertex::Vector polynomial_vertices_;
  mav_trajectory_generation::Trajectory yaw_trajectory_;
  mav_trajectory_generation::Vertex::Vector yaw_vertices_;

  trajectory_msgs::msg::MultiDOFJointTrajectory multi_dof_msg_;

  // Interpolates intermediate points between waypoints in a sequence.
  void addIntermediateWaypoints();

  // Adds current MAV position as a waypoint to the path.
  void addCurrentOdometryWaypoint();

  // Creates and optimizes a smooth polynomial trajectory from a waypoint list.
  void createTrajectory();
  void createReducedTrajectory();

  // Starts sending execution commands to the controller.
  void publishCommands();

  // Publishes the last trajectory sample constructed from reduced_coarse_waypoints_
  void publishTrajectorySample();

  // Deletes old polynomial trajectory markers.
  void deletePolynomialMarkers();

  void odometryCallback(const nav_msgs::msg::Odometry& odometry_message);
  void waypointCallback(const mav_trajectory_generation_ros2::msg::Waypoint& wp_msg);
  void multiDofCallback(const trajectory_msgs::msg::MultiDOFJointTrajectoryPoint& msg);

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<mav_trajectory_generation_ros2::msg::Waypoint>::SharedPtr waypoint_sub_;
  rclcpp::Subscription<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint>::SharedPtr multi_dof_wp_sub_;
    
  rclcpp::Publisher<mav_trajectory_generation_ros2::msg::PolynomialTrajectory4D>::SharedPtr  path_segments_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr polynomial_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>::SharedPtr command_pub_;

};

////////////////////////////////// Class definition ///////////////////////

// const double GoToWaypointNode::kCommandTimerFrequency = 5.0;
const double GoToWaypointNode::kWaypointAchievementDistance = 0.5;
const double GoToWaypointNode::kIntermediatePoseTolerance = 0.1;
const int GoToWaypointNode::kDimensions = 3;
const int GoToWaypointNode::kDerivativeToOptimize =
    mav_trajectory_generation::derivative_order::ACCELERATION;
const int GoToWaypointNode::kPolynomialCoefficients = 10;

GoToWaypointNode::GoToWaypointNode() : Node("go_to_waypoint_node"),
got_odometry_(false)
{

  // Get ros2 params
  // @todo Implement
  this->declare_parameter("heading_mode", "auto");
  heading_mode_ = this->get_parameter("heading_mode").get_parameter_value().get<std::string>();

  this->declare_parameter("intermediate_poses", true);
  intermediate_poses_ = this->get_parameter("intermediate_poses").get_parameter_value().get<bool>();

  this->declare_parameter("intermediate_pose_separation", 1.0); // meters
  intermediate_pose_separation_ = this->get_parameter("intermediate_pose_separation").get_parameter_value().get<double>();
  

  odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", rclcpp::SensorDataQoS(), std::bind(&GoToWaypointNode::odometryCallback, this, _1));

  waypoint_sub_ = this->create_subscription<mav_trajectory_generation_ros2::msg::Waypoint>(
      "waypoint", 10, std::bind(&GoToWaypointNode::waypointCallback, this, _1));

  multi_dof_wp_sub_ = this->create_subscription<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint>(
      "multi_dof_wp", 10, std::bind(&GoToWaypointNode::multiDofCallback, this, _1));

  path_segments_pub_ = this->create_publisher<mav_trajectory_generation_ros2::msg::PolynomialTrajectory4D>("path_segments_4D", 10);
  polynomial_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("waypoint_navigator_polynomial_markers", 10);
  command_pub_ = this->create_publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>("command/trajectory", 10);
  

  RCLCPP_INFO(this->get_logger(), "Node has started.");

}

GoToWaypointNode::~GoToWaypointNode() {/* Destructor*/}

void GoToWaypointNode::odometryCallback(const nav_msgs::msg::Odometry& odom_msg)
{
  // auto clock = this->get_clock();
  // RCLCPP_INFO_THROTTLE(this->get_logger(), *clock, 1000, "[GoToWaypointNode::odometryCallback] Got Odometry msg.");
  mav_msgs::eigenOdometryFromMsg(odom_msg, &odometry_);
  if (!got_odometry_) {
    got_odometry_ = true;
  }
}

void GoToWaypointNode::multiDofCallback(const trajectory_msgs::msg::MultiDOFJointTrajectoryPoint& msg)
{
  auto clock = this->get_clock();
  if (!got_odometry_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *clock, 1000, "[GoToWaypointNode::multiDofCallback] Did no get odometry. Not executing waypoint.");
    return;
  }

  // Make sure it is a new waypoint (new either in position OR heading)
  // For now, just comparing position
  Eigen::Vector3d des_pos = Eigen::Vector3d(msg.transforms[0].translation.x,
                                            msg.transforms[0].translation.y,
                                            msg.transforms[0].translation.z);


  coarse_waypoints_.clear();
  current_leg_ = 0;
  // timer_counter_ = 0;
  // command_timer_.stop();

  double d = (odometry_.position_W - des_pos).norm();
  if (d > kWaypointAchievementDistance)
  {
    addCurrentOdometryWaypoint();
  }

  // Add the new waypoint.
  mav_msgs::EigenTrajectoryPoint vwp;
  vwp.position_W.x() = msg.transforms[0].translation.x;
  vwp.position_W.y() = msg.transforms[0].translation.y;
  vwp.position_W.z() = msg.transforms[0].translation.z;
  auto acc_vec = mav_msgs::vector3FromMsg(msg.accelerations[0].linear);
  reference_acceleration_ = acc_vec.norm();
  auto vel_vec = mav_msgs::vector3FromMsg(msg.velocities[0].linear);
  reference_speed_ = vel_vec.norm();

  if (heading_mode_ == "zero") {
    vwp.setFromYaw(0.0);
  } else if (sqrt(pow(msg.transforms[0].translation.y - odometry_.position_W.y(), 2) +
                  pow(msg.transforms[0].translation.x - odometry_.position_W.x(), 2)) < 0.1) {
    vwp.orientation_W_B = odometry_.orientation_W_B;
  } else {
    vwp.setFromYaw(atan2(msg.transforms[0].translation.y - odometry_.position_W.y(),
                        msg.transforms[0].translation.x - odometry_.position_W.x()));
  }
  coarse_waypoints_.push_back(vwp);

  // Limit the maximum distance between waypoints.
  if (intermediate_poses_) {
    addIntermediateWaypoints();
  }

  // @todo 
  // crete trajectroy
  // Sample the trajectroy at the maximum time => corresponding to the lsat wp
  // publish the traj. point as multidofjoint for the ocntroller to consume

  // update reduced_coarse_waypoints_ to have only the current wp (initial) and next one from coarse_waypoints_, ignore the rest, if there is any
  if(coarse_waypoints_.size() < 2)
  {
    RCLCPP_WARN(this->get_logger(), "Goal is too close");
    // just pubish the last point
    // auto clock = this->get_clock();
    RCLCPP_WARN(this->get_logger(), " Publishing last command");
    command_pub_->publish(multi_dof_msg_);
    return;
  }
  reduced_coarse_waypoints_.resize(2);
  reduced_coarse_waypoints_.clear();
  reduced_coarse_waypoints_.push_back(coarse_waypoints_[0]);
  reduced_coarse_waypoints_.push_back(coarse_waypoints_[1]);


  // publishCommands();
  publishTrajectorySample();
  LOG(INFO) << "Going to a new waypoint...";
}

void GoToWaypointNode::waypointCallback(const mav_trajectory_generation_ros2::msg::Waypoint& wp_msg)
{
  if (!got_odometry_) {
    auto clock = this->get_clock();
    RCLCPP_WARN_THROTTLE(this->get_logger(), *clock, 1000, "[GoToWaypointNode::waypointCallback] Did no get odometry. Not executing waypoint.");
    return;
  }

  coarse_waypoints_.clear();
  current_leg_ = 0;
  // timer_counter_ = 0;
  // command_timer_.stop();

  addCurrentOdometryWaypoint();

  // Add the new waypoint.
  mav_msgs::EigenTrajectoryPoint vwp;
  vwp.position_W.x() = wp_msg.position.x;
  vwp.position_W.y() = wp_msg.position.y;
  vwp.position_W.z() = wp_msg.position.z;
  auto acc_vec = mav_msgs::vector3FromPointMsg(wp_msg.acceleration);
  reference_acceleration_ = acc_vec.norm();
  auto vel_vec = mav_msgs::vector3FromPointMsg(wp_msg.velocity);
  reference_speed_ = vel_vec.norm();

  if (heading_mode_ == "zero") {
    vwp.setFromYaw(0.0);
  } else if (sqrt(pow(wp_msg.position.y - odometry_.position_W.y(), 2) +
                  pow(wp_msg.position.x - odometry_.position_W.x(), 2)) < 0.05) {
    vwp.orientation_W_B = odometry_.orientation_W_B;
  } else {
    vwp.setFromYaw(atan2(wp_msg.position.y - odometry_.position_W.y(),
                        wp_msg.position.x - odometry_.position_W.x()));
  }
  coarse_waypoints_.push_back(vwp);

  // Limit the maximum distance between waypoints.
  if (intermediate_poses_) {
    addIntermediateWaypoints();
  }

  publishCommands();
  LOG(INFO) << "Going to a new waypoint...";
  
}

void GoToWaypointNode::publishTrajectorySample()
{
  createReducedTrajectory();
  // Publish the trajectory point to the controller
  mav_trajectory_generation::Trajectory traj_with_yaw;
  polynomial_trajectory_.getTrajectoryWithAppendedDimension(yaw_trajectory_,
                                                            &traj_with_yaw);
  // trajectory_msgs::msg::MultiDOFJointTrajectory multi_dof_msg;
  mav_msgs::EigenTrajectoryPoint trajectory_point;

  bool success = mav_trajectory_generation::sampleTrajectoryAtTime(
        traj_with_yaw, traj_with_yaw.getMaxTime(), &trajectory_point);
  if (!success)
    return;

  mav_planning_msgs::msgMultiDofJointTrajectoryFromEigen(trajectory_point, &multi_dof_msg_);
  // Convert time_from_start to seconds and nanoseconds
  int64_t seconds = static_cast<int64_t>(traj_with_yaw.getMaxTime());
  int64_t nanoseconds = static_cast<int64_t>((traj_with_yaw.getMaxTime() - seconds) * 1e9);
  multi_dof_msg_.points[0].time_from_start.sec = seconds;
  multi_dof_msg_.points[0].time_from_start.nanosec = nanoseconds;


  command_pub_->publish(multi_dof_msg_);
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
                            kDerivativeToOptimize);
      
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

void GoToWaypointNode::createReducedTrajectory()
{
  polynomial_vertices_.clear();
  polynomial_trajectory_.clear();
  yaw_vertices_.clear();
  yaw_trajectory_.clear();
  deletePolynomialMarkers();

  // NOTE: We expect reduced_coarse_waypoints_ to have only two waypoints
  if (reduced_coarse_waypoints_.size() != 2)
  {
    RCLCPP_WARN(this->get_logger(), "reduced_coarse_waypoints_ size must be exactly 2. reduced_coarse_waypoints_ size = %d", static_cast<int>( reduced_coarse_waypoints_.size()) );
    return;
  }

  mav_trajectory_generation::Vertex vertex(kDimensions);
  mav_trajectory_generation::Vertex yaw(1);

  // WP 0
  vertex.makeStartOrEnd(reduced_coarse_waypoints_[0].position_W,
                            kDerivativeToOptimize);
  polynomial_vertices_.push_back(vertex);

  // WP 1
  // if (coarse_waypoints_.size() == reduced_coarse_waypoints_.size()) // It's the end goal
  // {
  //   vertex.makeStartOrEnd(reduced_coarse_waypoints_[1].position_W,
  //                           kDerivativeToOptimize);
  // }
  // else // it is intermediate!
  // {
  //   vertex.addConstraint(
  //         mav_trajectory_generation::derivative_order::POSITION,
  //         reduced_coarse_waypoints_[1].position_W);
  // }
  vertex.makeStartOrEnd(reduced_coarse_waypoints_[1].position_W,
                            kDerivativeToOptimize);
  polynomial_vertices_.push_back(vertex);

  // yaw
  // Check whether to rotate clockwise or counter-clockwise in yaw.
  double yaw_mod = fmod(
      reduced_coarse_waypoints_[1].getYaw() - reduced_coarse_waypoints_[0].getYaw(),
      2 * M_PI);
  if (yaw_mod < -M_PI) {
    yaw_mod += 2 * M_PI;
  } else if (yaw_mod > M_PI) {
    yaw_mod -= 2 * M_PI;
  }
  reduced_coarse_waypoints_[1].setFromYaw(reduced_coarse_waypoints_[0].getYaw() +
                                  yaw_mod);
  // YAW of WP 0
  yaw.addConstraint(mav_trajectory_generation::derivative_order::ORIENTATION,
                      reduced_coarse_waypoints_[0].getYaw());
  yaw_vertices_.push_back(yaw);
  // Yaw of WP 1
  yaw.addConstraint(mav_trajectory_generation::derivative_order::ORIENTATION,
                      reduced_coarse_waypoints_[1].getYaw());
  yaw_vertices_.push_back(yaw);

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

void GoToWaypointNode::addIntermediateWaypoints()
{
  if(coarse_waypoints_.size() < 2)
    return;

  for (size_t i = 1; i < coarse_waypoints_.size(); ++i) {
    mav_msgs::EigenTrajectoryPoint wpa = coarse_waypoints_[i - 1];
    mav_msgs::EigenTrajectoryPoint wpb = coarse_waypoints_[i];
    double dist = (wpa.position_W - wpb.position_W).norm();

    // Minimum tolerance between points set to avoid subsequent numerical errors
    // in trajectory optimization.
    while (dist > intermediate_pose_separation_ &&
           dist > kIntermediatePoseTolerance) {
      mav_msgs::EigenTrajectoryPoint iwp;
      iwp.position_W.x() = wpa.position_W.x() +
                           (intermediate_pose_separation_ / dist) *
                               (wpb.position_W.x() - wpa.position_W.x());
      iwp.position_W.y() = wpa.position_W.y() +
                           (intermediate_pose_separation_ / dist) *
                               (wpb.position_W.y() - wpa.position_W.y());
      iwp.position_W.z() = wpa.position_W.z() +
                           (intermediate_pose_separation_ / dist) *
                               (wpb.position_W.z() - wpa.position_W.z());
      iwp.orientation_W_B = wpb.orientation_W_B;
      coarse_waypoints_.insert(coarse_waypoints_.begin() + i, iwp);
      wpa = iwp;
      dist = (wpa.position_W - wpb.position_W).norm();
      i++;
    }
  }
}

void GoToWaypointNode::addCurrentOdometryWaypoint()
{
  mav_msgs::EigenTrajectoryPoint vwp;
  vwp.position_W = odometry_.position_W;
  vwp.orientation_W_B = odometry_.orientation_W_B;
  coarse_waypoints_.push_back(vwp);
}
void GoToWaypointNode::deletePolynomialMarkers()
{
    for (size_t i = 0; i < markers_.markers.size(); ++i) {
    markers_.markers[i].action = visualization_msgs::msg::Marker::DELETE;
  }
  polynomial_pub_->publish(markers_);
}

/**
 * Main function
*/

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoToWaypointNode>());
  rclcpp::shutdown();
  return 0;
}