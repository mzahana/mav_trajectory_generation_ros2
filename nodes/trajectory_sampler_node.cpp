#include <Eigen/Eigen>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"

#include <mav_trajectory_generation_ros2/conversions.h>
#include <mav_trajectory_generation_ros2/eigen_mav_msgs.h>
#include <mav_trajectory_generation_ros2/msg/polynomial_segment.hpp>
#include <mav_trajectory_generation_ros2/msg/polynomial_trajectory.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>

#include <mav_trajectory_generation_ros2/polynomial.h>
#include <mav_trajectory_generation_ros2/trajectory_sampling.h>
#include <mav_trajectory_generation_ros2/ros_conversions.h>

#include <geometry_msgs/msg/point.hpp>

// deprecated
#include <mav_trajectory_generation_ros2/msg/polynomial_segment4_d.hpp>
#include <mav_trajectory_generation_ros2/msg/polynomial_trajectory4_d.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;

class TrajectorySamplerNode : public rclcpp::Node
{
 public:
  TrajectorySamplerNode();
  ~TrajectorySamplerNode();

 private:
  void pathSegmentsCallback(
      const mav_trajectory_generation_ros2::msg::PolynomialTrajectory& segments_message);
  void pathSegments4DCallback(
      const mav_trajectory_generation_ros2::msg::PolynomialTrajectory4D& segments_message);
  // bool stopSamplingCallback(std_srvs::Empty::Request& request,
  //                           std_srvs::Empty::Response& response);
  // void commandTimerCallback(const ros::TimerEvent&);
  void processTrajectory();

  void commandTimerCallback();

  void odomCallback(const nav_msgs::msg::Odometry& msg);

  void startTimer();

  rclcpp::Subscription<mav_trajectory_generation_ros2::msg::PolynomialTrajectory>::SharedPtr trajectory_sub_;
  rclcpp::Subscription<mav_trajectory_generation_ros2::msg::PolynomialTrajectory4D>::SharedPtr trajectory4D_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  rclcpp::Publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>::SharedPtr command_pub_;

  rclcpp::TimerBase::SharedPtr timer_;
  bool run_timer_;

  // This to keep publishing the lsat sample in the trajectory
  // to prevent the controller from timing out.
  // So the drone does not drift, when the controller times out,
  // and does not publish commands to the lower level dorne contorllers
  bool hold_last_sample_;

  // ros::ServiceServer stop_srv_;
  rclcpp::Time start_time_;

  // Service client for getting the MAV interface to listen to our sent
  // commands.
  // ros::ServiceClient position_hold_client_;

  // Flag whether to publish entire trajectory at once or not.
  bool publish_whole_trajectory_;
  // Trajectory sampling interval.
  double dt_;
  // Time at currently published trajectory sample.
  double current_sample_time_;

  // The trajectory to sub-sample.
  mav_trajectory_generation::Trajectory trajectory_;

  Eigen::Vector3d current_position_;
};

/////////////////////// Class definition ////////////////////////

TrajectorySamplerNode::TrajectorySamplerNode(): Node("trjectory_sampler_node"),
  run_timer_(false),
  hold_last_sample_(true),
  publish_whole_trajectory_(false),
  dt_(0.01),
  current_sample_time_(0.0),
  current_position_(Eigen::Vector3d::Zero())
{
  this->declare_parameter("publish_whole_trajectory", false);
  publish_whole_trajectory_ = this->get_parameter("publish_whole_trajectory").get_parameter_value().get<bool>();

  this->declare_parameter("command_frequency", 0.01);
  dt_ = this->get_parameter("command_frequency").get_parameter_value().get<double>();

  this->declare_parameter("hold_last_sample", true);
  hold_last_sample_ = this->get_parameter("hold_last_sample").get_parameter_value().get<bool>();

  trajectory_sub_ = this->create_subscription<mav_trajectory_generation_ros2::msg::PolynomialTrajectory>(
      "path_segments", rclcpp::SensorDataQoS(), std::bind(&TrajectorySamplerNode::pathSegmentsCallback, this, _1)); 
  trajectory4D_sub_ = this->create_subscription<mav_trajectory_generation_ros2::msg::PolynomialTrajectory4D>(
      "path_segments_4D", rclcpp::SensorDataQoS(), std::bind(&TrajectorySamplerNode::pathSegments4DCallback, this, _1));

  command_pub_ = this->create_publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>("command/trajectory", 10);

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", rclcpp::SensorDataQoS(), std::bind(&TrajectorySamplerNode::odomCallback, this, _1));

  // timer_ = this->create_wall_timer(
  //           std::chrono::milliseconds(static_cast<int64_t>(dt_*1000)),
  //           std::bind(&TrajectorySamplerNode::commandTimerCallback, this) );
}

TrajectorySamplerNode::~TrajectorySamplerNode(){ timer_->cancel(); }

void TrajectorySamplerNode::odomCallback(const nav_msgs::msg::Odometry& msg)
{
  current_position_(0) = msg.pose.pose.position.x;
  current_position_(1) = msg.pose.pose.position.y;
  current_position_(2) = msg.pose.pose.position.z;
}

void TrajectorySamplerNode::pathSegments4DCallback(
      const mav_trajectory_generation_ros2::msg::PolynomialTrajectory4D& segments_message)
{
  if (segments_message.segments.empty()) {
    RCLCPP_WARN(this->get_logger(),"Trajectory sampler: received empty waypoint message");
    return;
  } else {
    RCLCPP_INFO(this->get_logger(),"Trajectory sampler: received %lu waypoints",
             segments_message.segments.size());
  }

    bool success = mav_trajectory_generation::polynomialTrajectoryMsgToTrajectory(
        segments_message, &trajectory_);
    if (!success) {
      return;
    }
    processTrajectory();
}

void TrajectorySamplerNode::pathSegmentsCallback(
      const mav_trajectory_generation_ros2::msg::PolynomialTrajectory& segments_message)
{
  if (segments_message.segments.empty()) {
    RCLCPP_WARN(this->get_logger(), "Trajectory sampler: received empty waypoint message");
    return;
  } else {
    RCLCPP_INFO(this->get_logger(), "Trajectory sampler: received %lu waypoints",
             segments_message.segments.size());
  }

    bool success = mav_trajectory_generation::polynomialTrajectoryMsgToTrajectory(
        segments_message, &trajectory_);
    if (!success) {
      return;
    }
    processTrajectory();
}

void TrajectorySamplerNode::processTrajectory()
{
  // Call the service call to takeover publishing commands.
  // if (position_hold_client_.exists()) {
  //   std_srvs::Empty empty_call;
  //   position_hold_client_.call(empty_call);
  // }

  if (publish_whole_trajectory_) {
    // Publish the entire trajectory at once.
    mav_msgs::EigenTrajectoryPoint::Vector trajectory_points;
    mav_trajectory_generation::sampleWholeTrajectory(trajectory_, dt_,
                                                     &trajectory_points);
    trajectory_msgs::msg::MultiDOFJointTrajectory msg_pub;
    mav_planning_msgs::msgMultiDofJointTrajectoryFromEigen(trajectory_points, &msg_pub);
    command_pub_->publish(msg_pub);
  } else {
    // publish_timer_.start();
    current_sample_time_ = 0.0;
    start_time_ = this->now();
    startTimer();
    // run_timer_ = true;    
  }
}

void TrajectorySamplerNode::commandTimerCallback()
{
  // if(!run_timer_)
  //   return;

  trajectory_msgs::msg::MultiDOFJointTrajectory msg;
  mav_msgs::EigenTrajectoryPoint trajectory_point;
  if (current_sample_time_ <= trajectory_.getMaxTime()) {
    // trajectory_msgs::msg::MultiDOFJointTrajectory msg;
    // mav_msgs::EigenTrajectoryPoint trajectory_point;
    bool success = mav_trajectory_generation::sampleTrajectoryAtTime(
        trajectory_, current_sample_time_, &trajectory_point);
    if (!success) {
      // publish_timer_.stop();
      // run_timer_ = false;
      timer_->cancel();
      return;
    }
    mav_planning_msgs::msgMultiDofJointTrajectoryFromEigen(trajectory_point, &msg);
    // Convert time_from_start to seconds and nanoseconds
    int64_t seconds = static_cast<int64_t>(current_sample_time_);
    int64_t nanoseconds = static_cast<int64_t>((current_sample_time_ - seconds) * 1e9);
    msg.points[0].time_from_start.sec = seconds;
    msg.points[0].time_from_start.nanosec = nanoseconds;

    command_pub_->publish(msg);
    if( (trajectory_point.position_W - current_position_).norm() < 0.1 )
      current_sample_time_ += dt_;
    
  }
  else 
  {
    if(!hold_last_sample_){ timer_->cancel(); return ;}
    bool success = mav_trajectory_generation::sampleTrajectoryAtTime(
        trajectory_, trajectory_.getMaxTime(), &trajectory_point);
    if (!success) {
      // publish_timer_.stop();
      // run_timer_ = false;
      timer_->cancel();
      return;
    }
    mav_planning_msgs::msgMultiDofJointTrajectoryFromEigen(trajectory_point, &msg);
    // Convert time_from_start to seconds and nanoseconds
    int64_t seconds = static_cast<int64_t>(trajectory_.getMaxTime());
    int64_t nanoseconds = static_cast<int64_t>((trajectory_.getMaxTime() - seconds) * 1e9);
    msg.points[0].time_from_start.sec = seconds;
    msg.points[0].time_from_start.nanosec = nanoseconds;

    auto clock = this->get_clock();
    RCLCPP_INFO_THROTTLE(this->get_logger(), *clock, 1000, "[TrajectorySamplerNode] Holding last sample.");
    msg.points[0].accelerations[0].linear.x=0.0;
    msg.points[0].accelerations[0].linear.y=0.0;
    msg.points[0].accelerations[0].linear.z=0.0;
    msg.points[0].velocities[0].linear.x=0.0;
    msg.points[0].velocities[0].linear.y=0.0;
    msg.points[0].velocities[0].linear.z=0.0;
    msg.points[0].velocities[0].angular.x = 0.0;
    msg.points[0].velocities[0].angular.y = 0.0;
    msg.points[0].velocities[0].angular.z = 0.0;
    command_pub_->publish(msg);    
  }

}

void TrajectorySamplerNode::startTimer()
{
    // Create a timer that triggers after 2 seconds
    timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int64_t>(dt_*1000)), 
                                      std::bind(&TrajectorySamplerNode::commandTimerCallback, this));
}

/**
 * Main function
*/

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectorySamplerNode>());
  rclcpp::shutdown();
  return 0;
}