/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef MAV_PLANNING_MSGS_CONVERSIONS_H
#define MAV_PLANNING_MSGS_CONVERSIONS_H

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory_point.hpp>

#include <mav_trajectory_generation_ros2/msg/polynomial_segment.hpp>
#include <mav_trajectory_generation_ros2/msg/polynomial_trajectory.hpp>
#include "mav_trajectory_generation_ros2/eigen_planning_msgs.h"
#include <mav_trajectory_generation_ros2/eigen_mav_msgs.h>


// deprecated
#include "mav_trajectory_generation_ros2/conversions_deprecated.h"

namespace mav_planning_msgs {

/// Converts a PolynomialSegment double array to an Eigen::VectorXd.
inline void vectorFromMsgArray(const mav_trajectory_generation_ros2::msg::PolynomialSegment::_x_type& array,
                               Eigen::VectorXd* x) {
  *x = Eigen::Map<const Eigen::VectorXd>(&(array[0]), array.size());
}

/// Converts an Eigen::VectorXd to a PolynomialSegment double array.
inline void msgArrayFromVector(const Eigen::VectorXd& x,
                               mav_trajectory_generation_ros2::msg::PolynomialSegment::_x_type* array) {
  array->resize(x.size());
  Eigen::Map<Eigen::VectorXd> map =
      Eigen::Map<Eigen::VectorXd>(&((*array)[0]), array->size());
  map = x;
}

/// Converts a PolynomialSegment message to an EigenPolynomialSegment structure.
inline void eigenPolynomialSegmentFromMsg(const mav_trajectory_generation_ros2::msg::PolynomialSegment& msg,
                                          EigenPolynomialSegment* segment) {
  assert(segment != NULL);

  vectorFromMsgArray(msg.x, &(segment->x));
  vectorFromMsgArray(msg.y, &(segment->y));
  vectorFromMsgArray(msg.z, &(segment->z));
  vectorFromMsgArray(msg.yaw, &(segment->yaw));
  vectorFromMsgArray(msg.rx, &(segment->rx));
  vectorFromMsgArray(msg.ry, &(segment->ry));
  vectorFromMsgArray(msg.rz, &(segment->rz));

  segment->segment_time_ns = static_cast<uint64_t>(msg.segment_time.sec) * 1'000'000'000 + msg.segment_time.nanosec;
  segment->num_coeffs = msg.num_coeffs;
}

/// Converts a PolynomialTrajectory message to a EigenPolynomialTrajectory
inline void eigenPolynomialTrajectoryFromMsg(
    const mav_trajectory_generation_ros2::msg::PolynomialTrajectory& msg,
    EigenPolynomialTrajectory* eigen_trajectory) {
  assert(eigen_trajectory != NULL);
  eigen_trajectory->clear();
  eigen_trajectory->reserve(msg.segments.size());
  for (mav_trajectory_generation_ros2::msg::PolynomialTrajectory::_segments_type::const_iterator it =
           msg.segments.begin();
       it != msg.segments.end(); ++it) {
    EigenPolynomialSegment segment;
    eigenPolynomialSegmentFromMsg(*it, &segment);
    eigen_trajectory->push_back(segment);
  }
}

/// Converts an EigenPolynomialSegment to a PolynomialSegment message. Does NOT
/// set the header!
inline void polynomialSegmentMsgFromEigen(const EigenPolynomialSegment& segment,
                                          mav_trajectory_generation_ros2::msg::PolynomialSegment* msg) {
  assert(msg != NULL);
  msgArrayFromVector(segment.x, &(msg->x));
  msgArrayFromVector(segment.y, &(msg->y));
  msgArrayFromVector(segment.z, &(msg->z));
  msgArrayFromVector(segment.yaw, &(msg->yaw));
  msgArrayFromVector(segment.rx, &(msg->rx));
  msgArrayFromVector(segment.ry, &(msg->ry));
  msgArrayFromVector(segment.rz, &(msg->rz));

  msg->segment_time.sec = segment.segment_time_ns / 1'000'000'000;  // Convert total nanoseconds to seconds
  msg->segment_time.nanosec = segment.segment_time_ns % 1'000'000'000;  // Get the remaining nanoseconds
  msg->num_coeffs = segment.num_coeffs;
}

/// Converts an EigenPolynomialTrajectory to a PolynomialTrajectory message.
/// Does NOT set the header!
inline void polynomialTrajectoryMsgFromEigen(
    const EigenPolynomialTrajectory& eigen_trajectory,
    mav_trajectory_generation_ros2::msg::PolynomialTrajectory* msg) {
  assert(msg != NULL);
  msg->segments.reserve(eigen_trajectory.size());
  for (EigenPolynomialTrajectory::const_iterator it = eigen_trajectory.begin();
       it != eigen_trajectory.end(); ++it) {
    mav_trajectory_generation_ros2::msg::PolynomialSegment segment;
    polynomialSegmentMsgFromEigen(*it, &segment);
    msg->segments.push_back(segment);
  }
}

inline void msgMultiDofJointTrajectoryPointFromEigen(
    const mav_msgs::EigenTrajectoryPoint& trajectory_point,
    trajectory_msgs::msg::MultiDOFJointTrajectoryPoint* msg) {
  assert(msg != NULL);

  msg->time_from_start.sec = trajectory_point.time_from_start_ns / 1'000'000'000; 
  msg->time_from_start.nanosec = trajectory_point.time_from_start_ns % 1'000'000'000;
  msg->transforms.resize(1);
  msg->velocities.resize(1);
  msg->accelerations.resize(1);

  mav_msgs::vectorEigenToMsg(trajectory_point.position_W,
                   &msg->transforms[0].translation);
  mav_msgs::quaternionEigenToMsg(trajectory_point.orientation_W_B,
                       &msg->transforms[0].rotation);
  mav_msgs::vectorEigenToMsg(trajectory_point.velocity_W, &msg->velocities[0].linear);
  mav_msgs::vectorEigenToMsg(trajectory_point.angular_velocity_W,
                   &msg->velocities[0].angular);
  mav_msgs::vectorEigenToMsg(trajectory_point.acceleration_W,
                   &msg->accelerations[0].linear);
  mav_msgs::vectorEigenToMsg(trajectory_point.angular_acceleration_W,
                   &msg->accelerations[0].angular);
}

inline void msgMultiDofJointTrajectoryFromEigen(
    const mav_msgs::EigenTrajectoryPointVector& trajectory, const std::string& link_name,
    trajectory_msgs::msg::MultiDOFJointTrajectory* msg) {
  assert(msg != NULL);

  if (trajectory.empty()) {
    std::cout << "EigenTrajectoryPointVector is empty." << std::endl;
    return;
  }

  msg->joint_names.clear();
  msg->joint_names.push_back(link_name);
  msg->points.clear();

  for (const auto& trajectory_point : trajectory) {
    trajectory_msgs::msg::MultiDOFJointTrajectoryPoint point_msg;
    msgMultiDofJointTrajectoryPointFromEigen(trajectory_point, &point_msg);
    msg->points.push_back(point_msg);
  }
}

inline void msgMultiDofJointTrajectoryFromEigen(
    const mav_msgs::EigenTrajectoryPointVector& trajectory,
    trajectory_msgs::msg::MultiDOFJointTrajectory* msg) {
  msgMultiDofJointTrajectoryFromEigen(trajectory, "base_link", msg);
}

inline void msgMultiDofJointTrajectoryFromEigen(
    const mav_msgs::EigenTrajectoryPoint& trajectory_point, const std::string& link_name,
    trajectory_msgs::msg::MultiDOFJointTrajectory* msg) {
  assert(msg != NULL);
  trajectory_msgs::msg::MultiDOFJointTrajectoryPoint point_msg;
  msgMultiDofJointTrajectoryPointFromEigen(trajectory_point, &point_msg);

  msg->joint_names.clear();
  msg->points.clear();
  msg->joint_names.push_back(link_name);
  msg->points.push_back(point_msg);
}

inline void msgMultiDofJointTrajectoryFromEigen(
    const mav_msgs::EigenTrajectoryPoint& trajectory_point,
    trajectory_msgs::msg::MultiDOFJointTrajectory* msg) {
  msgMultiDofJointTrajectoryFromEigen(trajectory_point, "base_link", msg);
}

// inline void msgMultiDofJointTrajectoryFromEigen(
//     const mav_msgs::EigenTrajectoryPointVector& trajectory,
//     trajectory_msgs::msg::MultiDOFJointTrajectory* msg) {
//   msgMultiDofJointTrajectoryFromEigen(trajectory, "base_link", msg);
// }

}  // namespace mav_planning_msgs


#endif // MAV_PLANNING_MSGS_CONVERSIONS_H