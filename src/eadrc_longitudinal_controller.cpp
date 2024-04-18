// Copyright 2024 Marcel_Chudecki
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "eadrc_longitudinal_controller/eadrc_longitudinal_controller.hpp"

#include "motion_utils/trajectory/trajectory.hpp"
#include "tier4_autoware_utils/geometry/geometry.hpp"
#include "tier4_autoware_utils/math/normalization.hpp"

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <iostream>
#include <Eigen/Eigen>

namespace eadrc_longitudinal_controller
{

EadrcLongitudinalController::EadrcLongitudinalController(rclcpp::Node & node)
: node_parameters_(node.get_node_parameters_interface()),
  clock_(node.get_clock()),
  logger_(node.get_logger().get_child("longitudinal_controller")),
  diagnostic_updater_(&node)
{
  m_longitudinal_ctrl_period = node.get_parameter("ctrl_period").as_double();
}

int64_t EadrcLongitudinalController::foo(int64_t bar) const
{
  std::cout << "Hello World, " << bar << std::endl;
  return bar;
}

bool EadrcLongitudinalController::isReady(
  [[maybe_unused]] const trajectory_follower::InputData & input_data)
{
  return true;
}

void EadrcLongitudinalController::setTrajectory(
  const autoware_auto_planning_msgs::msg::Trajectory & msg)
{
  if (!longitudinal_utils::isValidTrajectory(msg)) {
    RCLCPP_ERROR_THROTTLE(logger_, *clock_, 3000, "received invalid trajectory. ignore.");
    return;
  }

  if (msg.points.size() < 2) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 3000, "Unexpected trajectory size < 2. Ignored.");
    return;
  }

  m_trajectory = msg;
}

void EadrcLongitudinalController::setKinematicState(const nav_msgs::msg::Odometry & msg)
{
  m_current_kinematic_state = msg;
}

void EadrcLongitudinalController::setCurrentAcceleration(
  const geometry_msgs::msg::AccelWithCovarianceStamped & msg)
{
  m_current_accel = msg;
}

void EadrcLongitudinalController::setCurrentOperationMode(const OperationModeState & msg)
{
  m_current_operation_mode = msg;
}

double EadrcLongitudinalController::getDt()
{
  double dt;
  if (!m_prev_control_time) {
    dt = m_longitudinal_ctrl_period;
    m_prev_control_time = std::make_shared<rclcpp::Time>(clock_->now());
  } else {
    dt = (clock_->now() - *m_prev_control_time).seconds();
    *m_prev_control_time = clock_->now();
  }
  const double max_dt = m_longitudinal_ctrl_period * 2.0;
  const double min_dt = m_longitudinal_ctrl_period * 0.5;
  return std::max(std::min(dt, max_dt), min_dt);
}

std::pair<autoware_auto_planning_msgs::msg::TrajectoryPoint, size_t>
EadrcLongitudinalController::calcInterpolatedTrajPointAndSegment(
  const autoware_auto_planning_msgs::msg::Trajectory & traj,
  const geometry_msgs::msg::Pose & pose) const
{
  if (traj.points.size() == 1) {
    return std::make_pair(traj.points.at(0), 0);
  }

  // apply linear interpolation
  return longitudinal_utils::lerpTrajectoryPoint(
    traj.points, pose, m_ego_nearest_dist_threshold, m_ego_nearest_yaw_threshold);
}

EadrcLongitudinalController::StateAfterDelay EadrcLongitudinalController::predictedStateAfterDelay(
  const Motion current_motion, const double delay_compensation_time) const
{
  const double current_vel = current_motion.vel;
  const double current_acc = current_motion.acc;
  double running_distance = 0.0;
  double pred_vel = current_vel;
  double pred_acc = current_acc;
  if (m_ctrl_cmd_vec.empty() || m_current_operation_mode.mode != OperationModeState::AUTONOMOUS) {
    // check time to stop
    const double time_to_stop = -current_vel / current_acc;
    const double delay_time_calculation =
      time_to_stop > 0.0 && time_to_stop < delay_compensation_time ? time_to_stop
                                                                   : delay_compensation_time;
    // simple linear prediction
    pred_vel = current_vel + current_acc * delay_time_calculation;
    running_distance = std::abs(
      delay_time_calculation * current_vel +
      0.5 * current_acc * delay_time_calculation * delay_time_calculation);
    // avoid to change sign of current_vel and pred_vel
    return StateAfterDelay{pred_vel, pred_acc, running_distance};
  }

  for (std::size_t i = 0; i < m_ctrl_cmd_vec.size(); ++i) {
    if ((clock_->now() - m_ctrl_cmd_vec.at(i).stamp).seconds() < delay_compensation_time) {
      // add velocity to accel * dt
      const double time_to_next_acc =
        (i == m_ctrl_cmd_vec.size() - 1)
          ? std::min(
              (clock_->now() - m_ctrl_cmd_vec.back().stamp).seconds(), delay_compensation_time)
          : std::min(
              (rclcpp::Time(m_ctrl_cmd_vec.at(i + 1).stamp) -
               rclcpp::Time(m_ctrl_cmd_vec.at(i).stamp))
                .seconds(),
              delay_compensation_time);
      const double acc = m_ctrl_cmd_vec.at(i).acceleration;
      // because acc_cmd is positive when vehicle is running backward
      pred_acc = std::copysignf(1.0, static_cast<float>(pred_vel)) * acc;
      running_distance += std::abs(
        std::abs(pred_vel) * time_to_next_acc + 0.5 * acc * time_to_next_acc * time_to_next_acc);
      pred_vel += pred_vel < 0.0 ? (-acc * time_to_next_acc) : (acc * time_to_next_acc);
      if (pred_vel / current_vel < 0.0) {
        // sign of velocity is changed
        pred_vel = 0.0;
        break;
      }
    }
  }

  return StateAfterDelay{pred_vel, pred_acc, running_distance};
}

enum EadrcLongitudinalController::Shift EadrcLongitudinalController::getCurrentShift(
  const ControlData & control_data) const
{
  constexpr double epsilon = 1e-5;

  const double target_vel =
    control_data.interpolated_traj.points.at(control_data.target_idx).longitudinal_velocity_mps;

  if (target_vel > epsilon) {
    return Shift::Forward;
  } else if (target_vel < -epsilon) {
    return Shift::Reverse;
  }

  return m_prev_shift;
}

EadrcLongitudinalController::ControlData EadrcLongitudinalController::getControlData(
  const geometry_msgs::msg::Pose & current_pose)
{
  ControlData control_data{};

  // dt
  control_data.dt = getDt();

  // current velocity and acceleration
  control_data.current_motion.vel = m_current_kinematic_state.twist.twist.linear.x;
  control_data.current_motion.acc = m_current_accel.accel.accel.linear.x;
  control_data.interpolated_traj = m_trajectory;

  // calculate the interpolated point and segment
  const auto current_interpolated_pose =
    calcInterpolatedTrajPointAndSegment(control_data.interpolated_traj, current_pose);

  // Insert the interpolated point
  control_data.interpolated_traj.points.insert(
    control_data.interpolated_traj.points.begin() + current_interpolated_pose.second + 1,
    current_interpolated_pose.first);
  control_data.nearest_idx = current_interpolated_pose.second + 1;
  control_data.target_idx = control_data.nearest_idx;
  const auto nearest_point = current_interpolated_pose.first;
  auto target_point = current_interpolated_pose.first;

  // check if the deviation is worth emergency
  m_diagnostic_data.trans_deviation =
    tier4_autoware_utils::calcDistance2d(current_interpolated_pose.first, current_pose);
  const bool is_dist_deviation_large =
    m_state_transition_params.emergency_state_traj_trans_dev < m_diagnostic_data.trans_deviation;
  m_diagnostic_data.rot_deviation = std::abs(tier4_autoware_utils::normalizeRadian(
    tf2::getYaw(current_interpolated_pose.first.pose.orientation) -
    tf2::getYaw(current_pose.orientation)));
  const bool is_yaw_deviation_large =
    m_state_transition_params.emergency_state_traj_rot_dev < m_diagnostic_data.rot_deviation;

  if (is_dist_deviation_large || is_yaw_deviation_large) {
    // return here if nearest index is not found
    control_data.is_far_from_trajectory = true;
    return control_data;
  }

  // Delay compensation - Calculate the distance we got, predicted velocity and predicted
  // acceleration after delay
  control_data.state_after_delay =
    predictedStateAfterDelay(control_data.current_motion, m_delay_compensation_time);

  // calculate the target motion for delay compensation
  constexpr double min_running_dist = 0.01;
  if (control_data.state_after_delay.running_distance > min_running_dist) {
    const auto target_pose = longitudinal_utils::findTrajectoryPoseAfterDistance(
      control_data.nearest_idx, control_data.state_after_delay.running_distance,
      control_data.interpolated_traj);
    const auto target_interpolated_point =
      calcInterpolatedTrajPointAndSegment(control_data.interpolated_traj, target_pose);
    control_data.target_idx = target_interpolated_point.second + 1;
    control_data.interpolated_traj.points.insert(
      control_data.interpolated_traj.points.begin() + control_data.target_idx,
      target_interpolated_point.first);
    target_point = target_interpolated_point.first;
  }
  // Remove overlapped points after inserting the interpolated points
  control_data.interpolated_traj.points =
    motion_utils::removeOverlapPoints(control_data.interpolated_traj.points);
  control_data.nearest_idx = motion_utils::findFirstNearestIndexWithSoftConstraints(
    control_data.interpolated_traj.points, nearest_point.pose, m_ego_nearest_dist_threshold,
    m_ego_nearest_yaw_threshold);
  control_data.target_idx = motion_utils::findFirstNearestIndexWithSoftConstraints(
    control_data.interpolated_traj.points, target_point.pose, m_ego_nearest_dist_threshold,
    m_ego_nearest_yaw_threshold);

  // send debug values
  m_debug_values.setValues(DebugValues::TYPE::PREDICTED_VEL, control_data.state_after_delay.vel);
  m_debug_values.setValues(
    DebugValues::TYPE::TARGET_VEL,
    control_data.interpolated_traj.points.at(control_data.target_idx).longitudinal_velocity_mps);

  // shift
  control_data.shift = getCurrentShift(control_data);
  if (control_data.shift != m_prev_shift) {
    m_eadrc_vel.reset();
  }
  m_prev_shift = control_data.shift;
}



trajectory_follower::LongitudinalOutput EadrcLongitudinalController::run(
  trajectory_follower::InputData const & input_data)
{
    // set input data
  setTrajectory(input_data.current_trajectory);
  setKinematicState(input_data.current_odometry);
  setCurrentAcceleration(input_data.current_accel);
  setCurrentOperationMode(input_data.current_operation_mode);

  // calculate current pose and control data
  geometry_msgs::msg::Pose current_pose = m_current_kinematic_state.pose.pose;

  const auto control_data = getControlData(current_pose);


  return output;
}

}  // namespace eadrc_longitudinal_controller
