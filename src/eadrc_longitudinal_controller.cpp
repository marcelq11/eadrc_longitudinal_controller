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

namespace autoware::motion::control::eadrc_longitudinal_controller
{
EadrcLongitudinalController::EadrcLongitudinalController(rclcpp::Node & node)
: node_parameters_(node.get_node_parameters_interface()),
  clock_(node.get_clock()),
  logger_(node.get_logger().get_child("longitudinal_controller")),
  diagnostic_updater_(&node)
{
  m_longitudinal_ctrl_period = node.get_parameter("ctrl_period").as_double();

// parameters for delay compensation
  m_delay_compensation_time = node.declare_parameter<double>("delay_compensation_time");  // [s] 

  p_obserwer = std::make_shared<eadrc_longitudinal_controller::ESO>(1,15);
  m_kp=2;
  m_b_hat=1;
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

void EadrcLongitudinalController::updatePitchDebugValues(
  const double pitch, const double traj_pitch, const double raw_pitch)
{
  const double to_degrees = (180.0 / static_cast<double>(M_PI));
  m_debug_values.setValues(DebugValues::TYPE::PITCH_LPF_RAD, pitch);
  m_debug_values.setValues(DebugValues::TYPE::PITCH_LPF_DEG, pitch * to_degrees);
  m_debug_values.setValues(DebugValues::TYPE::PITCH_RAW_RAD, raw_pitch);
  m_debug_values.setValues(DebugValues::TYPE::PITCH_RAW_DEG, raw_pitch * to_degrees);
  m_debug_values.setValues(DebugValues::TYPE::PITCH_RAW_TRAJ_RAD, traj_pitch);
  m_debug_values.setValues(DebugValues::TYPE::PITCH_RAW_TRAJ_DEG, traj_pitch * to_degrees);
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
    // m_pid_vel.reset();
  }
  m_prev_shift = control_data.shift;

  // distance to stopline
  control_data.stop_dist = longitudinal_utils::calcStopDistance(
    control_data.interpolated_traj.points.at(control_data.nearest_idx).pose,
    control_data.interpolated_traj, m_ego_nearest_dist_threshold, m_ego_nearest_yaw_threshold);

  // pitch
  // NOTE: getPitchByTraj() calculates the pitch angle as defined in
  // ../media/slope_definition.drawio.svg while getPitchByPose() is not, so `raw_pitch` is reversed
  const double raw_pitch = (-1.0) * longitudinal_utils::getPitchByPose(current_pose.orientation);
  const double traj_pitch = longitudinal_utils::getPitchByTraj(
    control_data.interpolated_traj, control_data.target_idx, m_wheel_base);

  if (m_slope_source == SlopeSource::RAW_PITCH) {
    control_data.slope_angle = m_lpf_pitch->filter(raw_pitch);
  } else if (m_slope_source == SlopeSource::TRAJECTORY_PITCH) {
    control_data.slope_angle = traj_pitch;
  } else if (m_slope_source == SlopeSource::TRAJECTORY_ADAPTIVE) {
    // if velocity is high, use target idx for slope, otherwise, use raw_pitch
    if (control_data.current_motion.vel > m_adaptive_trajectory_velocity_th) {
      control_data.slope_angle = traj_pitch;
      m_lpf_pitch->filter(raw_pitch);
    } else {
      control_data.slope_angle = m_lpf_pitch->filter(raw_pitch);
    }
  } else {
    RCLCPP_ERROR_THROTTLE(
      logger_, *clock_, 3000, "Slope source is not valid. Using raw_pitch option as default");
    control_data.slope_angle = m_lpf_pitch->filter(raw_pitch);
  }

  updatePitchDebugValues(control_data.slope_angle, traj_pitch, raw_pitch);

  return control_data;
}

EadrcLongitudinalController::Motion EadrcLongitudinalController::calcEmergencyCtrlCmd(
  const double dt) const
{
  // These accelerations are without slope compensation
  const auto & p = m_emergency_state_params;
  const double vel =
    longitudinal_utils::applyDiffLimitFilter(p.vel, m_prev_raw_ctrl_cmd.vel, dt, p.acc);
  const double acc =
    longitudinal_utils::applyDiffLimitFilter(p.acc, m_prev_raw_ctrl_cmd.acc, dt, p.jerk);

  RCLCPP_ERROR_THROTTLE(
    logger_, *clock_, 3000, "[Emergency stop] vel: %3.3f, acc: %3.3f", vel, acc);

  return Motion{vel, acc};
}

// Do not use nearest_idx here
autoware_auto_control_msgs::msg::LongitudinalCommand EadrcLongitudinalController::createCtrlCmdMsg(
  const Motion & ctrl_cmd, const double & current_vel)
{
  // publish control command
  autoware_auto_control_msgs::msg::LongitudinalCommand cmd{};
  cmd.stamp = clock_->now();
  cmd.speed = static_cast<decltype(cmd.speed)>(ctrl_cmd.vel);
  cmd.acceleration = static_cast<decltype(cmd.acceleration)>(ctrl_cmd.acc);

  // store current velocity history
  m_vel_hist.push_back({clock_->now(), current_vel});
  while (m_vel_hist.size() >
         static_cast<size_t>(m_delay_compensation_time / m_longitudinal_ctrl_period)) {
    m_vel_hist.erase(m_vel_hist.begin());
  }

  m_prev_ctrl_cmd = ctrl_cmd;

  return cmd;
}

void EadrcLongitudinalController::publishDebugData(
  const Motion & ctrl_cmd, const ControlData & control_data)
{
  // set debug values
  m_debug_values.setValues(DebugValues::TYPE::DT, control_data.dt);
  m_debug_values.setValues(DebugValues::TYPE::CALCULATED_ACC, control_data.current_motion.acc);
  m_debug_values.setValues(DebugValues::TYPE::SHIFT, static_cast<double>(control_data.shift));
  m_debug_values.setValues(DebugValues::TYPE::STOP_DIST, control_data.stop_dist);
  m_debug_values.setValues(DebugValues::TYPE::CONTROL_STATE, static_cast<double>(m_control_state));
  m_debug_values.setValues(DebugValues::TYPE::ACC_CMD_PUBLISHED, ctrl_cmd.acc);

  // publish debug values
  tier4_debug_msgs::msg::Float32MultiArrayStamped debug_msg{};
  debug_msg.stamp = clock_->now();
  for (const auto & v : m_debug_values.getValues()) {
    debug_msg.data.push_back(static_cast<decltype(debug_msg.data)::value_type>(v));
  }
  m_pub_debug->publish(debug_msg);

  // slope angle
  tier4_debug_msgs::msg::Float32MultiArrayStamped slope_msg{};
  slope_msg.stamp = clock_->now();
  slope_msg.data.push_back(
    static_cast<decltype(slope_msg.data)::value_type>(control_data.slope_angle));
  m_pub_slope->publish(slope_msg);
}

void EadrcLongitudinalController::updateControlState(const ControlData & control_data)
{
  const double current_vel = control_data.current_motion.vel;
  const double current_acc = control_data.current_motion.acc;
  const double stop_dist = control_data.stop_dist;

  // flags for state transition
  const auto & p = m_state_transition_params;

  const bool departure_condition_from_stopping =
    stop_dist > p.drive_state_stop_dist + p.drive_state_offset_stop_dist;
  const bool departure_condition_from_stopped = stop_dist > p.drive_state_stop_dist;

  // NOTE: the same velocity threshold as motion_utils::searchZeroVelocity
  static constexpr double vel_epsilon = 1e-3;

  // Let vehicle start after the steering is converged for control accuracy
  const bool keep_stopped_condition = std::fabs(current_vel) < vel_epsilon &&
                                      m_enable_keep_stopped_until_steer_convergence &&
                                      !lateral_sync_data_.is_steer_converged;
  if (keep_stopped_condition) {
    auto marker = createDefaultMarker(
      "map", clock_->now(), "stop_reason", 0, Marker::TEXT_VIEW_FACING,
      createMarkerScale(0.0, 0.0, 1.0), createMarkerColor(1.0, 1.0, 1.0, 0.999));
    marker.pose = tier4_autoware_utils::calcOffsetPose(
      m_current_kinematic_state.pose.pose, m_wheel_base + m_front_overhang,
      m_vehicle_width / 2 + 2.0, 1.5);
    marker.text = "steering not\nconverged";
    m_pub_stop_reason_marker->publish(marker);
  }

  const bool stopping_condition = stop_dist < p.stopping_state_stop_dist;

  const bool is_stopped = std::abs(current_vel) < p.stopped_state_entry_vel &&
                          std::abs(current_acc) < p.stopped_state_entry_acc;
  // Case where the ego slips in the opposite direction of the gear due to e.g. a slope is also
  // considered as a stop
  const bool is_not_running = [&]() {
    if (control_data.shift == Shift::Forward) {
      if (is_stopped || current_vel < 0.0) {
        // NOTE: Stopped or moving backward
        return true;
      }
    } else {
      if (is_stopped || 0.0 < current_vel) {
        // NOTE: Stopped or moving forward
        return true;
      }
    }
    return false;
  }();
  if (!is_not_running) {
    m_last_running_time = std::make_shared<rclcpp::Time>(clock_->now());
  }
  const bool stopped_condition =
    m_last_running_time
      ? (clock_->now() - *m_last_running_time).seconds() > p.stopped_state_entry_duration_time
      : false;

  // ==========================================================================================
  // NOTE: due to removeOverlapPoints() in getControlData() m_trajectory and
  // control_data.interpolated_traj have different size.
  // ==========================================================================================
  const double current_vel_cmd = std::fabs(
    control_data.interpolated_traj.points.at(control_data.nearest_idx).longitudinal_velocity_mps);
  const bool emergency_condition = m_enable_overshoot_emergency &&
                                   stop_dist < -p.emergency_state_overshoot_stop_dist &&
                                   current_vel_cmd < vel_epsilon;
  const bool has_nonzero_target_vel = std::abs(current_vel_cmd) > 1.0e-5;

  const auto changeState = [this](const auto s) {
    if (s != m_control_state) {
      RCLCPP_DEBUG_STREAM(
        logger_, "controller state changed: " << toStr(m_control_state) << " -> " << toStr(s));
    }
    m_control_state = s;
    return;
  };

  const auto debug_msg_once = [this](const auto & s) { RCLCPP_DEBUG_ONCE(logger_, "%s", s); };

  const bool is_under_control = m_current_operation_mode.is_autoware_control_enabled &&
                                m_current_operation_mode.mode == OperationModeState::AUTONOMOUS;

  if (is_under_control != m_prev_vehicle_is_under_control) {
    m_prev_vehicle_is_under_control = is_under_control;
    m_under_control_starting_time =
      is_under_control ? std::make_shared<rclcpp::Time>(clock_->now()) : nullptr;
  }
  // transit state
  // in DRIVE state
  if (m_control_state == ControlState::DRIVE) {
    if (emergency_condition) {
      return changeState(ControlState::EMERGENCY);
    }
    if (!is_under_control && stopped_condition && keep_stopped_condition) {
      // NOTE: When the ego is stopped on manual driving, since the driving state may transit to
      //       autonomous, keep_stopped_condition should be checked.
      return changeState(ControlState::STOPPED);
    }

    if (m_enable_smooth_stop) {
      if (stopping_condition) {
        // predictions after input time delay
        const double pred_vel_in_target = control_data.state_after_delay.vel;
        const double pred_stop_dist =
          control_data.stop_dist -
          0.5 * (pred_vel_in_target + current_vel) * m_delay_compensation_time;
        m_smooth_stop.init(pred_vel_in_target, pred_stop_dist);
        return changeState(ControlState::STOPPING);
      }
    } else {
      if (stopped_condition && !departure_condition_from_stopped) {
        return changeState(ControlState::STOPPED);
      }
    }
    return;
  }

  // in STOPPING state
  if (m_control_state == ControlState::STOPPING) {
    if (emergency_condition) {
      return changeState(ControlState::EMERGENCY);
    }
    if (stopped_condition) {
      return changeState(ControlState::STOPPED);
    }

    if (departure_condition_from_stopping) {
      // m_epid_vel.reset();
      m_lpf_vel_error->reset(0.0);
      // prevent the car from taking a long time to start to move
      m_prev_ctrl_cmd.acc = std::max(0.0, m_prev_ctrl_cmd.acc);
      return changeState(ControlState::DRIVE);
    }
    return;
  }

  // in STOPPED state
  if (m_control_state == ControlState::STOPPED) {
    // -- debug print --
    if (has_nonzero_target_vel && !departure_condition_from_stopped) {
      debug_msg_once("target speed > 0, but departure condition is not met. Keep STOPPED.");
    }
    if (has_nonzero_target_vel && keep_stopped_condition) {
      debug_msg_once("target speed > 0, but keep stop condition is met. Keep STOPPED.");
    }
    // ---------------

    if (keep_stopped_condition) {
      return changeState(ControlState::STOPPED);
    }
    if (departure_condition_from_stopped) {
      // m_pid_vel.reset();
      m_lpf_vel_error->reset(0.0);
      // prevent the car from taking a long time to start to move
      m_prev_ctrl_cmd.acc = std::max(0.0, m_prev_ctrl_cmd.acc);
      return changeState(ControlState::DRIVE);
    }

    return;
  }

  // in EMERGENCY state
  if (m_control_state == ControlState::EMERGENCY) {
    if (stopped_condition) {
      return changeState(ControlState::STOPPED);
    }

    if (!emergency_condition) {
      if (!is_under_control) {
        // NOTE: On manual driving, no need stopping to exit the emergency.
        return changeState(ControlState::DRIVE);
      }
    }
    return;
  }

  RCLCPP_FATAL(logger_, "invalid state found.");
  return;
}








double EadrcLongitudinalController::getTimeUnderControl()
{
  if (!m_under_control_starting_time) return 0.0;
  return (clock_->now() - *m_under_control_starting_time).seconds();
}

double EadrcLongitudinalController::calculateControlSignal(const double error, const double dt)
{
  Eigen::RowVector2d stateVector = p_obserwer->getStateVector();

  double controlSignal = (stateVector[0]*m_kp + stateVector[1])*1/m_b_hat;

  stateVector = p_obserwer->calculateStateOfESO(error, dt, controlSignal);

  p_obserwer->setLastStateVector(stateVector);

  return controlSignal;
}

double EadrcLongitudinalController::applyVelocityFeedback(const ControlData & control_data)
{
  // NOTE: Acceleration command is always positive even if the ego drives backward.
  const double vel_sign = (control_data.shift == Shift::Forward)
                            ? 1.0
                            : (control_data.shift == Shift::Reverse ? -1.0 : 0.0);
  const double current_vel = control_data.current_motion.vel;
  const auto target_motion = Motion{
    control_data.interpolated_traj.points.at(control_data.target_idx).longitudinal_velocity_mps,
    control_data.interpolated_traj.points.at(control_data.target_idx).acceleration_mps2};
  const double diff_vel = (target_motion.vel - current_vel) * vel_sign;
  const double error_vel_filtered = m_lpf_vel_error->filter(diff_vel);


  double eadrc_acc = calculateControlSignal(error_vel_filtered, control_data.dt);
  

  // Feedforward scaling:
  // This is for the coordinate conversion where feedforward is applied, from Time to Arclength.
  // Details: For accurate control, the feedforward should be calculated in the arclength coordinate
  // system, not in the time coordinate system. Otherwise, even if FF is applied, the vehicle speed
  // deviation will be bigger.
  // constexpr double ff_scale_max = 2.0;  // for safety
  // constexpr double ff_scale_min = 0.5;  // for safety
  // const double ff_scale = std::clamp(
  //   std::abs(current_vel) / std::max(std::abs(target_motion.vel), 0.1), ff_scale_min, ff_scale_max);
  // const double ff_acc =
  //   control_data.interpolated_traj.points.at(control_data.target_idx).acceleration_mps2 * ff_scale;

  //const double feedback_acc = ff_acc + pid_acc;


  // m_debug_values.setValues(DebugValues::TYPE::ACC_CMD_PID_APPLIED, feedback_acc);
  // m_debug_values.setValues(DebugValues::TYPE::ERROR_VEL_FILTERED, error_vel_filtered);
  // m_debug_values.setValues(DebugValues::TYPE::FF_SCALE, ff_scale);
  // m_debug_values.setValues(DebugValues::TYPE::ACC_CMD_FF, ff_acc);

  return eadrc_acc;
}

EadrcLongitudinalController::Motion EadrcLongitudinalController::keepBrakeBeforeStop(
  const ControlData & control_data, const Motion & target_motion, const size_t nearest_idx) const
{
  Motion output_motion = target_motion;

  if (m_enable_brake_keeping_before_stop == false) {
    return output_motion;
  }
  const auto traj = control_data.interpolated_traj;

  const auto stop_idx = motion_utils::searchZeroVelocityIndex(traj.points);
  if (!stop_idx) {
    return output_motion;
  }

  double min_acc_before_stop = std::numeric_limits<double>::max();
  size_t min_acc_idx = std::numeric_limits<size_t>::max();
  for (int i = static_cast<int>(*stop_idx); i >= 0; --i) {
    const auto ui = static_cast<size_t>(i);
    if (traj.points.at(ui).acceleration_mps2 > static_cast<float>(min_acc_before_stop)) {
      break;
    }
    min_acc_before_stop = traj.points.at(ui).acceleration_mps2;
    min_acc_idx = ui;
  }

  const double brake_keeping_acc = std::max(m_brake_keeping_acc, min_acc_before_stop);
  if (nearest_idx >= min_acc_idx && target_motion.acc > brake_keeping_acc) {
    output_motion.acc = brake_keeping_acc;
  }

  return output_motion;
}

void EadrcLongitudinalController::storeAccelCmd(const double accel)
{
  if (m_control_state == ControlState::DRIVE) {
    // convert format
    autoware_auto_control_msgs::msg::LongitudinalCommand cmd;
    cmd.stamp = clock_->now();
    cmd.acceleration = static_cast<decltype(cmd.acceleration)>(accel);

    // store published ctrl cmd
    m_ctrl_cmd_vec.emplace_back(cmd);
  } else {
    // reset command
    m_ctrl_cmd_vec.clear();
  }

  // remove unused ctrl cmd
  if (m_ctrl_cmd_vec.size() <= 2) {
    return;
  }
  if ((clock_->now() - m_ctrl_cmd_vec.at(1).stamp).seconds() > m_delay_compensation_time) {
    m_ctrl_cmd_vec.erase(m_ctrl_cmd_vec.begin());
  }
}

double EadrcLongitudinalController::applySlopeCompensation(
  const double input_acc, const double pitch, const Shift shift) const
{
  if (!m_enable_slope_compensation) {
    return input_acc;
  }
  const double pitch_limited = std::min(std::max(pitch, m_min_pitch_rad), m_max_pitch_rad);

  // Acceleration command is always positive independent of direction (= shift) when car is running
  double sign = (shift == Shift::Forward) ? 1.0 : (shift == Shift::Reverse ? -1.0 : 0);
  double compensated_acc = input_acc + sign * 9.81 * std::sin(pitch_limited);
  return compensated_acc;
}

double EadrcLongitudinalController::calcFilteredAcc(
  const double raw_acc, const ControlData & control_data)
{
  const double acc_max_filtered = std::clamp(raw_acc, m_min_acc, m_max_acc);
  m_debug_values.setValues(DebugValues::TYPE::ACC_CMD_ACC_LIMITED, acc_max_filtered);

  // store ctrl cmd without slope filter
  storeAccelCmd(acc_max_filtered);

  // apply slope compensation
  const double acc_slope_filtered =
    applySlopeCompensation(acc_max_filtered, control_data.slope_angle, control_data.shift);
  m_debug_values.setValues(DebugValues::TYPE::ACC_CMD_SLOPE_APPLIED, acc_slope_filtered);

  // This jerk filter must be applied after slope compensation
  const double acc_jerk_filtered = longitudinal_utils::applyDiffLimitFilter(
    acc_slope_filtered, m_prev_ctrl_cmd.acc, control_data.dt, m_max_jerk, m_min_jerk);
  m_debug_values.setValues(DebugValues::TYPE::ACC_CMD_JERK_LIMITED, acc_jerk_filtered);

  return acc_jerk_filtered;
}

void EadrcLongitudinalController::updateDebugVelAcc(const ControlData & control_data)
{
  m_debug_values.setValues(DebugValues::TYPE::CURRENT_VEL, control_data.current_motion.vel);
  m_debug_values.setValues(
    DebugValues::TYPE::TARGET_VEL,
    control_data.interpolated_traj.points.at(control_data.target_idx).longitudinal_velocity_mps);
  m_debug_values.setValues(
    DebugValues::TYPE::TARGET_ACC,
    control_data.interpolated_traj.points.at(control_data.target_idx).acceleration_mps2);
  m_debug_values.setValues(
    DebugValues::TYPE::NEAREST_VEL,
    control_data.interpolated_traj.points.at(control_data.nearest_idx).longitudinal_velocity_mps);
  m_debug_values.setValues(
    DebugValues::TYPE::NEAREST_ACC,
    control_data.interpolated_traj.points.at(control_data.nearest_idx).acceleration_mps2);
  m_debug_values.setValues(
    DebugValues::TYPE::ERROR_VEL,
    control_data.interpolated_traj.points.at(control_data.nearest_idx).longitudinal_velocity_mps -
      control_data.current_motion.vel);
}

EadrcLongitudinalController::Motion EadrcLongitudinalController::calcCtrlCmd(
  const ControlData & control_data)
{
  const size_t target_idx = control_data.target_idx;

  // velocity and acceleration command
  Motion raw_ctrl_cmd{
    control_data.interpolated_traj.points.at(target_idx).longitudinal_velocity_mps,
    control_data.interpolated_traj.points.at(target_idx).acceleration_mps2};

  if (m_control_state == ControlState::DRIVE) {
    raw_ctrl_cmd.vel =
      control_data.interpolated_traj.points.at(control_data.target_idx).longitudinal_velocity_mps;
    raw_ctrl_cmd.acc = applyVelocityFeedback(control_data);
    raw_ctrl_cmd = keepBrakeBeforeStop(control_data, raw_ctrl_cmd, target_idx);

    RCLCPP_DEBUG(
      logger_,
      "[feedback control]  vel: %3.3f, acc: %3.3f, dt: %3.3f, v_curr: %3.3f, v_ref: %3.3f "
      "feedback_ctrl_cmd.ac: %3.3f",
      raw_ctrl_cmd.vel, raw_ctrl_cmd.acc, control_data.dt, control_data.current_motion.vel,
      control_data.interpolated_traj.points.at(control_data.target_idx).longitudinal_velocity_mps,
      raw_ctrl_cmd.acc);
  } else if (m_control_state == ControlState::STOPPING) {
    raw_ctrl_cmd.acc = m_smooth_stop.calculate(
      control_data.stop_dist, control_data.current_motion.vel, control_data.current_motion.acc,
      m_vel_hist, m_delay_compensation_time);
    raw_ctrl_cmd.vel = m_stopped_state_params.vel;

    RCLCPP_DEBUG(
      logger_, "[smooth stop]: Smooth stopping. vel: %3.3f, acc: %3.3f", raw_ctrl_cmd.vel,
      raw_ctrl_cmd.acc);
  } else if (m_control_state == ControlState::STOPPED) {
    // This acceleration is without slope compensation
    const auto & p = m_stopped_state_params;
    raw_ctrl_cmd.vel = p.vel;
    raw_ctrl_cmd.acc = longitudinal_utils::applyDiffLimitFilter(
      p.acc, m_prev_raw_ctrl_cmd.acc, control_data.dt, p.jerk);

    RCLCPP_DEBUG(logger_, "[Stopped]. vel: %3.3f, acc: %3.3f", raw_ctrl_cmd.vel, raw_ctrl_cmd.acc);
  } else if (m_control_state == ControlState::EMERGENCY) {
    raw_ctrl_cmd = calcEmergencyCtrlCmd(control_data.dt);
  }

  // store acceleration without slope compensation
  m_prev_raw_ctrl_cmd = raw_ctrl_cmd;

  // apply slope compensation and filter acceleration and jerk
  const double filtered_acc_cmd = calcFilteredAcc(raw_ctrl_cmd.acc, control_data);
  const Motion filtered_ctrl_cmd{raw_ctrl_cmd.vel, filtered_acc_cmd};

  // update debug visualization
  updateDebugVelAcc(control_data);

  return filtered_ctrl_cmd;
}

bool EadrcLongitudinalController::isReady(
  [[maybe_unused]] const trajectory_follower::InputData & input_data)
{
  return true;
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

  // self pose is far from trajectory
  if (control_data.is_far_from_trajectory) {
    const Motion raw_ctrl_cmd = calcEmergencyCtrlCmd(control_data.dt);  // calculate control command
    m_prev_raw_ctrl_cmd = raw_ctrl_cmd;
    const auto cmd_msg =
      createCtrlCmdMsg(raw_ctrl_cmd, control_data.current_motion.vel);  // create control command
    publishDebugData(raw_ctrl_cmd, control_data);                       // publish debug data
    trajectory_follower::LongitudinalOutput output;
    output.control_cmd = cmd_msg;
    return output;
  }

  // update control state
  updateControlState(control_data);

  // calculate control command
  const Motion ctrl_cmd = calcCtrlCmd(control_data);

  // publish control command
  const auto cmd_msg = createCtrlCmdMsg(ctrl_cmd, control_data.current_motion.vel);
  trajectory_follower::LongitudinalOutput output;
  output.control_cmd = cmd_msg;

  // publish debug data
  publishDebugData(ctrl_cmd, control_data);

  // diagnostic
  diagnostic_updater_.force_update();


  return output;
}

}  // autoware::motion::control::namespace eadrc_longitudinal_controller
