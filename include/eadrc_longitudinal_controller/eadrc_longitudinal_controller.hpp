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

#ifndef EADRC_LONGITUDINAL_CONTROLLER__EADRC_LONGITUDINAL_CONTROLLER_HPP_
#define EADRC_LONGITUDINAL_CONTROLLER__EADRC_LONGITUDINAL_CONTROLLER_HPP_

#include <cstdint>

#include "eadrc_longitudinal_controller/visibility_control.hpp"


#include "diagnostic_updater/diagnostic_updater.hpp"
#include "eadrc_longitudinal_controller/ESO.hpp"
#include "eadrc_longitudinal_controller/longitudinal_controller_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/utils.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tier4_autoware_utils/ros/marker_helper.hpp"
#include "trajectory_follower_base/longitudinal_controller_base.hpp"
#include "vehicle_info_util/vehicle_info_util.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "autoware_adapi_v1_msgs/msg/operation_mode_state.hpp"
#include "autoware_auto_control_msgs/msg/longitudinal_command.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "autoware_auto_vehicle_msgs/msg/vehicle_odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tier4_debug_msgs/msg/float32_multi_array_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace autoware::motion::control::eadrc_longitudinal_controller
{
using autoware_adapi_v1_msgs::msg::OperationModeState;
using tier4_autoware_utils::createDefaultMarker;
using tier4_autoware_utils::createMarkerColor;
using tier4_autoware_utils::createMarkerScale;
using visualization_msgs::msg::Marker;

namespace trajectory_follower = ::autoware::motion::control::trajectory_follower;

class EadrcLongitudinalController : public trajectory_follower::LongitudinalControllerBase
{
public: 
  EadrcLongitudinalController();
  
  void calculateOuterLoopUStar();
  void calculateInternalLoopCompensationTotalDisturbance();
  void saturation();

private:
  int16_t m_saturationValueUpper;
  int16_t m_saturationValueLower;
  };

  struct StateAfterDelay
  {
    StateAfterDelay(const double velocity, const double acceleration, const double distance)
    : vel(velocity), acc(acceleration), running_distance(distance)
    {
    }
    double vel{0.0};
    double acc{0.0};
    double running_distance{0.0};
  };
  enum class Shift { Forward = 0, Reverse };

  struct ControlData
  {
    bool is_far_from_trajectory{false};
    autoware_auto_planning_msgs::msg::Trajectory interpolated_traj{};
    size_t nearest_idx{0};  // nearest_idx = 0 when nearest_idx is not found with findNearestIdx
    size_t target_idx{0};
    StateAfterDelay state_after_delay{0.0, 0.0, 0.0};
    Motion current_motion{};
    Shift shift{Shift::Forward};  // shift is used only to calculate the sign of pitch compensation
    double stop_dist{0.0};  // signed distance that is positive when car is before the stopline
    double slope_angle{0.0};
    double dt{0.0};
  };

  // pointers for ros topic
  nav_msgs::msg::Odometry m_current_kinematic_state;
  geometry_msgs::msg::AccelWithCovarianceStamped m_current_accel;
  autoware_auto_planning_msgs::msg::Trajectory m_trajectory;
  OperationModeState m_current_operation_mode;

  // slope compensation
  enum class SlopeSource { RAW_PITCH = 0, TRAJECTORY_PITCH, TRAJECTORY_ADAPTIVE };
  SlopeSource m_slope_source{SlopeSource::RAW_PITCH};
  double m_adaptive_trajectory_velocity_th;
  std::shared_ptr<LowpassFilter1d> m_lpf_pitch{nullptr};
  double m_max_pitch_rad;
  double m_min_pitch_rad;

  // debug values
  DebugValues m_debug_values;

  std::shared_ptr<rclcpp::Time> m_last_running_time{std::make_shared<rclcpp::Time>(clock_->now())};

  // drive
  PIDController m_eadrc_vel;

  // buffer of send command
  std::vector<autoware_auto_control_msgs::msg::LongitudinalCommand> m_ctrl_cmd_vec;

  bool isReady(const trajectory_follower::InputData & input_data) override;

  trajectory_follower::LongitudinalOutput run(
    trajectory_follower::InputData const & input_data) override;

  /**
   * @brief set reference trajectory with received message
   * @param [in] msg trajectory message
   */
  void setTrajectory(const autoware_auto_planning_msgs::msg::Trajectory & msg);

  /**
   * @brief set current and previous velocity with received message
   * @param [in] msg current state message
   */
  void setKinematicState(const nav_msgs::msg::Odometry & msg);

  /**
   * @brief set current acceleration with received message
   * @param [in] msg trajectory message
   */
  void setCurrentAcceleration(const geometry_msgs::msg::AccelWithCovarianceStamped & msg);

  /**
   * @brief set current operation mode with received message
   * @param [in] msg operation mode report message
   */
  void setCurrentOperationMode(const OperationModeState & msg);

  /**
   * @brief calculate data for controllers whose type is ControlData
   * @param [in] current_pose current ego pose
   */
  ControlData getControlData(const geometry_msgs::msg::Pose & current_pose);

  /**
   * @brief calculate time between current and previous one
   */
  double getDt();

  /**
   * @brief interpolate trajectory point that is nearest to vehicle
   * @param [in] traj reference trajectory
   * @param [in] point vehicle position
   * @param [in] nearest_idx index of the trajectory point nearest to the vehicle position
   */
  std::pair<autoware_auto_planning_msgs::msg::TrajectoryPoint, size_t>
  calcInterpolatedTrajPointAndSegment(
    const autoware_auto_planning_msgs::msg::Trajectory & traj,
    const geometry_msgs::msg::Pose & pose) const;
  
  /**
   * @brief calculate predicted velocity after time delay based on past control commands
   * @param [in] current_motion current velocity and acceleration of the vehicle
   * @param [in] delay_compensation_time predicted time delay
   */
  StateAfterDelay predictedStateAfterDelay(
    const Motion current_motion, const double delay_compensation_time) const;


  /**
   * @brief calculate direction (forward or backward) that vehicle moves
   * @param [in] control_data data for control calculation
   */
  enum Shift getCurrentShift(const ControlData & control_data) const;
  

  /**
   * @brief update variables for debugging about pitch
   * @param [in] pitch current pitch of the vehicle (filtered)
   * @param [in] traj_pitch current trajectory pitch
   * @param [in] raw_pitch current raw pitch of the vehicle (unfiltered)
   */
  void updatePitchDebugValues(const double pitch, const double traj_pitch, const double raw_pitch);

  /**
   * @brief calculate control command in emergency state
   * @param [in] dt time between previous and current one
   */
  Motion calcEmergencyCtrlCmd(const double dt) const;
  



}  // namespace eadrc_longitudinal_controller

#endif  // EADRC_LONGITUDINAL_CONTROLLER__EADRC_LONGITUDINAL_CONTROLLER_HPP_
