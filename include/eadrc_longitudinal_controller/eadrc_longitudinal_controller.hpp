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


namespace eadrc_longitudinal_controller
{

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



}  // namespace eadrc_longitudinal_controller

#endif  // EADRC_LONGITUDINAL_CONTROLLER__EADRC_LONGITUDINAL_CONTROLLER_HPP_
