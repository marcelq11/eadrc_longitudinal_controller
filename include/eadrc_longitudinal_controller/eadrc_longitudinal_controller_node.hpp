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

#ifndef EADRC_LONGITUDINAL_CONTROLLER__EADRC_LONGITUDINAL_CONTROLLER_NODE_HPP_
#define EADRC_LONGITUDINAL_CONTROLLER__EADRC_LONGITUDINAL_CONTROLLER_NODE_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "eadrc_longitudinal_controller/eadrc_longitudinal_controller.hpp"

namespace eadrc_longitudinal_controller
{
using EadrcLongitudinalControllerPtr = std::unique_ptr<eadrc_longitudinal_controller::EadrcLongitudinalController>;

class EADRC_LONGITUDINAL_CONTROLLER_PUBLIC EadrcLongitudinalControllerNode : public rclcpp::Node
{
public:
  explicit EadrcLongitudinalControllerNode(const rclcpp::NodeOptions & options);

private:
  EadrcLongitudinalControllerPtr eadrc_longitudinal_controller_{nullptr};
  int64_t param_name_{123};
};
}  // namespace eadrc_longitudinal_controller

#endif  // EADRC_LONGITUDINAL_CONTROLLER__EADRC_LONGITUDINAL_CONTROLLER_NODE_HPP_
