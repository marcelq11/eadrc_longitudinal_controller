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

class EADRC_LONGITUDINAL_CONTROLLER_PUBLIC EadrcLongitudinalController
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

}  // namespace eadrc_longitudinal_controller

#endif  // EADRC_LONGITUDINAL_CONTROLLER__EADRC_LONGITUDINAL_CONTROLLER_HPP_
