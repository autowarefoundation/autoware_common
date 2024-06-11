// Copyright 2023 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "autoware_utils/math/trigonometry.hpp"

#include "autoware_utils/math/constants.hpp"
#include "autoware_utils/math/sin_table.hpp"

#include <cmath>

namespace autoware_utils
{

float sin(float radian)
{
  float degree =
    radian * (180.f / static_cast<float>(autoware_utils::pi)) * (discrete_arcs_num_360 / 360.f);
  size_t idx =
    (static_cast<int>(std::round(degree)) % discrete_arcs_num_360 + discrete_arcs_num_360) %
    discrete_arcs_num_360;

  float mul = 1.f;
  if (discrete_arcs_num_90 <= idx && idx < 2 * discrete_arcs_num_90) {
    idx = 2 * discrete_arcs_num_90 - idx;
  } else if (2 * discrete_arcs_num_90 <= idx && idx < 3 * discrete_arcs_num_90) {
    mul = -1.f;
    idx = idx - 2 * discrete_arcs_num_90;
  } else if (3 * discrete_arcs_num_90 <= idx && idx < 4 * discrete_arcs_num_90) {
    mul = -1.f;
    idx = 4 * discrete_arcs_num_90 - idx;
  }

  return mul * g_sin_table[idx];
}

float cos(float radian)
{
  return sin(radian + static_cast<float>(autoware_utils::pi) / 2.f);
}

std::pair<float, float> sin_and_cos(float radian)
{
  constexpr float tmp =
    (180.f / static_cast<float>(autoware_utils::pi)) * (discrete_arcs_num_360 / 360.f);
  const float degree = radian * tmp;
  size_t idx =
    (static_cast<int>(std::round(degree)) % discrete_arcs_num_360 + discrete_arcs_num_360) %
    discrete_arcs_num_360;

  if (idx < discrete_arcs_num_90) {
    return {g_sin_table[idx], g_sin_table[discrete_arcs_num_90 - idx]};
  } else if (discrete_arcs_num_90 <= idx && idx < 2 * discrete_arcs_num_90) {
    idx = 2 * discrete_arcs_num_90 - idx;
    return {g_sin_table[idx], -g_sin_table[discrete_arcs_num_90 - idx]};
  } else if (2 * discrete_arcs_num_90 <= idx && idx < 3 * discrete_arcs_num_90) {
    idx = idx - 2 * discrete_arcs_num_90;
    return {-g_sin_table[idx], -g_sin_table[discrete_arcs_num_90 - idx]};
  } else {  // 3 * discrete_arcs_num_90 <= idx && idx < 4 * discrete_arcs_num_90
    idx = 4 * discrete_arcs_num_90 - idx;
    return {-g_sin_table[idx], g_sin_table[discrete_arcs_num_90 - idx]};
  }
}

}  // namespace autoware_utils
