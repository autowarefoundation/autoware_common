// Copyright 2023 Autoware Foundation
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

#ifndef LANELET2_EXTENSION__LOCALIZATION__LANDMARK_HPP_
#define LANELET2_EXTENSION__LOCALIZATION__LANDMARK_HPP_

#include "autoware_auto_mapping_msgs/msg/had_map_bin.hpp"

#include <lanelet2_core/primitives/Polygon.h>

#include <string>
#include <vector>

// NOLINTBEGIN(readability-identifier-naming)

namespace lanelet::localization
{

std::vector<lanelet::Polygon3d> parseLandmarkPolygons(
  const autoware_auto_mapping_msgs::msg::HADMapBin::ConstSharedPtr & msg,
  const std::string & target_subtype);

}  // namespace lanelet::localization

// NOLINTEND(readability-identifier-naming)

#endif  // LANELET2_EXTENSION__LOCALIZATION__LANDMARK_HPP_
