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

#ifndef LANELET2_EXTENSION__AUTOWARE_LANELET2_VALIDATION__VALIDATION_HPP_
#define LANELET2_EXTENSION__AUTOWARE_LANELET2_VALIDATION__VALIDATION_HPP_

#include "lanelet2_extension/autoware_lanelet2_validation/cli.hpp"
#include "lanelet2_extension/autoware_lanelet2_validation/utils.hpp"
#include "lanelet2_extension/projection/mgrs_projector.hpp"
#include "lanelet2_extension/projection/transverse_mercator_projector.hpp"

#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_validation/Cli.h>
#include <lanelet2_validation/Validation.h>

#include <memory>
#include <regex>
#include <vector>

namespace
{
namespace projector_names
{
constexpr const char * mgrs = "mgrs";
constexpr const char * transverse_mercator = "transverse_mercator";
constexpr const char * utm = "utm";
}  // namespace projector_names
}  // namespace

namespace lanelet
{
namespace autoware
{
namespace validation
{
std::unique_ptr<lanelet::Projector> getProjector(const MetaConfig & config);
std::vector<lanelet::validation::DetectedIssues> validateMap(const MetaConfig & config);
}  // namespace validation
}  // namespace autoware
}  // namespace lanelet

#endif  // LANELET2_EXTENSION__AUTOWARE_LANELET2_VALIDATION__VALIDATION_HPP_
