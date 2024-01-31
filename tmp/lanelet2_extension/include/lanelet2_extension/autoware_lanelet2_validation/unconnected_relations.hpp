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

#include <lanelet2_extension/autoware_lanelet2_validation/utils.hpp>
#include <lanelet2_extension/regulatory_elements/crosswalk.hpp>
#include <range/v3/view/filter.hpp>
#include <range/v3/view/transform.hpp>
#include <range/v3/view/unique.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_validation/Validation.h>
#include <lanelet2_validation/ValidatorFactory.h>

#include <set>

namespace lanelet
{
namespace validation
{

// TODO: update description
//! This check looks for points within linestrings or polygons that appear two times in succession.
//! These are not allowed because they often confuse geometry algorithms.
class UnconnectedRelationsChecker : public lanelet::validation::MapValidator
{
public:
  constexpr static const char * name() { return "mapping.unconnected_relations"; }

  lanelet::validation::Issues operator()(const lanelet::LaneletMap & map) override;

private:
  lanelet::validation::Issues checkRegElemExistsInAllTrafficLight(const lanelet::LaneletMap & map);
  lanelet::validation::Issues checkRegElemExistsInAllCrosswalk(const lanelet::LaneletMap & map);
  lanelet::validation::Issues checkRegElemOfTrafficLight(const lanelet::LaneletMap & map);
  lanelet::validation::Issues checkRegElemOfCrosswalk(const lanelet::LaneletMap & map);
  template <typename T>
  void checkPrimitiveType(
    std::vector<T> & in_vec, const std::string & expected_type, const std::string & message,
    lanelet::validation::Issues & issues);
  template <typename T>
  void checkPrimitiveType(
    std::vector<T> & in_vec, const std::string & expected_type,
    const std::string & expected_subtype, const std::string & message,
    lanelet::validation::Issues & issues);

  std::set<lanelet::Id> tl_elem_with_cw_;
};
}  // namespace validation
}  // namespace lanelet
