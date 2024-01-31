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

#include <lanelet2_extension/autoware_lanelet2_validation/validators/missing_regulatory_elements.hpp>

namespace lanelet
{
namespace validation
{
namespace
{
lanelet::validation::RegisterMapValidator<MissingRegulatoryElementsChecker> reg;
}  // namespace

lanelet::validation::Issues MissingRegulatoryElementsChecker::operator()(
  const lanelet::LaneletMap & map)
{
  // All issues found by all validators
  lanelet::validation::Issues issues;

  // Append issues found by each validator
  appendIssues(issues, checkMissingReglatoryElementsInTrafficLight(map));
  appendIssues(issues, checkMissingReglatoryElementsInCrosswalk(map));
  return issues;
}

lanelet::validation::Issues
MissingRegulatoryElementsChecker::checkMissingReglatoryElementsInTrafficLight(
  const lanelet::LaneletMap & map)
{
  lanelet::validation::Issues issues;

  // Get all line strings whose type is traffic light
  auto tl_ids =
    map.lineStringLayer | ranges::views::filter([](auto && ls) {
      const auto & attrs = ls.attributes();
      const auto & it = attrs.find(lanelet::AttributeName::Type);
      return it != attrs.end() && it->second == lanelet::AttributeValueString::TrafficLight;
    }) |
    ranges::views::transform([](auto && ls) { return ls.id(); }) | ranges::views::unique;

  // Filter regulatory elements whose type is traffic light and has refers
  auto reg_elem_tl = map.regulatoryElementLayer | ranges::views::filter([](auto && elem) {
                       const auto & attrs = elem->attributes();
                       const auto & it = attrs.find(lanelet::AttributeName::Subtype);
                       const auto & params = elem->getParameters();
                       return it != attrs.end() &&
                              it->second == lanelet::AttributeValueString::TrafficLight and
                              params.find(lanelet::RoleNameString::Refers) != params.end();
                     });
  // Get all line strings of traffic light referred by regulatory elements
  std::set<lanelet::Id> tl_ids_reg_elem;
  for (const auto & elem : reg_elem_tl) {
    const auto & refers =
      elem->getParameters<lanelet::ConstLineString3d>(lanelet::RoleName::Refers);
    for (const auto & refer : refers) {
      tl_ids_reg_elem.insert(refer.id());
    }
  }

  // Check if all line strings of traffic light referred by regulatory elements
  for (const auto & tl_id : tl_ids) {
    if (tl_ids_reg_elem.find(tl_id) == tl_ids_reg_elem.end()) {
      issues.emplace_back(
        lanelet::validation::Severity::Error, lanelet::validation::Primitive::LineString, tl_id,
        "Traffic light must have a regulatory element.");
    }
  }

  return issues;
}

lanelet::validation::Issues
MissingRegulatoryElementsChecker::checkMissingReglatoryElementsInCrosswalk(
  const lanelet::LaneletMap & map)
{
  lanelet::validation::Issues issues;

  // Get all lanelets whose type is crosswalk
  std::set<lanelet::Id> cw_ids;
  for (const auto & ll : map.laneletLayer) {
    const auto & attrs = ll.attributes();
    const auto & it = attrs.find(lanelet::AttributeName::Subtype);
    // Check if this lanelet is crosswalk
    if (it != attrs.end() && it->second == lanelet::AttributeValueString::Crosswalk) {
      cw_ids.insert(ll.id());

      // Check if crosswalk has reg elem of traffic light
      for (const auto & elem : ll.regulatoryElements()) {
        const auto & attrs = elem->attributes();
        const auto & it = attrs.find(lanelet::AttributeName::Subtype);
        if (it != attrs.end() && it->second == lanelet::AttributeValueString::TrafficLight) {
          tl_elem_with_cw_.insert(elem->id());
        }
      }
    }
  }

  // Filter regulatory elements whose type is crosswalk and has refers
  auto reg_elem_cw =
    map.regulatoryElementLayer | ranges::views::filter([](auto && elem) {
      const auto & attrs = elem->attributes();
      const auto & it = attrs.find(lanelet::AttributeName::Subtype);
      return it != attrs.end() && it->second == lanelet::AttributeValueString::Crosswalk;
    }) |
    ranges::views::filter([](auto && elem) {
      const auto & param = elem->getParameters();
      return param.find(lanelet::RoleNameString::Refers) != param.end();
    });

  // Get all lanelets of crosswalk referred by regulatory elements
  std::set<lanelet::Id> cw_ids_reg_elem;
  for (const auto & elem : reg_elem_cw) {
    const auto & refers = elem->getParameters<lanelet::ConstLanelet>(lanelet::RoleName::Refers);
    for (const lanelet::ConstLanelet & refer : refers) {
      cw_ids_reg_elem.insert(refer.id());
    }
  }

  // Check if all lanelets of crosswalk referred by regulatory elements
  for (const auto & cw_id : cw_ids) {
    if (cw_ids_reg_elem.find(cw_id) == cw_ids_reg_elem.end()) {
      issues.emplace_back(
        lanelet::validation::Severity::Error, lanelet::validation::Primitive::Lanelet, cw_id,
        "Crosswalk must have a regulatory element.");
    }
  }

  return issues;
}

}  // namespace validation
}  // namespace lanelet
