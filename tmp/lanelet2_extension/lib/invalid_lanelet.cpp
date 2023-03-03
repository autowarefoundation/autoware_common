// Copyright 2015-2019 Autoware Foundation. All rights reserved.
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

// NOLINTBEGIN(readability-identifier-naming)

#include "lanelet2_extension/regulatory_elements/invalid_lanelet.hpp"

#include <boost/variant.hpp>

#include <lanelet2_core/primitives/RegulatoryElement.h>

#include <algorithm>
#include <memory>
#include <utility>
#include <vector>

namespace lanelet::autoware
{
namespace
{
template <typename T>
bool findAndErase(const T & primitive, RuleParameters * member)
{
  if (member == nullptr) {
    std::cerr << __FUNCTION__ << ": member is null pointer";
    return false;
  }
  auto it = std::find(member->begin(), member->end(), RuleParameter(primitive));
  if (it == member->end()) {
    return false;
  }
  member->erase(it);
  return true;
}

Polygons3d getPoly(const RuleParameterMap & paramsMap, RoleName role)
{
  auto params = paramsMap.find(role);
  if (params == paramsMap.end()) {
    return {};
  }

  Polygons3d result;
  for (auto & param : params->second) {
    auto p = boost::get<Polygon3d>(&param);
    if (p != nullptr) {
      result.push_back(*p);
    }
  }
  return result;
}

ConstPolygons3d getConstPoly(const RuleParameterMap & params, RoleName role)
{
  auto cast_func = [](auto & poly) { return static_cast<ConstPolygon3d>(poly); };
  return utils::transform(getPoly(params, role), cast_func);
}

RegulatoryElementDataPtr constructInvalidLaneletData(
  Id id, const AttributeMap & attributes, const Polygon3d & invalid_lanelet)
{
  RuleParameterMap rpm;
  RuleParameters rule_parameters = {invalid_lanelet};
  rpm.insert(std::make_pair(RoleNameString::Refers, rule_parameters));

  auto data = std::make_shared<RegulatoryElementData>(id, rpm, attributes);
  data->attributes[AttributeName::Type] = AttributeValueString::RegulatoryElement;
  data->attributes[AttributeName::Subtype] = "invalid_lanelet";
  return data;
}
}  // namespace

InvalidLanelet::InvalidLanelet(const RegulatoryElementDataPtr & data) : RegulatoryElement(data)
{
  if (getConstPoly(data->parameters, RoleName::Refers).size() != 1) {
    throw InvalidInputError("There must be exactly one invalid lanelet defined!");
  }
}

InvalidLanelet::InvalidLanelet(Id id, const AttributeMap & attributes, const Polygon3d & invalid_lanelet)
: InvalidLanelet(constructInvalidLaneletData(id, attributes, invalid_lanelet))
{
}

ConstPolygon3d InvalidLanelet::invalidLanelet() const
{
  return getConstPoly(parameters(), RoleName::Refers).front();
}

Polygon3d InvalidLanelet::invalidLanelet() { return getPoly(parameters(), RoleName::Refers).front(); }

void InvalidLanelet::addInvalidLanelet(const Polygon3d & primitive)
{
  parameters()["invalid_lanelet"].emplace_back(primitive);
}

bool InvalidLanelet::removeInvalidLanelet(const Polygon3d & primitive)
{
  return findAndErase(primitive, &parameters().find("invalid_lanelet")->second);
}

}  // namespace lanelet::autoware

// NOLINTEND(readability-identifier-naming)
