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


#ifndef LANELET2_EXTENSION__REGULATORY_ELEMENTS__INVALID_LANELET_HPP_
#define LANELET2_EXTENSION__REGULATORY_ELEMENTS__INVALID_LANELET_HPP_

// NOLINTBEGIN(readability-identifier-naming)

#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_core/primitives/Lanelet.h>

#include <memory>
#include <vector>

namespace lanelet::autoware
{
class InvalidLanelet : public lanelet::RegulatoryElement
{
public:
  using Ptr = std::shared_ptr<InvalidLanelet>;
  static constexpr char RuleName[] = "invalid_lanelet";

  static Ptr make(Id id, const AttributeMap & attributes, const Polygon3d & invalid_lanelet)
  {
    return Ptr{new InvalidLanelet(id, attributes, invalid_lanelet)};
  }

  /**
   * @brief get the relevant invalid lanelet
   * @return invalid lanelet
   */
  [[nodiscard]] ConstPolygon3d invalidLanelet() const;
  [[nodiscard]] Polygon3d invalidLanelet();

  /**
   * @brief add a new invalid lanelet
   * @param primitive invalid lanelet to add
   */
  void addInvalidLanelet(const Polygon3d & primitive);

  /**
   * @brief remove an invalid lanelet
   * @param primitive invalid lanelet to remove
   * @return true if the invalid lanelet existed and was removed
   */
  bool removeInvalidLanelet(const Polygon3d & primitive);

private:
  // the following lines are required so that lanelet2 can create this object
  // when loading a map with this regulatory element
  friend class lanelet::RegisterRegulatoryElement<InvalidLanelet>;
  InvalidLanelet(Id id, const AttributeMap & attributes, const Polygon3d & invalid_lanelet);
  explicit InvalidLanelet(const lanelet::RegulatoryElementDataPtr & data);
};
static lanelet::RegisterRegulatoryElement<InvalidLanelet> regInvalidLanelet;

}  // namespace lanelet::autoware

// NOLINTEND(readability-identifier-naming)

#endif  // LANELET2_EXTENSION__REGULATORY_ELEMENTS__INVALID_LANELET_HPP_
