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

#ifndef LANELET2_EXTENSION__AUTOWARE_LANELET2_VALIDATION__UTILS_HPP_
#define LANELET2_EXTENSION__AUTOWARE_LANELET2_VALIDATION__UTILS_HPP_

#include <lanelet2_validation/Validation.h>
#include <lanelet2_validation/ValidatorFactory.h>

#include <string>
#include <vector>

namespace lanelet
{
namespace autoware
{
namespace validation
{
template <typename T>
void appendIssues(std::vector<T> & to, std::vector<T> && from)
{
  to.insert(to.end(), std::make_move_iterator(from.begin()), std::make_move_iterator(from.end()));
}

template <typename T>
void checkPrimitivesType(
  std::vector<T> & in_vec, const std::string & expected_type,
  const lanelet::validation::Issue & issue, lanelet::validation::Issues & issues)
{
  for (auto iter = in_vec.begin(); iter != in_vec.end(); ++iter) {
    const auto & item = *iter;
    const auto & attrs = item.attributes();
    const auto & it = attrs.find(lanelet::AttributeName::Type);
    if (it == attrs.end() || it->second != expected_type) {
      issues.emplace_back(issue.severity, issue.primitive, item.id(), issue.message);
      const auto new_it = in_vec.erase(iter);
      if (new_it != in_vec.end()) {
        iter = new_it;
      } else {
        break;
      }
    }
  }
}

template <typename T>
void checkPrimitivesType(
  std::vector<T> & in_vec, const std::string & expected_type, const std::string & expected_subtype,
  const lanelet::validation::Issue & issue, lanelet::validation::Issues & issues)
{
  for (auto iter = in_vec.begin(); iter != in_vec.end(); ++iter) {
    const auto & item = *iter;
    const auto & attrs = item.attributes();
    const auto & it = attrs.find(lanelet::AttributeName::Type);
    const auto & it_sub = attrs.find(lanelet::AttributeName::Subtype);
    if (
      it == attrs.end() || it->second != expected_type || it_sub == attrs.end() ||
      it_sub->second != expected_subtype) {
      issues.emplace_back(issue.severity, issue.primitive, item.id(), issue.message);
      const auto new_it = in_vec.erase(iter);
      if (new_it != in_vec.end()) {
        iter = new_it;
      } else {
        break;
      }
    }
  }
}

}  // namespace validation
}  // namespace autoware
}  // namespace lanelet

#endif  // LANELET2_EXTENSION__AUTOWARE_LANELET2_VALIDATION__UTILS_HPP_
