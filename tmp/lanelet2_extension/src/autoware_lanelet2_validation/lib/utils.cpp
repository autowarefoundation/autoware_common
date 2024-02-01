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

namespace lanelet
{
namespace validation
{

void appendIssues(
  std::vector<lanelet::validation::DetectedIssues> & to,
  std::vector<lanelet::validation::DetectedIssues> && from)
{
  to.insert(to.end(), std::make_move_iterator(from.begin()), std::make_move_iterator(from.end()));
}
void appendIssues(lanelet::validation::Issues & to, lanelet::validation::Issues && from)
{
  to.insert(to.end(), std::make_move_iterator(from.begin()), std::make_move_iterator(from.end()));
}

}  // namespace validation
}  // namespace lanelet
