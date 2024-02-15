// Copyright 2022 Tier IV, Inc.
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

#ifndef AUTOWARE_UTILS__ROS__UUID_HELPER_HPP_
#define AUTOWARE_UTILS__ROS__UUID_HELPER_HPP_

#include <unique_identifier_msgs/msg/uuid.hpp>

#include <boost/uuid/uuid.hpp>

#include <algorithm>
#include <random>
#include <string>

namespace autoware_utils
{
inline unique_identifier_msgs::msg::UUID generate_uuid()
{
  // Generate random number
  unique_identifier_msgs::msg::UUID uuid;
  std::mt19937 gen(std::random_device{}());
  std::independent_bits_engine<std::mt19937, 8, uint8_t> bit_eng(gen);
  std::generate(uuid.uuid.begin(), uuid.uuid.end(), bit_eng);

  return uuid;
}
inline unique_identifier_msgs::msg::UUID generate_default_uuid()
{
  // Generate UUID with all zeros
  unique_identifier_msgs::msg::UUID default_uuid;
  // Use std::generate to fill the UUID with zeros
  std::generate(default_uuid.uuid.begin(), default_uuid.uuid.end(), []() { return 0; });

  return default_uuid;
}
inline std::string to_hex_string(const unique_identifier_msgs::msg::UUID & id)
{
  std::stringstream ss;
  for (auto i = 0; i < 16; ++i) {
    ss << std::hex << std::setfill('0') << std::setw(2) << +id.uuid[i];
  }
  return ss.str();
}

inline boost::uuids::uuid to_boost_uuid(const unique_identifier_msgs::msg::UUID & id)
{
  boost::uuids::uuid boost_uuid{};
  std::copy(id.uuid.begin(), id.uuid.end(), boost_uuid.begin());
  return boost_uuid;
}

inline unique_identifier_msgs::msg::UUID to_uuid_msg(const boost::uuids::uuid & id)
{
  unique_identifier_msgs::msg::UUID ros_uuid{};
  std::copy(id.begin(), id.end(), ros_uuid.uuid.begin());
  return ros_uuid;
}

}  // namespace autoware_utils

#endif  // AUTOWARE_UTILS__ROS__UUID_HELPER_HPP_
