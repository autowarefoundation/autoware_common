// Copyright 2024 TIER IV, Inc.
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


#include "autoware_utils/ros/uuid_helper.hpp"

#include <boost/uuid/uuid_generators.hpp> 
#include <gtest/gtest.h>


TEST(UUIDHelperTest, GenerateUUID)
{
    // Generate two UUIDs and ensure they are all different

    unique_identifier_msgs::msg::UUID uuid1 = autoware_utils::generateUUID();
    unique_identifier_msgs::msg::UUID uuid2 = autoware_utils::generateUUID();

    EXPECT_FALSE(uuid1 == uuid2) << "Duplicate UUID generated: " << autoware_utils::toHexString(uuid2);
}

TEST(UUIDHelperTest, ToHexString)
{
    unique_identifier_msgs::msg::UUID uuid;
    // Populate the UUID with some values
    std::fill(uuid.uuid.begin(), uuid.uuid.end(), 0x42);

    std::string hexString = autoware_utils::toHexString(uuid);

    // Check if the generated hex string is correct
    EXPECT_EQ(hexString, "42424242424242424242424242424242");
}

TEST(UUIDHelperTest, ToBoostUUID)
{
    unique_identifier_msgs::msg::UUID uuid;
    // Populate the UUID with some values
    std::fill(uuid.uuid.begin(), uuid.uuid.end(), 0x42);

    boost::uuids::uuid boostUUID;
    std::fill(boostUUID.begin(), boostUUID.end(), 0x42);

    // Check if the conversion from ROS UUID to Boost UUID is correct
    EXPECT_TRUE(boostUUID == autoware_utils::toBoostUUID(uuid));
}

TEST(UUIDHelperTest, ToUUIDMsg)
{
    boost::uuids::random_generator generator;
    boost::uuids::uuid boostUUID = generator();
    unique_identifier_msgs::msg::UUID rosUUID = autoware_utils::toUUIDMsg(boostUUID);

    // Check if the conversion from Boost UUID to ROS UUID is correct
    EXPECT_TRUE(std::equal(boostUUID.begin(), boostUUID.end(), rosUUID.uuid.begin()));
}
