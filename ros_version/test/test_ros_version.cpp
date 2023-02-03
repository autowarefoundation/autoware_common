// Copyright 2023 Tier IV, Inc.
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

#include <rclcpp/rclcpp.hpp>
#include <ros_version/ros_version.hpp>

#include <std_srvs/srv/trigger.hpp>

#include <gtest/gtest.h>
#include <rclcpp/version.h>

#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <string>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;
using std_srvs::srv::Trigger;

class Minimal : public rclcpp::Node
{
public:
  Minimal() : Node("minimal")
  {
// APIs taking rclcpp::QoS objects are only available in ROS 2 Iron and higher
#if RCLCPP_VERSION_MAJOR >= 18
    const auto qos = rclcpp::ServicesQoS();
    std::cerr << "this version is not yet supported in current version" << std::cerr;
#else
    const auto qos = rmw_qos_profile_services_default;
#endif
    auto test_service =
      create_service<Trigger>("/test_service", std::bind(&Minimal::testService, this, _1, _2), qos);
    auto client_test = create_client<Trigger>("/test_service", qos);
    auto req = std::make_shared<Trigger::Request>();
    client_test->async_send_request(
      req, [this]([[maybe_unused]] rclcpp::Client<Trigger>::SharedFuture result) {});
  }

  void testService(
    [[maybe_unused]] const Trigger::Request::SharedPtr req,
    [[maybe_unused]] const Trigger::Response::SharedPtr res)
  {
  }

private:
};

TEST(test, nominal)
{
  rclcpp::init(0, nullptr);
  rclcpp::spin_some(std::make_shared<Minimal>());
  rclcpp::shutdown();
}

int main(int argc, char * argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
