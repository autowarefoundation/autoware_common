#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>

#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_core/primitives/Lanelet.h>

#include <iostream>
#include <string>

namespace bp = boost::python;

namespace
{
// for handling functions with ros message type
lanelet::ArcCoordinates getArcCoordinates(
  const lanelet::ConstLanelets & lanelet_sequence, const std::string & pose_byte)
{
  rclcpp::SerializedMessage serialized_msg;
  static constexpr size_t message_header_length = 8u;
  serialized_msg.reserve(message_header_length + pose_byte.size());
  serialized_msg.get_rcl_serialized_message().buffer_length = pose_byte.size();
  for (size_t i = 0; i < pose_byte.size(); ++i) {
    serialized_msg.get_rcl_serialized_message().buffer[i] = pose_byte[i];
  }
  geometry_msgs::msg::Pose pose;
  static rclcpp::Serialization<geometry_msgs::msg::Pose> serializer;
  serializer.deserialize_message(&serialized_msg, &pose);
  return lanelet::utils::getArcCoordinates(lanelet_sequence, pose);
}

double getLaneletAngle(const lanelet::ConstLanelet & lanelet, const std::string & point_byte)
{
  rclcpp::SerializedMessage serialized_msg;
  static constexpr size_t message_header_length = 8u;
  serialized_msg.reserve(message_header_length + point_byte.size());
  serialized_msg.get_rcl_serialized_message().buffer_length = point_byte.size();
  for (size_t i = 0; i < point_byte.size(); ++i) {
    serialized_msg.get_rcl_serialized_message().buffer[i] = point_byte[i];
  }
  geometry_msgs::msg::Point point;
  static rclcpp::Serialization<geometry_msgs::msg::Point> serializer;
  serializer.deserialize_message(&serialized_msg, &point);
  return lanelet::utils::getLaneletAngle(lanelet, point);
}

bool isInLanelet(
  const std::string & pose_byte, const lanelet::ConstLanelet & lanelet, const double radius = 0.0)
{
  rclcpp::SerializedMessage serialized_msg;
  static constexpr size_t message_header_length = 8u;
  serialized_msg.reserve(message_header_length + pose_byte.size());
  serialized_msg.get_rcl_serialized_message().buffer_length = pose_byte.size();
  for (size_t i = 0; i < pose_byte.size(); ++i) {
    serialized_msg.get_rcl_serialized_message().buffer[i] = pose_byte[i];
  }
  geometry_msgs::msg::Pose pose;
  static rclcpp::Serialization<geometry_msgs::msg::Pose> serializer;
  serializer.deserialize_message(&serialized_msg, &pose);
  return lanelet::utils::isInLanelet(pose, lanelet, radius);
}

std::vector<char> getClosestCenterPose(
  const lanelet::ConstLanelet & lanelet, const std::string & search_point_byte)
{
  rclcpp::SerializedMessage serialized_point_msg;
  static constexpr size_t message_header_length = 8u;
  serialized_point_msg.reserve(message_header_length + search_point_byte.size());
  serialized_point_msg.get_rcl_serialized_message().buffer_length = search_point_byte.size();
  for (size_t i = 0; i < search_point_byte.size(); ++i) {
    serialized_point_msg.get_rcl_serialized_message().buffer[i] = search_point_byte[i];
  }
  geometry_msgs::msg::Point search_point;
  static rclcpp::Serialization<geometry_msgs::msg::Point> serializer_point;
  serializer_point.deserialize_message(&serialized_point_msg, &search_point);
  std::cout << search_point.x << ", " << search_point.y << ", " << search_point.z << std::endl;
  // ここまで正しい
  const geometry_msgs::msg::Pose pose = lanelet::utils::getClosestCenterPose(lanelet, search_point);
  std::cout << pose.position.x << ", " << pose.position.y << ", " << pose.position.z << std::endl;
  std::cout << pose.orientation.x << ", " << pose.orientation.y << ", " << pose.orientation.z
            << ", " << pose.orientation.w << std::endl;
  // serializationも間違っていないはずなのだが,
  // ここ以降かutilities.pyのgetClosestCenterPoseが間違っている
  static rclcpp::Serialization<geometry_msgs::msg::Pose> serializer_pose;
  rclcpp::SerializedMessage serialized_pose_msg;
  serializer_pose.serialize_message(&pose, &serialized_pose_msg);
  std::vector<char> pose_byte;
  for (size_t i = 0; i < serialized_pose_msg.size(); ++i) {
    pose_byte.push_back(serialized_pose_msg.get_rcl_serialized_message().buffer[i]);
  }
  return pose_byte;
}

double getLateralDistanceToCenterline(
  const lanelet::ConstLanelet & lanelet, const std::string & pose_byte)
{
  rclcpp::SerializedMessage serialized_msg;
  static constexpr size_t message_header_length = 8u;
  serialized_msg.reserve(message_header_length + pose_byte.size());
  serialized_msg.get_rcl_serialized_message().buffer_length = pose_byte.size();
  for (size_t i = 0; i < pose_byte.size(); ++i) {
    serialized_msg.get_rcl_serialized_message().buffer[i] = pose_byte[i];
  }
  geometry_msgs::msg::Pose pose;
  static rclcpp::Serialization<geometry_msgs::msg::Pose> serializer;
  serializer.deserialize_message(&serialized_msg, &pose);
  return lanelet::utils::getLateralDistanceToCenterline(lanelet, pose);
}

double getLateralDistanceToClosestLanelet(
  const lanelet::ConstLanelets & lanelet_sequence, const std::string & pose_byte)
{
  rclcpp::SerializedMessage serialized_msg;
  static constexpr size_t message_header_length = 8u;
  serialized_msg.reserve(message_header_length + pose_byte.size());
  serialized_msg.get_rcl_serialized_message().buffer_length = pose_byte.size();
  for (size_t i = 0; i < pose_byte.size(); ++i) {
    serialized_msg.get_rcl_serialized_message().buffer[i] = pose_byte[i];
  }
  geometry_msgs::msg::Pose pose;
  static rclcpp::Serialization<geometry_msgs::msg::Pose> serializer;
  serializer.deserialize_message(&serialized_msg, &pose);
  return lanelet::utils::getLateralDistanceToClosestLanelet(lanelet_sequence, pose);
}

lanelet::ConstLanelets getLaneletsWithinRange(
  const lanelet::ConstLanelets & lanelets, const std::string & point_byte, const double range)
{
  rclcpp::SerializedMessage serialized_msg;
  static constexpr size_t message_header_length = 8u;
  serialized_msg.reserve(message_header_length + point_byte.size());
  serialized_msg.get_rcl_serialized_message().buffer_length = point_byte.size();
  for (size_t i = 0; i < point_byte.size(); ++i) {
    serialized_msg.get_rcl_serialized_message().buffer[i] = point_byte[i];
  }
  geometry_msgs::msg::Point point;
  static rclcpp::Serialization<geometry_msgs::msg::Point> serializer;
  serializer.deserialize_message(&serialized_msg, &point);
  return lanelet::utils::query::getLaneletsWithinRange(lanelets, point, range);
}

lanelet::ConstLanelets getLaneChangeableNeighbors(
  const lanelet::routing::RoutingGraphPtr & graph, const lanelet::ConstLanelets & road_lanelets,
  const std::string & point_byte)
{
  rclcpp::SerializedMessage serialized_msg;
  static constexpr size_t message_header_length = 8u;
  serialized_msg.reserve(message_header_length + point_byte.size());
  serialized_msg.get_rcl_serialized_message().buffer_length = point_byte.size();
  for (size_t i = 0; i < point_byte.size(); ++i) {
    serialized_msg.get_rcl_serialized_message().buffer[i] = point_byte[i];
  }
  geometry_msgs::msg::Point point;
  static rclcpp::Serialization<geometry_msgs::msg::Point> serializer;
  serializer.deserialize_message(&serialized_msg, &point);
  return lanelet::utils::query::getLaneChangeableNeighbors(graph, road_lanelets, point);
}

lanelet::ConstLanelets getAllNeighbors(
  const lanelet::routing::RoutingGraphPtr & graph, const lanelet::ConstLanelets & road_lanelets,
  const std::string & point_byte)
{
  rclcpp::SerializedMessage serialized_msg;
  static constexpr size_t message_header_length = 8u;
  serialized_msg.reserve(message_header_length + point_byte.size());
  serialized_msg.get_rcl_serialized_message().buffer_length = point_byte.size();
  for (size_t i = 0; i < point_byte.size(); ++i) {
    serialized_msg.get_rcl_serialized_message().buffer[i] = point_byte[i];
  }
  geometry_msgs::msg::Point point;
  static rclcpp::Serialization<geometry_msgs::msg::Point> serializer;
  serializer.deserialize_message(&serialized_msg, &point);
  return lanelet::utils::query::getAllNeighbors(graph, road_lanelets, point);
}

bool getClosestLaneletWithConstrains(
  const lanelet::ConstLanelets & lanelets, const std::string & pose_byte,
  lanelet::ConstLanelet * closest_lanelet_ptr,
  const double dist_threshold = std::numeric_limits<double>::max(),
  const double yaw_threshold = std::numeric_limits<double>::max())
{
  rclcpp::SerializedMessage serialized_msg;
  static constexpr size_t message_header_length = 8u;
  serialized_msg.reserve(message_header_length + pose_byte.size());
  serialized_msg.get_rcl_serialized_message().buffer_length = pose_byte.size();
  for (size_t i = 0; i < pose_byte.size(); ++i) {
    serialized_msg.get_rcl_serialized_message().buffer[i] = pose_byte[i];
  }
  geometry_msgs::msg::Pose pose;
  static rclcpp::Serialization<geometry_msgs::msg::Pose> serializer;
  serializer.deserialize_message(&serialized_msg, &pose);
  return lanelet::utils::query::getClosestLaneletWithConstrains(
    lanelets, pose, closest_lanelet_ptr, dist_threshold, yaw_threshold);
}

bool getCurrentLanelets_point(
  const lanelet::ConstLanelets & lanelets, const std::string & point_byte,
  lanelet::ConstLanelets * current_lanelets_ptr)
{
  rclcpp::SerializedMessage serialized_msg;
  static constexpr size_t message_header_length = 8u;
  serialized_msg.reserve(message_header_length + point_byte.size());
  serialized_msg.get_rcl_serialized_message().buffer_length = point_byte.size();
  for (size_t i = 0; i < point_byte.size(); ++i) {
    serialized_msg.get_rcl_serialized_message().buffer[i] = point_byte[i];
  }
  geometry_msgs::msg::Point point;
  static rclcpp::Serialization<geometry_msgs::msg::Point> serializer;
  serializer.deserialize_message(&serialized_msg, &point);
  return lanelet::utils::query::getCurrentLanelets(lanelets, point, current_lanelets_ptr);
}

bool getCurrentLanelets_pose(
  const lanelet::ConstLanelets & lanelets, const std::string & pose_byte,
  lanelet::ConstLanelets * current_lanelets_ptr)
{
  rclcpp::SerializedMessage serialized_msg;
  static constexpr size_t message_header_length = 8u;
  serialized_msg.reserve(message_header_length + pose_byte.size());
  serialized_msg.get_rcl_serialized_message().buffer_length = pose_byte.size();
  for (size_t i = 0; i < pose_byte.size(); ++i) {
    serialized_msg.get_rcl_serialized_message().buffer[i] = pose_byte[i];
  }
  geometry_msgs::msg::Pose pose;
  static rclcpp::Serialization<geometry_msgs::msg::Pose> serializer;
  serializer.deserialize_message(&serialized_msg, &pose);
  return lanelet::utils::query::getCurrentLanelets(lanelets, pose, current_lanelets_ptr);
}
}  // namespace

// for handling functions with default arguments
BOOST_PYTHON_FUNCTION_OVERLOADS(
  generateFineCenterline_overload, lanelet::utils::generateFineCenterline, 1, 2)
BOOST_PYTHON_FUNCTION_OVERLOADS(
  getCenterlineWithOffset_overload, lanelet::utils::getCenterlineWithOffset, 2, 3)
BOOST_PYTHON_FUNCTION_OVERLOADS(
  getRightBoundWithOffset_overload, lanelet::utils::getRightBoundWithOffset, 2, 3)
BOOST_PYTHON_FUNCTION_OVERLOADS(
  getLeftBoundWithOffset_overload, lanelet::utils::getLeftBoundWithOffset, 2, 3)
BOOST_PYTHON_FUNCTION_OVERLOADS(
  overwriteLaneletsCenterline_overload, lanelet::utils::overwriteLaneletsCenterline, 1, 3)
BOOST_PYTHON_FUNCTION_OVERLOADS(isInLanelet_overload, ::isInLanelet, 2, 3)
BOOST_PYTHON_FUNCTION_OVERLOADS(
  getPrecedingLaneletSequences_overload, lanelet::utils::query::getPrecedingLaneletSequences, 3, 4)
BOOST_PYTHON_FUNCTION_OVERLOADS(
  getClosestLaneletWithConstrains_overload, ::getClosestLaneletWithConstrains, 3, 5)

BOOST_PYTHON_MODULE(_lanelet2_extension_python_boost_python_utility)
{
  // utilities.cpp
  bp::def("combineLaneletsShape", lanelet::utils::combineLaneletsShape);
  bp::def(
    "generateFineCenterline", lanelet::utils::generateFineCenterline,
    generateFineCenterline_overload());
  bp::def(
    "getCenterlineWithOffset", lanelet::utils::getCenterlineWithOffset,
    getCenterlineWithOffset_overload());
  bp::def(
    "getRightBoundWithOffset", lanelet::utils::getRightBoundWithOffset,
    getRightBoundWithOffset_overload());
  bp::def(
    "getLeftBoundWithOffset", lanelet::utils::getLeftBoundWithOffset,
    getLeftBoundWithOffset_overload());
  bp::def("getExpandedLanelet", lanelet::utils::getExpandedLanelet);
  bp::def("getExpandedLanelets", lanelet::utils::getExpandedLanelets);
  bp::def(
    "overwriteLaneletsCenterline", lanelet::utils::overwriteLaneletsCenterline,
    overwriteLaneletsCenterline_overload());
  bp::def<double(const lanelet::ConstLanelet &)>(
    "getLaneletLength2d", lanelet::utils::getLaneletLength2d);
  bp::def<double(const lanelet::ConstLanelet &)>(
    "getLaneletLength3d", lanelet::utils::getLaneletLength3d);
  bp::def<double(const lanelet::ConstLanelets &)>(
    "getLaneletLength2d", lanelet::utils::getLaneletLength2d);
  bp::def<double(const lanelet::ConstLanelets &)>(
    "getLaneletLength3d", lanelet::utils::getLaneletLength3d);
  bp::def("getArcCoordinates", ::getArcCoordinates);  // depends ros msg
  bp::def("getClosestSegment", lanelet::utils::getClosestSegment);
  bp::def("getPolygonFromArcLength", lanelet::utils::getPolygonFromArcLength);
  bp::def("getLaneletAngle", ::getLaneletAngle);                  // depends on ros msg
  bp::def("isInLanelet", ::isInLanelet, isInLanelet_overload());  // depends ros msg
  bp::class_<std::vector<char>>("bytes").def(bp::vector_indexing_suite<std::vector<char>>());
  bp::def("getClosestCenterPose", getClosestCenterPose);                        // depends ros msg
  bp::def("getLateralDistanceToCenterline", ::getLateralDistanceToCenterline);  // depends ros msg
  bp::def(
    "getLateralDistanceToClosestLanelet", ::getLateralDistanceToClosestLanelet);  // depends ros msg

  // query.cpp
  bp::def("laneletLayer", lanelet::utils::query::laneletLayer);
  bp::def("subtypeLanelets", lanelet::utils::query::subtypeLanelets);
  bp::def("crosswalkLanelets", lanelet::utils::query::crosswalkLanelets);
  bp::def("walkwayLanelets", lanelet::utils::query::walkwayLanelets);
  bp::def("roadLanelets", lanelet::utils::query::roadLanelets);
  bp::def("shoulderLanelets", lanelet::utils::query::shoulderLanelets);
  bp::def("trafficLights", lanelet::utils::query::trafficLights);
  bp::def("autowareTrafficLights", lanelet::utils::query::autowareTrafficLights);
  bp::def("detectionAreas", lanelet::utils::query::detectionAreas);
  bp::def("noStoppingAreas", lanelet::utils::query::noStoppingAreas);
  bp::def("noParkingAreas", lanelet::utils::query::noParkingAreas);
  bp::def("speedBumps", lanelet::utils::query::speedBumps);
  bp::def("crosswalks", lanelet::utils::query::crosswalks);
  bp::def("curbstones", lanelet::utils::query::curbstones);
  bp::def("getAllPolygonsByType", lanelet::utils::query::getAllPolygonsByType);
  bp::def("getAllObstaclePolygons", lanelet::utils::query::getAllObstaclePolygons);
  bp::def("getAllParkingLots", lanelet::utils::query::getAllParkingLots);
  bp::def("getAllPartitions", lanelet::utils::query::getAllPartitions);
  bp::def("getAllFences", lanelet::utils::query::getAllFences);
  bp::def("getAllPedestrianMarkings", lanelet::utils::query::getAllPedestrianMarkings);
  bp::def("getAllParkingSpaces", lanelet::utils::query::getAllParkingSpaces);
  bp::def<lanelet::ConstLineStrings3d(
    const lanelet::ConstLanelet &, const lanelet::LaneletMapConstPtr &)>(
    "getLinkedParkingSpaces", lanelet::utils::query::getLinkedParkingSpaces);
  bp::def<lanelet::ConstLineStrings3d(
    const lanelet::ConstLanelet &, const lanelet::ConstLineStrings3d &,
    const lanelet::ConstPolygons3d &)>(
    "getLinkedParkingSpaces", lanelet::utils::query::getLinkedParkingSpaces);
  bp::def<bool(
    const lanelet::ConstLineString3d &, const lanelet::ConstLanelets &,
    const lanelet::ConstPolygons3d &, lanelet::ConstLanelet *)>(
    "getLinkedLanelet", lanelet::utils::query::getLinkedLanelet);
  bp::def<bool(
    const lanelet::ConstLineString3d &, const lanelet::LaneletMapConstPtr &,
    lanelet::ConstLanelet *)>("getLinkedLanelet", lanelet::utils::query::getLinkedLanelet);
  bp::def<lanelet::ConstLanelets(
    const lanelet::ConstLineString3d &, const lanelet::ConstLanelets &,
    const lanelet::ConstPolygons3d &)>(
    "getLinkedLanelets", lanelet::utils::query::getLinkedLanelets);
  bp::def<lanelet::ConstLanelets(
    const lanelet::ConstLineString3d &, const lanelet::LaneletMapConstPtr &)>(
    "getLinkedLanelets", lanelet::utils::query::getLinkedLanelets);
  bp::def<bool(
    const lanelet::ConstLanelet &, const lanelet::ConstPolygons3d &, lanelet::ConstPolygon3d *)>(
    "getLinkedParkingLot", lanelet::utils::query::getLinkedParkingLot);
  bp::def<bool(
    const lanelet::BasicPoint2d &, const lanelet::ConstPolygons3d &, lanelet::ConstPolygon3d *)>(
    "getLinkedParkingLot", lanelet::utils::query::getLinkedParkingLot);
  bp::def<bool(
    const lanelet::ConstLineString3d &, const lanelet::ConstPolygons3d &,
    lanelet::ConstPolygon3d *)>("getLinkedParkingLot", lanelet::utils::query::getLinkedParkingLot);
  bp::def("stopLinesLanelets", lanelet::utils::query::stopLinesLanelets);
  bp::def("stopLinesLanelet", lanelet::utils::query::stopLinesLanelet);
  bp::def("stopSignStopLines", lanelet::utils::query::stopSignStopLines);
  bp::def<lanelet::ConstLanelets(
    const lanelet::ConstLanelets &, const lanelet::BasicPoint2d &, const double)>(
    "getLaneletsWithinRange", lanelet::utils::query::getLaneletsWithinRange);
  bp::def<lanelet::ConstLanelets(
    const lanelet::ConstLanelets &, const std::string &, const double)>(
    "getLaneletsWithinRange", ::getLaneletsWithinRange);  // depends on ros msg
  bp::def<lanelet::ConstLanelets(
    const lanelet::routing::RoutingGraphPtr &, const lanelet::ConstLanelet &)>(
    "getLaneChangeableNeighbors", lanelet::utils::query::getLaneChangeableNeighbors);
  bp::def<lanelet::ConstLanelets(
    const lanelet::routing::RoutingGraphPtr &, const lanelet::ConstLanelets &,
    const std::string &)>(
    "getLaneChangeableNeighbors", ::getLaneChangeableNeighbors);  // depends on ros msg
  bp::def<lanelet::ConstLanelets(
    const lanelet::routing::RoutingGraphPtr &, const lanelet::ConstLanelet &)>(
    "getAllNeighbors", lanelet::utils::query::getAllNeighbors);
  bp::def<lanelet::ConstLanelets(
    const lanelet::routing::RoutingGraphPtr &, const lanelet::ConstLanelets &,
    const std::string &)>("getAllNeighbors", ::getAllNeighbors);
  bp::def("getAllNeighborsLeft", lanelet::utils::query::getAllNeighborsLeft);
  bp::def("getAllNeighborsRight", lanelet::utils::query::getAllNeighborsRight);
  bp::def(
    "getClosestLaneletWithConstrains", ::getClosestLaneletWithConstrains,
    getClosestLaneletWithConstrains_overload());                    // depends on ros msg
  bp::def("getCurrentLanelets_point", ::getCurrentLanelets_point);  // depends on ros msg
  bp::def("getCurrentLanelets_pose", ::getCurrentLanelets_pose);    // depends on ros msg
  bp::def("getSucceedingLaneletSequences", lanelet::utils::query::getSucceedingLaneletSequences);
  bp::def(
    "getPrecedingLaneletSequences", lanelet::utils::query::getPrecedingLaneletSequences,
    getPrecedingLaneletSequences_overload());
}
