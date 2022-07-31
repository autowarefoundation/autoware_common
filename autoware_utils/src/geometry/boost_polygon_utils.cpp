// Copyright 2022 TIER IV, Inc.
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

#include "tier4_autoware_utils/geometry/boost_polygon_utils.hpp"

#include "tier4_autoware_utils/geometry/geometry.hpp"

#include <tf2/utils.h>

namespace
{
namespace bg = boost::geometry;
using tier4_autoware_utils::Point2d;
using tier4_autoware_utils::Polygon2d;

void appendPointToPolygon(Polygon2d & polygon, const geometry_msgs::msg::Point & geom_point)
{
  Point2d point;
  point.x() = geom_point.x;
  point.y() = geom_point.y;

  bg::append(polygon.outer(), point);
}

void appendPointToPolygon(Polygon2d & polygon, const Point2d point)
{
  bg::append(polygon.outer(), point);
}

/*
 * NOTE: Area is negative when footprint.points is clock wise.
 *       Area is positive when footprint.points is anti clock wise.
 */
double getPolygonArea(const geometry_msgs::msg::Polygon & footprint)
{
  double area = 0.0;

  for (size_t i = 0; i < footprint.points.size(); ++i) {
    size_t j = (i + 1) % footprint.points.size();
    area += 0.5 * (footprint.points.at(i).x * footprint.points.at(j).y -
                   footprint.points.at(j).x * footprint.points.at(i).y);
  }

  return area;
}

double getRectangleArea(const geometry_msgs::msg::Vector3 & dimensions)
{
  return static_cast<double>(dimensions.x * dimensions.y);
}

double getCircleArea(const geometry_msgs::msg::Vector3 & dimensions)
{
  return static_cast<double>((dimensions.x / 2.0) * (dimensions.x / 2.0) * M_PI);
}
}  // namespace

namespace tier4_autoware_utils
{
bool isClockwise(const Polygon2d & polygon)
{
  const int n = polygon.outer().size();
  const double x_offset = polygon.outer().at(0).x();
  const double y_offset = polygon.outer().at(0).y();
  double sum = 0.0;
  for (std::size_t i = 0; i < polygon.outer().size(); ++i) {
    sum +=
      (polygon.outer().at(i).x() - x_offset) * (polygon.outer().at((i + 1) % n).y() - y_offset) -
      (polygon.outer().at(i).y() - y_offset) * (polygon.outer().at((i + 1) % n).x() - x_offset);
  }

  return sum < 0.0;
}

Polygon2d inverseClockwise(const Polygon2d & polygon)
{
  auto output_polygon = polygon;
  boost::geometry::reverse(output_polygon);
  return output_polygon;
}

geometry_msgs::msg::Polygon rotatePolygon(
  const geometry_msgs::msg::Polygon & polygon, const double & angle)
{
  const double cos = std::cos(angle);
  const double sin = std::sin(angle);
  geometry_msgs::msg::Polygon rotated_polygon;
  for (const auto & point : polygon.points) {
    auto rotated_point = point;
    rotated_point.x = cos * point.x - sin * point.y;
    rotated_point.y = sin * point.x + cos * point.y;
    rotated_polygon.points.push_back(rotated_point);
  }
  return rotated_polygon;
}

Polygon2d toPolygon2d(
  const geometry_msgs::msg::Pose & pose, const autoware_auto_perception_msgs::msg::Shape & shape)
{
  Polygon2d polygon;

  if (shape.type == autoware_auto_perception_msgs::msg::Shape::BOUNDING_BOX) {
    const auto point0 = tier4_autoware_utils::calcOffsetPose(
                          pose, shape.dimensions.x / 2.0, shape.dimensions.y / 2.0, 0.0)
                          .position;
    const auto point1 = tier4_autoware_utils::calcOffsetPose(
                          pose, -shape.dimensions.x / 2.0, shape.dimensions.y / 2.0, 0.0)
                          .position;
    const auto point2 = tier4_autoware_utils::calcOffsetPose(
                          pose, -shape.dimensions.x / 2.0, -shape.dimensions.y / 2.0, 0.0)
                          .position;
    const auto point3 = tier4_autoware_utils::calcOffsetPose(
                          pose, shape.dimensions.x / 2.0, -shape.dimensions.y / 2.0, 0.0)
                          .position;

    appendPointToPolygon(polygon, point0);
    appendPointToPolygon(polygon, point1);
    appendPointToPolygon(polygon, point2);
    appendPointToPolygon(polygon, point3);
  } else if (shape.type == autoware_auto_perception_msgs::msg::Shape::CYLINDER) {
    const double radius = shape.dimensions.x / 2.0;
    constexpr int circle_discrete_num = 6;
    for (int i = 0; i < circle_discrete_num; ++i) {
      geometry_msgs::msg::Point point;
      point.x = std::cos(
                  (static_cast<double>(i) / static_cast<double>(circle_discrete_num)) * 2.0 * M_PI +
                  M_PI / static_cast<double>(circle_discrete_num)) *
                  radius +
                pose.position.x;
      point.y = std::sin(
                  (static_cast<double>(i) / static_cast<double>(circle_discrete_num)) * 2.0 * M_PI +
                  M_PI / static_cast<double>(circle_discrete_num)) *
                  radius +
                pose.position.y;
      appendPointToPolygon(polygon, point);
    }
  } else if (shape.type == autoware_auto_perception_msgs::msg::Shape::POLYGON) {
    const double poly_yaw = tf2::getYaw(pose.orientation);
    const auto rotated_footprint = rotatePolygon(shape.footprint, poly_yaw);
    for (const auto rel_point : rotated_footprint.points) {
      geometry_msgs::msg::Point abs_point;
      abs_point.x = pose.position.x + rel_point.x;
      abs_point.y = pose.position.y + rel_point.y;

      appendPointToPolygon(polygon, abs_point);
    }
  } else {
    throw std::logic_error("The shape type is not supported in tier4_autoware_utils.");
  }

  // NOTE: push back the first point in order to close polygon
  if (polygon.outer().size() > 0) {
    appendPointToPolygon(polygon, polygon.outer().front());
  }

  return isClockwise(polygon) ? polygon : inverseClockwise(polygon);
}

double getArea(const autoware_auto_perception_msgs::msg::Shape & shape)
{
  if (shape.type == autoware_auto_perception_msgs::msg::Shape::BOUNDING_BOX) {
    return getRectangleArea(shape.dimensions);
  } else if (shape.type == autoware_auto_perception_msgs::msg::Shape::CYLINDER) {
    return getCircleArea(shape.dimensions);
  } else if (shape.type == autoware_auto_perception_msgs::msg::Shape::POLYGON) {
    return getPolygonArea(shape.footprint);
  }

  throw std::logic_error("The shape type is not supported in tier4_autoware_utils.");
}
}  // namespace tier4_autoware_utils
